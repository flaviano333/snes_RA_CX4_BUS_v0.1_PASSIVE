#include "cx4_active_bus.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

/*
 * CX4 BUS v0.2 ACTIVE SELFTEST
 * ---------------------------
 * SNES-side interface model only. This deliberately does NOT execute the
 * HG51B S169 DSP yet. It implements the external 3 KiB DRAM + MMIO/GPR
 * read/write behaviour needed to validate that the RP2350B can actually act
 * as a cartridge-side device on the real bus.
 *
 * Reference model for the memory map/MMIO semantics: the permissively-licensed
 * ares-derived CX4 core in RetroPortingToolKit/snesrecomp (ISC portion).
 * No game ROM or proprietary CX4 firmware is embedded here.
 */

#define PIN_PHI2       0u
#define PIN_WR         1u
#define PIN_RD         35u

#define DATA_MASK ((1u << 2) | (1u << 3) | (1u << 4) | \
                   (1u << 6) | (1u << 7) | (1u << 8) | \
                   (1u << 9) | (1u << 10))

#define PHI2_MASK (1u << PIN_PHI2)
#define WR_MASK   (1u << PIN_WR)
#define RD_HI_MASK (1u << (PIN_RD - 32u))

/* Address pins used by the fast Cx4 decode. */
#define A13_MASK (1u << 25)  /* GP25 */
#define A14_MASK (1u << 27)  /* GP27 */
#define A15_MASK (1u << 28)  /* GP28 */
#define A22_HI_MASK (1u << (36u - 32u)) /* GP36 */

/* Visible Cx4 state. */
typedef struct {
    uint8_t dram[0x0c00];       /* 3 KiB, mirrored at $6000/$7000 */
    uint32_t gpr[16];           /* 24-bit R0-R15 */
    uint8_t vector[32];

    uint32_t dma_source;        /* 24-bit */
    uint16_t dma_length;
    uint32_t dma_target;        /* 24-bit */

    uint8_t cache_page;
    uint32_t cache_base;        /* 24-bit */
    uint8_t cache_lock[2];
    uint16_t cache_pb;          /* 15-bit */
    uint8_t cache_pc;

    uint8_t wait_ram;           /* 3-bit */
    uint8_t wait_rom;           /* 3-bit */
    uint8_t irq_inhibit;        /* $7F51 bit0 */
    uint8_t rom_mode;           /* $7F52 bit0 */
    uint8_t suspend_enable;
    uint8_t i_flag;

    /* Diagnostic counters. Written only by core1 except reset_count. */
    volatile uint64_t cpu_reads;
    volatile uint64_t cpu_writes;
    volatile uint64_t driven_reads;
    volatile uint64_t dram_reads;
    volatile uint64_t dram_writes;
    volatile uint64_t io_reads;
    volatile uint64_t io_writes;
    volatile uint64_t pc_starts;
    volatile uint64_t reset_vectors;
    volatile uint64_t reset_count;
    volatile uint32_t last_read_addr;
    volatile uint32_t last_write_addr;
    volatile uint8_t last_read_data;
    volatile uint8_t last_write_data;
} cx4bus_state_t;

static cx4bus_state_t s;
static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile uint8_t s_reset_arm = 0;

static inline void set24(uint32_t *r, unsigned byte, uint8_t v) {
    unsigned sh = byte * 8u;
    *r = (*r & ~(0xffu << sh)) | ((uint32_t)v << sh);
    *r &= 0xffffffu;
}

/* Inputs are always visible to SIO even when a pin's output mux is another
 * peripheral. D0-D7 are explicitly switched to SIO in cx4bus_init() so core1
 * can control OE during Cx4 reads; PIO can still observe their input state. */
static inline uint8_t raw_data(uint32_t lo) {
    return (uint8_t)(((lo >> 2) & 0x07u) | ((lo >> 3) & 0xf8u));
}

static inline uint32_t packed_data(uint8_t v) {
    return (((uint32_t)v & 0x07u) << 2) | (((uint32_t)v & 0xf8u) << 3);
}

static inline void data_release(void) {
    gpio_set_dir_in_masked(DATA_MASK);
    s_driving = false;
}

static inline void data_drive(uint8_t v) {
    /* Preload value first, then enable output: avoids a transient wrong byte. */
    gpio_put_masked(DATA_MASK, packed_data(v));
    gpio_set_dir_out_masked(DATA_MASK);
    s_driving = true;
}

/* Fast hardware decode of the ares mapping=0 Cx4 CPU window:
 *   (address & $40E000) == $006000
 * i.e. A22=0 and A15..A13=011. Bank A23/A21..A16 are don't-care.
 * This covers $6000-$7FFF in banks $00-$3F/$80-$BF.
 */
static inline bool raw_is_cx4(uint32_t lo, uint32_t hi) {
    if (hi & A22_HI_MASK) return false;
    if (lo & A15_MASK) return false;
    if (!(lo & A14_MASK)) return false;
    if (!(lo & A13_MASK)) return false;
    return true;
}

/* Only A0..A12 vary inside the $6000-$7FFF Cx4 window. */
static inline uint16_t raw_cx4_offset(uint32_t lo, uint32_t hi) {
    uint16_t off = 0x6000u;
    off |= (uint16_t)((lo >> 11) & 0x01ffu);          /* A0..A8 */
    off |= (uint16_t)(((lo >> 21) & 1u) << 9);       /* A9 */
    off |= (uint16_t)(((hi >> (40u - 32u)) & 1u) << 10); /* A10 GP40 */
    off |= (uint16_t)(((lo >> 23) & 1u) << 11);      /* A11 */
    off |= (uint16_t)(((lo >> 24) & 1u) << 12);      /* A12 */
    return off;
}

/* Full address is only used for diagnostics/reset-vector inference, never on
 * the latency-critical Cx4 response path. */
static inline uint32_t raw_address24(uint32_t lo, uint32_t hi) {
    uint32_t a = 0;
    a |= ((lo >> 11) & 0x1ffu);               /* A0..A8 */
    a |= ((lo >> 21) & 1u) << 9;              /* A9 */
    a |= ((hi >> 8) & 1u) << 10;              /* A10 GP40 */
    a |= ((lo >> 23) & 0x7u) << 11;           /* A11..A13 GP23..25 */
    a |= ((lo >> 27) & 0x1fu) << 14;          /* A14..A18 GP27..31 */
    a |= (hi & 0x7u) << 19;                   /* A19..A21 GP32..34 */
    a |= ((hi >> 4) & 1u) << 22;              /* A22 GP36 */
    a |= ((hi >> 5) & 1u) << 23;              /* A23 GP37 */
    return a & 0xffffffu;
}

static inline uint8_t io_read(uint16_t offset) {
    uint16_t a = (uint16_t)(0x7c00u | (offset & 0x03ffu));
    switch (a) {
        case 0x7f40: return (uint8_t)(s.dma_source & 0xffu);
        case 0x7f41: return (uint8_t)((s.dma_source >> 8) & 0xffu);
        case 0x7f42: return (uint8_t)((s.dma_source >> 16) & 0xffu);
        case 0x7f43: return (uint8_t)(s.dma_length & 0xffu);
        case 0x7f44: return (uint8_t)(s.dma_length >> 8);
        case 0x7f45: return (uint8_t)(s.dma_target & 0xffu);
        case 0x7f46: return (uint8_t)((s.dma_target >> 8) & 0xffu);
        case 0x7f47: return (uint8_t)((s.dma_target >> 16) & 0xffu);
        case 0x7f48: return s.cache_page & 1u;
        case 0x7f49: return (uint8_t)(s.cache_base & 0xffu);
        case 0x7f4a: return (uint8_t)((s.cache_base >> 8) & 0xffu);
        case 0x7f4b: return (uint8_t)((s.cache_base >> 16) & 0xffu);
        case 0x7f4c: return (uint8_t)((s.cache_lock[0] & 1u) | ((s.cache_lock[1] & 1u) << 1));
        case 0x7f4d: return (uint8_t)(s.cache_pb & 0xffu);
        case 0x7f4e: return (uint8_t)((s.cache_pb >> 8) & 0x7fu);
        case 0x7f4f: return s.cache_pc;
        case 0x7f50: return (uint8_t)((s.wait_ram & 7u) | ((s.wait_rom & 7u) << 4));
        case 0x7f51: return s.irq_inhibit & 1u;
        case 0x7f52: return s.rom_mode & 1u;

        /* Stub status: the DSP core is not present in v0.2, so a start request
         * completes immediately. This lets X2 continue its self-test and show
         * exactly which bus-facing sections already work instead of hanging. */
        case 0x7f53: case 0x7f54: case 0x7f55: case 0x7f56:
        case 0x7f57: case 0x7f59: case 0x7f5b: case 0x7f5c:
        case 0x7f5d: case 0x7f5e:
        case 0x7f5f:
            return (uint8_t)((s.suspend_enable ? 1u : 0u) | ((s.i_flag & 1u) << 1));
        default: break;
    }

    if (a >= 0x7f60u && a <= 0x7f7fu) return s.vector[a & 0x1fu];

    if ((a >= 0x7f80u && a <= 0x7fafu) ||
        (a >= 0x7fc0u && a <= 0x7fefu)) {
        uint32_t x = a & 0x3fu;
        uint32_t r = s.gpr[(x / 3u) & 0x0fu];
        return (uint8_t)((r >> ((x % 3u) * 8u)) & 0xffu);
    }

    return 0x00;
}

static inline void io_write(uint16_t offset, uint8_t data) {
    uint16_t a = (uint16_t)(0x7c00u | (offset & 0x03ffu));
    switch (a) {
        case 0x7f40: set24(&s.dma_source, 0, data); return;
        case 0x7f41: set24(&s.dma_source, 1, data); return;
        case 0x7f42: set24(&s.dma_source, 2, data); return;
        case 0x7f43: s.dma_length = (uint16_t)((s.dma_length & 0xff00u) | data); return;
        case 0x7f44: s.dma_length = (uint16_t)((s.dma_length & 0x00ffu) | ((uint16_t)data << 8)); return;
        case 0x7f45: set24(&s.dma_target, 0, data); return;
        case 0x7f46: set24(&s.dma_target, 1, data); return;
        case 0x7f47: set24(&s.dma_target, 2, data); return; /* DMA engine arrives later */
        case 0x7f48: s.cache_page = data & 1u; return;
        case 0x7f49: set24(&s.cache_base, 0, data); return;
        case 0x7f4a: set24(&s.cache_base, 1, data); return;
        case 0x7f4b: set24(&s.cache_base, 2, data); return;
        case 0x7f4c:
            s.cache_lock[0] = data & 1u;
            s.cache_lock[1] = (data >> 1) & 1u;
            return;
        case 0x7f4d: s.cache_pb = (uint16_t)((s.cache_pb & 0x7f00u) | data); return;
        case 0x7f4e: s.cache_pb = (uint16_t)((s.cache_pb & 0x00ffu) | (((uint16_t)data & 0x7fu) << 8)); return;
        case 0x7f4f:
            s.cache_pc = data;
            s.pc_starts++;
            return; /* no HG51B execution in v0.2 */
        case 0x7f50:
            s.wait_ram = data & 7u;
            s.wait_rom = (data >> 4) & 7u;
            return;
        case 0x7f51:
            s.irq_inhibit = data & 1u;
            if (s.irq_inhibit) s.i_flag = 0;
            return;
        case 0x7f52: s.rom_mode = data & 1u; return;
        case 0x7f53: /* force halt/unlock on hardware */ return;
        case 0x7f55: s.suspend_enable = 1; return;
        case 0x7f56: s.suspend_enable = 1; return;
        case 0x7f57: s.suspend_enable = 1; return;
        case 0x7f58: s.suspend_enable = 1; return;
        case 0x7f59: s.suspend_enable = 1; return;
        case 0x7f5a: s.suspend_enable = 1; return;
        case 0x7f5b: s.suspend_enable = 1; return;
        case 0x7f5c: s.suspend_enable = 1; return;
        case 0x7f5d: s.suspend_enable = 0; return;
        case 0x7f5e: s.i_flag = 0; return;
        default: break;
    }

    if (a >= 0x7f60u && a <= 0x7f7fu) {
        s.vector[a & 0x1fu] = data;
        return;
    }

    if ((a >= 0x7f80u && a <= 0x7fafu) ||
        (a >= 0x7fc0u && a <= 0x7fefu)) {
        uint32_t x = a & 0x3fu;
        set24(&s.gpr[(x / 3u) & 0x0fu], x % 3u, data);
        return;
    }
}

static inline uint8_t bus_read_offset(uint16_t off) {
    uint16_t lin = off & 0x0fffu;
    if (lin < 0x0c00u) {
        s.dram_reads++;
        return s.dram[lin];
    }
    s.io_reads++;
    return io_read(off);
}

static inline void bus_write_offset(uint16_t off, uint8_t data) {
    uint16_t lin = off & 0x0fffu;
    if (lin < 0x0c00u) {
        s.dram[lin] = data;
        s.dram_writes++;
        return;
    }
    s.io_writes++;
    io_write(off, data);
}

void cx4bus_reset_state(void) {
    bool was_armed = s_armed;
    /* Preserve counters across reset except the device state itself. */
    uint64_t cpu_r=s.cpu_reads, cpu_w=s.cpu_writes, drv=s.driven_reads;
    uint64_t dr=s.dram_reads, dw=s.dram_writes, ir=s.io_reads, iw=s.io_writes;
    uint64_t starts=s.pc_starts, rv=s.reset_vectors, rc=s.reset_count;

    memset(&s, 0, sizeof(s));
    s.rom_mode = 1;   /* real CX4 power-on defaults */
    s.wait_rom = 3;
    s.wait_ram = 3;

    s.cpu_reads=cpu_r; s.cpu_writes=cpu_w; s.driven_reads=drv;
    s.dram_reads=dr; s.dram_writes=dw; s.io_reads=ir; s.io_writes=iw;
    s.pc_starts=starts; s.reset_vectors=rv; s.reset_count=rc + 1u;
    s_armed = was_armed;
    s_reset_arm = 0;
}

void cx4bus_init(void) {
    memset(&s, 0, sizeof(s));
    s.rom_mode = 1;
    s.wait_rom = 3;
    s.wait_ram = 3;
    s.reset_count = 1;
    s_armed = false;
    s_driving = false;

    /* D0-D7 are the only pins we ever drive. SIO is required for core1's
     * single-cycle OE/value control. PIO input paths remain connected on RP2350. */
    const uint pins[8] = {2,3,4,6,7,8,9,10};
    for (unsigned i=0;i<8;i++) {
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
    data_release();
}

void cx4bus_arm(bool enabled) {
    s_armed = enabled;
    if (!enabled) data_release();
}

bool cx4bus_is_armed(void) { return s_armed; }

/* Dedicated latency-critical core. It is intentionally kept in SRAM and does
 * no USB/printf/heap work. The loop locks to PHI2 edges, handles Cx4 writes at
 * the falling edge (same point the existing PIO capture uses), and preloads +
 * enables D0-D7 during Cx4 reads until PHI2 falls. */
static void __not_in_flash_func(cx4bus_core1)(void) {
    s_core1_started = true;
    data_release();

    for (;;) {
        /* Synchronize to the next PHI2 rising edge. */
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
        while (!(sio_hw->gpio_in & PHI2_MASK)) tight_loop_contents();

        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool is_read = (hi & RD_HI_MASK) == 0u;
        bool is_write = (lo & WR_MASK) == 0u;

        if (is_read) {
            if (raw_is_cx4(lo, hi)) {
                uint16_t off = raw_cx4_offset(lo, hi);
                uint8_t v = bus_read_offset(off);
                s.cpu_reads++;
                s.last_read_addr = off;
                s.last_read_data = v;
                if (s_armed) {
                    data_drive(v);
                    s.driven_reads++;
                }
            } else {
                /* Reset-vector inference is outside the time-critical response
                 * path. Allow a few reads between FFFC and FFFD to be tolerant
                 * of bus noise/debug accesses. */
                uint32_t a = raw_address24(lo, hi);
                if (a == 0x00fffcu) {
                    s_reset_arm = 8;
                } else if (a == 0x00fffdu && s_reset_arm) {
                    s.reset_vectors++;
                    cx4bus_reset_state();
                } else if (s_reset_arm) {
                    --s_reset_arm;
                }
            }

            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            if (s_driving) data_release();
            continue;
        }

        if (is_write && raw_is_cx4(lo, hi)) {
            uint16_t off = raw_cx4_offset(lo, hi);
            /* Match the proven PIO write sampler: data is sampled immediately
             * after PHI2 falls. */
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            uint8_t v = raw_data(sio_hw->gpio_in);
            bus_write_offset(off, v);
            s.cpu_writes++;
            s.last_write_addr = off;
            s.last_write_data = v;
            continue;
        }

        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
    }
}

void cx4bus_launch_core1(void) {
    if (s_core1_started) return;
    multicore_launch_core1(cx4bus_core1);
    while (!s_core1_started) tight_loop_contents();
}

void cx4bus_print_status(void) {
    printf("CX4STAT mode=ACTIVE_SELFTEST_V0.2 armed=%u drive=%s core1=%u "
           "cpu_r=%llu cpu_w=%llu driven=%llu dram_r=%llu dram_w=%llu "
           "io_r=%llu io_w=%llu starts=%llu vec_resets=%llu state_resets=%llu "
           "last_r=%04lX:%02X last_w=%04lX:%02X "
           "pc=%02X pb=%04X base=%06lX rom=%u wait=%u/%u\n",
           s_armed ? 1u : 0u, s_driving ? "ON" : "OFF", s_core1_started ? 1u : 0u,
           (unsigned long long)s.cpu_reads, (unsigned long long)s.cpu_writes,
           (unsigned long long)s.driven_reads,
           (unsigned long long)s.dram_reads, (unsigned long long)s.dram_writes,
           (unsigned long long)s.io_reads, (unsigned long long)s.io_writes,
           (unsigned long long)s.pc_starts, (unsigned long long)s.reset_vectors,
           (unsigned long long)s.reset_count,
           (unsigned long)s.last_read_addr, s.last_read_data,
           (unsigned long)s.last_write_addr, s.last_write_data,
           s.cache_pc, s.cache_pb, (unsigned long)s.cache_base,
           s.rom_mode & 1u, s.wait_ram & 7u, s.wait_rom & 7u);
}
