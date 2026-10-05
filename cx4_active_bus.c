#include "cx4_active_bus.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "hardware/regs/addressmap.h"
#include "cx4.h"

/*
 * CX4 BUS v0.3.1 LLE ROM-SLOT TURBO
 * ---------------------------------
 * Active SNES-side interface backed by an instruction-level HG51B S169 core.
 * The core source is fetched at build time from the permissively licensed
 * RetroPortingToolKit/snesrecomp CX4 implementation; it is not redistributed
 * in this package. The user's own game ROM is injected locally into a reserved flash slot after build.
 *
 * v0.3 deliberately runs the DSP in TURBO-to-idle mode after CPU writes that
 * can start work. This removes real-time scheduling from the first hardware
 * validation: the goal is functional correctness (self-test/sprites) before
 * reproducing exact 20 MHz busy timing.
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

#define A13_MASK (1u << 25)
#define A14_MASK (1u << 27)
#define A15_MASK (1u << 28)
#define A22_HI_MASK (1u << (36u - 32u))

/* v0.3.1 keeps the game ROM in a reserved flash slot instead of linking the
 * copyrighted ROM into the CI build.  The generic UF2 is built on GitHub; a
 * local Python tool then appends the user's own ROM as UF2 blocks at 4 MiB. */
#define CX4_ROM_SLOT_FLASH_OFFSET (4u * 1024u * 1024u)
#define CX4_ROM_SLOT_HEADER_SIZE  256u
#define CX4_ROM_SLOT_MAX_BYTES    (4u * 1024u * 1024u - CX4_ROM_SLOT_HEADER_SIZE)
#define CX4_ROM_SLOT_MAGIC        0x52345843u /* bytes: C X 4 R */
#define CX4_ROM_SLOT_VERSION      1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t rom_size;
    uint32_t crc32;
} cx4_rom_slot_header_t;

static const uint8_t *s_game_rom = NULL;
static uint32_t s_game_rom_size = 0;
static uint32_t s_game_rom_crc32 = 0;
static bool s_rom_slot_valid = false;

static void cx4_rom_slot_probe(void) {
    const uint8_t *slot = (const uint8_t *)(XIP_BASE + CX4_ROM_SLOT_FLASH_OFFSET);
    const cx4_rom_slot_header_t *h = (const cx4_rom_slot_header_t *)slot;
    s_rom_slot_valid = false;
    s_game_rom = NULL;
    s_game_rom_size = 0;
    s_game_rom_crc32 = 0;
    if (h->magic != CX4_ROM_SLOT_MAGIC || h->version != CX4_ROM_SLOT_VERSION) return;
    if (h->rom_size == 0 || h->rom_size > CX4_ROM_SLOT_MAX_BYTES) return;
    s_game_rom = slot + CX4_ROM_SLOT_HEADER_SIZE;
    s_game_rom_size = h->rom_size;
    s_game_rom_crc32 = h->crc32;
    s_rom_slot_valid = true;
}

static Cx4 *s_core = NULL;
static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile bool s_reset_requested = false;
static volatile bool s_work_pending = false;
static volatile uint8_t s_reset_arm = 0;

static uint64_t s_master_clock = 1;

/* Bus diagnostics. Only core1 mutates the hot-path counters. */
static volatile uint64_t s_cpu_reads = 0;
static volatile uint64_t s_cpu_writes = 0;
static volatile uint64_t s_driven_reads = 0;
static volatile uint64_t s_dram_reads = 0;
static volatile uint64_t s_dram_writes = 0;
static volatile uint64_t s_io_reads = 0;
static volatile uint64_t s_io_writes = 0;
static volatile uint64_t s_turbo_runs = 0;
static volatile uint64_t s_reset_vectors = 0;
static volatile uint64_t s_state_resets = 0;
static volatile uint32_t s_last_read_addr = 0;
static volatile uint32_t s_last_write_addr = 0;
static volatile uint8_t s_last_read_data = 0;
static volatile uint8_t s_last_write_data = 0;

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
    gpio_put_masked(DATA_MASK, packed_data(v));
    gpio_set_dir_out_masked(DATA_MASK);
    s_driving = true;
}

static inline bool raw_is_cx4(uint32_t lo, uint32_t hi) {
    if (hi & A22_HI_MASK) return false;
    if (lo & A15_MASK) return false;
    if (!(lo & A14_MASK)) return false;
    if (!(lo & A13_MASK)) return false;
    return true;
}

static inline uint16_t raw_cx4_offset(uint32_t lo, uint32_t hi) {
    uint16_t off = 0x6000u;
    off |= (uint16_t)((lo >> 11) & 0x01ffu);
    off |= (uint16_t)(((lo >> 21) & 1u) << 9);
    off |= (uint16_t)(((hi >> 8) & 1u) << 10); /* A10 = GP40 */
    off |= (uint16_t)(((lo >> 23) & 1u) << 11);
    off |= (uint16_t)(((lo >> 24) & 1u) << 12);
    return off;
}

static inline uint32_t raw_address24(uint32_t lo, uint32_t hi) {
    uint32_t a = 0;
    a |= ((lo >> 11) & 0x1ffu);
    a |= ((lo >> 21) & 1u) << 9;
    a |= ((hi >> 8) & 1u) << 10;
    a |= ((lo >> 23) & 0x7u) << 11;
    a |= ((lo >> 27) & 0x1fu) << 14;
    a |= (hi & 0x7u) << 19;
    a |= ((hi >> 4) & 1u) << 22;
    a |= ((hi >> 5) & 1u) << 23;
    return a & 0xffffffu;
}

static inline bool off_is_io(uint16_t off) {
    return (off & 0x0fffu) >= 0x0c00u;
}

static inline uint16_t canonical_io(uint16_t off) {
    return (uint16_t)(0x7c00u | (off & 0x03ffu));
}

static inline bool write_can_start_work(uint16_t off) {
    if (!off_is_io(off)) return false;
    uint16_t a = canonical_io(off);
    return a == 0x7f47u || a == 0x7f48u || a == 0x7f4fu ||
           (a >= 0x7f53u && a <= 0x7f5du);
}

static inline void core_step_master(uint32_t master_cycles) {
    if (!s_core) return;
    s_master_clock += master_cycles;
    cx4_sync(s_core, s_master_clock);
}

/* IMPORTANT: the latency-critical bus responder runs on core1. It must never
 * execute a long DSP burst, otherwise SNES cycles would pass unanswered.
 * Core1 only records that work may have started; core0 services the LLE core
 * in the normal main loop while core1 keeps D0-D7 responsive. */
static inline void core_note_write(uint16_t off) {
    if (write_can_start_work(off)) s_work_pending = true;
}

static void core_reset_now(void) {
    if (!s_core) return;
    cx4_reset(s_core);
    cx4_synthesize_data_rom(s_core);
    s_master_clock += 1;
    s_state_resets++;
}

void cx4bus_reset_state(void) {
    /* Core0 requests; latency-critical core1 performs it between SNES cycles. */
    s_reset_requested = true;
}

void cx4bus_init(void) {
    cx4_rom_slot_probe();
    s_core = s_rom_slot_valid ? cx4_create(s_game_rom, s_game_rom_size, NULL, 0) : NULL;
    if (s_core) cx4_synthesize_data_rom(s_core);

    s_master_clock = 1;
    s_armed = false;
    s_driving = false;
    s_reset_requested = false;
    s_work_pending = false;
    s_state_resets = s_core ? 1 : 0;

    const uint pins[8] = {2,3,4,6,7,8,9,10};
    for (unsigned i=0;i<8;i++) {
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
    data_release();
}

void cx4bus_arm(bool enabled) {
    s_armed = enabled && (s_core != NULL);
    if (!s_armed) data_release();
}

bool cx4bus_is_armed(void) { return s_armed; }

static void __not_in_flash_func(cx4bus_core1)(void) {
    s_core1_started = true;
    data_release();

    for (;;) {
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();

        if (s_reset_requested) {
            s_reset_requested = false;
            core_reset_now();
        }

        while (!(sio_hw->gpio_in & PHI2_MASK)) tight_loop_contents();

        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool is_read = (hi & RD_HI_MASK) == 0u;
        bool is_write = (lo & WR_MASK) == 0u;

        if (is_read) {
            if (raw_is_cx4(lo, hi)) {
                uint16_t off = raw_cx4_offset(lo, hi);
                /* Give pending LLE work a small chance to advance even if it was
                 * not started by a register we currently recognize. */
                core_step_master(128u);
                uint8_t v = s_core ? cx4_read(s_core, off) : 0u;
                s_cpu_reads++;
                if (off_is_io(off)) s_io_reads++; else s_dram_reads++;
                s_last_read_addr = off;
                s_last_read_data = v;
                if (s_armed) {
                    data_drive(v);
                    s_driven_reads++;
                }
            } else {
                uint32_t a = raw_address24(lo, hi);
                if (a == 0x00fffcu) {
                    s_reset_arm = 8;
                } else if (a == 0x00fffdu && s_reset_arm) {
                    s_reset_vectors++;
                    s_reset_requested = true;
                    s_reset_arm = 0;
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
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            uint8_t v = raw_data(sio_hw->gpio_in);

            if (s_core) {
                cx4_write(s_core, off, v);
                core_note_write(off);
            }
            s_cpu_writes++;
            if (off_is_io(off)) s_io_writes++; else s_dram_writes++;
            s_last_write_addr = off;
            s_last_write_data = v;
            continue;
        }

        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
    }
}

void cx4bus_service(void) {
    /* Runs on core0. Keep the responder on core1 free at all times. A generous
     * virtual slice lets the instruction-level core catch up much faster than
     * real-time; the game simply sees RUNNING/BUSY until the computation ends. */
    if (!s_core || !s_work_pending) return;

    core_step_master(65536u);
    s_turbo_runs++;

    /* $7F53 mirrors status. bit0=suspended, bit6=running, bit7=busy. */
    uint8_t st = cx4_read(s_core, 0x7f53u);
    if ((st & 0x01u) || (st & 0xc0u) == 0u) s_work_pending = false;
}

void cx4bus_launch_core1(void) {
    if (s_core1_started) return;
    multicore_launch_core1(cx4bus_core1);
    while (!s_core1_started) tight_loop_contents();
}

void cx4bus_print_status(void) {
    size_t rom_size = (size_t)s_game_rom_size;
    uint64_t insns = s_core ? cx4_instructions_executed(s_core) : 0;
    uint32_t rdrom = s_core ? cx4_rdrom_hits(s_core) : 0;
    uint32_t runs = s_core ? cx4_run_ring_count(s_core) : 0;
    int fw = s_core ? cx4_firmware_loaded(s_core) : 0;
    int locked = s_core ? cx4_locked(s_core) : 0;

    printf("CX4STAT mode=LLE_ROM_SLOT_TURBO_V0.3.1 armed=%u drive=%s core1=%u core=%u "
           "rom_slot=%u rom_bytes=%lu rom_crc=%08lX firmware=%d locked=%d runs=%lu insns=%llu rdrom=%lu service=%llu pending=%u "
           "cpu_r=%llu cpu_w=%llu driven=%llu dram_r=%llu dram_w=%llu io_r=%llu io_w=%llu "
           "vec_resets=%llu state_resets=%llu last_r=%04lX:%02X last_w=%04lX:%02X\n",
           s_armed ? 1u : 0u, s_driving ? "ON" : "OFF", s_core1_started ? 1u : 0u,
           s_core ? 1u : 0u, s_rom_slot_valid ? 1u : 0u, (unsigned long)rom_size,
           (unsigned long)s_game_rom_crc32, fw, locked,
           (unsigned long)runs, (unsigned long long)insns, (unsigned long)rdrom,
           (unsigned long long)s_turbo_runs, s_work_pending ? 1u : 0u,
           (unsigned long long)s_cpu_reads, (unsigned long long)s_cpu_writes,
           (unsigned long long)s_driven_reads,
           (unsigned long long)s_dram_reads, (unsigned long long)s_dram_writes,
           (unsigned long long)s_io_reads, (unsigned long long)s_io_writes,
           (unsigned long long)s_reset_vectors, (unsigned long long)s_state_resets,
           (unsigned long)s_last_read_addr, s_last_read_data,
           (unsigned long)s_last_write_addr, s_last_write_data);
}
