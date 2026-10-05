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
 * CX4 BUS v0.5 HLE-GAMEPLAY ISOLATED
 *
 * Goal: make the CPU-visible CX4 state deterministic for gameplay bring-up.
 * Runtime HG51B LLE is deliberately NOT connected to the CPU-visible C4RAM.
 * The LLE core remains available only for CX4SELF, which validates the injected
 * MMX2 ROM + HG51B core independently of the physical bus.
 *
 * Persistent CPU-visible C4 state is written only by the proven PIO+DMA write
 * path. The active core1 responder only reads this state and drives D0-D7.
 * $7F4F=$00 / $7F4D=$00 executes an OAM conversion matching Snes9x C4ConvOAM.
 */

#define PIN_PHI2 0u
#define PIN_WR   1u
#define PIN_RD   35u

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

#define CX4_ROM_SLOT_FLASH_OFFSET (4u * 1024u * 1024u)
#define CX4_ROM_SLOT_HEADER_SIZE 256u
#define CX4_ROM_SLOT_MAX_BYTES (4u * 1024u * 1024u - CX4_ROM_SLOT_HEADER_SIZE)
#define CX4_ROM_SLOT_MAGIC 0x52345843u
#define CX4_ROM_SLOT_VERSION 1u

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

/* CPU-visible 8 KiB C4 address space ($6000-$7FFF). For the real chip,
 * $7000-$7BFF mirrors 3 KiB data RAM and several IO areas have special
 * semantics. We implement the known RAM mirror and the subset required for
 * gameplay HLE; unimplemented areas read the byte last written (or zero). */
static uint8_t s_c4ram[0x2000];
static uint8_t s_hle_input[0x0c00];
static uint8_t s_oam_backup[0x0220];

static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile bool s_reset_requested = false;
static volatile uint8_t s_reset_arm = 0;

static volatile uint64_t s_cpu_reads = 0;
static volatile uint64_t s_cpu_writes = 0;
static volatile uint64_t s_driven_reads = 0;
static volatile uint64_t s_pio_writes = 0;
static volatile uint64_t s_pio_ram_writes = 0;
static volatile uint64_t s_pio_io_writes = 0;
static volatile uint64_t s_reset_vectors = 0;
static volatile uint64_t s_state_resets = 0;
static volatile uint32_t s_last_read_addr = 0;
static volatile uint32_t s_last_write_addr = 0;
static volatile uint8_t s_last_read_data = 0;
static volatile uint8_t s_last_write_data = 0;

static volatile uint64_t s_hle_jobs = 0;
static volatile uint64_t s_hle_good_jobs = 0;
static volatile uint64_t s_hle_rejected_jobs = 0;
static volatile uint64_t s_hle_groups = 0;
static volatile uint64_t s_hle_oam = 0;
static volatile uint64_t s_hle_badparts = 0;
static volatile uint64_t s_hle_zero_ptr = 0;
static volatile uint32_t s_hle_pmax = 0;
static volatile uint32_t s_hle_last_ptr = 0;
static volatile uint8_t s_hle_last_sub = 0xffu;

/* These three are invariants explicitly checked by Snes9x's C4 debugger. */
static volatile uint64_t s_inv625_bad = 0;
static volatile uint64_t s_inv629_bad = 0;
static volatile uint64_t s_inv627_bad = 0;
static volatile uint8_t s_inv_last_625 = 0;
static volatile uint8_t s_inv_last_626 = 0;
static volatile uint8_t s_inv_last_629 = 0;
static volatile uint16_t s_inv_last_627 = 0;

/* Tiny IO trace. */
typedef struct { uint16_t off; uint8_t data; uint8_t op; } trace_t;
#define TRACE_N 64u
static volatile trace_t s_trace[TRACE_N];
static volatile uint32_t s_trace_seq = 0;

static void rom_slot_probe(void) {
    const uint8_t *slot = (const uint8_t *)(XIP_BASE + CX4_ROM_SLOT_FLASH_OFFSET);
    const cx4_rom_slot_header_t *h = (const cx4_rom_slot_header_t *)slot;
    s_rom_slot_valid = false;
    s_game_rom = NULL;
    s_game_rom_size = 0;
    s_game_rom_crc32 = 0;
    if (h->magic != CX4_ROM_SLOT_MAGIC || h->version != CX4_ROM_SLOT_VERSION) return;
    if (!h->rom_size || h->rom_size > CX4_ROM_SLOT_MAX_BYTES) return;
    s_game_rom = slot + CX4_ROM_SLOT_HEADER_SIZE;
    s_game_rom_size = h->rom_size;
    s_game_rom_crc32 = h->crc32;
    s_rom_slot_valid = true;
}

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
    if (hi & A22_HI_MASK) return false;  /* banks 40-7F/C0-FF excluded */
    if (lo & A15_MASK) return false;
    if (!(lo & A14_MASK)) return false;
    if (!(lo & A13_MASK)) return false;
    return true;
}
static inline uint16_t raw_cx4_offset(uint32_t lo, uint32_t hi) {
    uint16_t off = 0x6000u;
    off |= (uint16_t)((lo >> 11) & 0x01ffu);
    off |= (uint16_t)(((lo >> 21) & 1u) << 9);
    off |= (uint16_t)(((hi >> 8) & 1u) << 10);
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
static inline bool cx4_bank(uint8_t b) {
    return b <= 0x3fu || (b >= 0x80u && b <= 0xbfu);
}

static inline uint16_t ram_index_from_off(uint16_t off) {
    return (uint16_t)(off - 0x6000u);
}

/* CPU-visible read. Match old HLE behavior for status: $7F5E is ready. */
static inline uint8_t bus_read(uint16_t off) {
    if (off == 0x7f5eu) return 0u;
    if (off >= 0x7000u && off <= 0x7bffu)
        return s_c4ram[(off - 0x7000u) & 0x0fffu];
    return s_c4ram[ram_index_from_off(off) & 0x1fffu];
}

static inline void bus_write_raw(uint16_t off, uint8_t v) {
    if (off >= 0x7000u && off <= 0x7bffu) {
        uint16_t i = (uint16_t)((off - 0x7000u) & 0x0fffu);
        if (i < 0x0c00u) s_c4ram[i] = v;
        return;
    }
    s_c4ram[ram_index_from_off(off) & 0x1fffu] = v;
}

static inline void trace_io(uint8_t op, uint16_t off, uint8_t data) {
    if (off < 0x7f40u) return;
    uint32_t i = s_trace_seq++ & (TRACE_N - 1u);
    s_trace[i].off = off; s_trace[i].data = data; s_trace[i].op = op;
}

static inline uint16_t rd16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd24(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}
static inline uint32_t c4_rom_offset(uint32_t a) {
    return ((a & 0xff0000u) >> 1) + (a & 0x7fffu);
}
static inline uint8_t rom8(uint32_t off) {
    return (s_game_rom && off < s_game_rom_size) ? s_game_rom[off] : 0u;
}

/* Exact gameplay OAM conversion structure from Snes9x C4ConvOAM, adapted to
 * the isolated CPU-visible RAM. We compute into live OAM but keep a backup and
 * roll back the entire OAM table when the input snapshot fails strong sanity
 * checks. This prevents one bad capture from turning a frame into garbage. */
static bool hle_conv_oam(void) {
    if (!s_game_rom) return false;

    memcpy(s_hle_input, s_c4ram, sizeof(s_hle_input));
    memcpy(s_oam_backup, s_c4ram, sizeof(s_oam_backup));

    const uint8_t v625 = s_hle_input[0x625];
    const uint8_t v626 = s_hle_input[0x626];
    const uint8_t v629 = s_hle_input[0x629];
    const uint16_t v627 = rd16(&s_hle_input[0x627]);
    s_inv_last_625 = v625; s_inv_last_626 = v626; s_inv_last_629 = v629; s_inv_last_627 = v627;

    bool inv_ok = true;
    if (v625 != 0u) { s_inv625_bad++; inv_ok = false; }
    if (v629 != (uint8_t)(v626 >> 2)) { s_inv629_bad++; inv_ok = false; }
    if (v627 != (uint16_t)((uint16_t)v626 << 2)) { s_inv627_bad++; inv_ok = false; }

    const uint8_t groups = s_hle_input[0x620];
    if (groups > 128u || v626 > 127u) inv_ok = false;

    s_hle_jobs++;
    s_hle_last_sub = s_c4ram[0x1f4d];

    if (!inv_ok) {
        s_hle_rejected_jobs++;
        return false;
    }

    uint16_t oam = (uint16_t)v626 << 2;
    for (int i = 0x01fd; i > (int)oam; i -= 4) s_c4ram[i] = 0xe0u;

    const int16_t global_x = (int16_t)rd16(&s_hle_input[0x621]);
    const int16_t global_y = (int16_t)rd16(&s_hle_input[0x623]);
    uint16_t oam2 = (uint16_t)(0x0200u + (v626 >> 2));
    uint8_t spr_count = (uint8_t)(128u - v626);
    uint8_t bitoff = (uint8_t)((v626 & 3u) * 2u);
    uint16_t src = 0x0220u;
    uint32_t emitted = 0;
    uint32_t bad_parts_this_job = 0;

    for (uint16_t gi = 0; gi < groups && spr_count; ++gi, src += 16u) {
        if (src + 15u >= sizeof(s_hle_input)) break;
        s_hle_groups++;

        int16_t sx = (int16_t)rd16(&s_hle_input[src]) - global_x;
        int16_t sy = (int16_t)rd16(&s_hle_input[src + 2u]) - global_y;
        uint8_t name = s_hle_input[src + 5u];
        uint8_t attr = (uint8_t)(s_hle_input[src + 4u] | s_hle_input[src + 6u]);
        uint32_t ptr = rd24(&s_hle_input[src + 7u]);
        s_hle_last_ptr = ptr;
        uint32_t rp = c4_rom_offset(ptr);
        uint8_t parts = rom8(rp);
        if (parts > s_hle_pmax) s_hle_pmax = parts;
        if (rp >= s_game_rom_size) s_hle_zero_ptr++;

        /* A C4 OAM job can never emit more than the remaining SNES sprite slots.
         * If the assembly claims more pieces than remain, treat that descriptor
         * as corrupt for this frame rather than consuming arbitrary ROM bytes. */
        if (parts > spr_count) {
            s_hle_badparts++;
            bad_parts_this_job++;
            continue;
        }

        if (parts) {
            rp++;
            for (uint16_t n = 0; n < parts && spr_count; ++n, rp += 4u) {
                uint8_t flags = rom8(rp + 0u);
                int16_t x = (int8_t)rom8(rp + 1u);
                int16_t y = (int8_t)rom8(rp + 2u);
                uint8_t td = rom8(rp + 3u);
                bool large = (flags & 0x20u) != 0u;

                if (attr & 0x40u) x = (int16_t)(-x - (large ? 16 : 8));
                x = (int16_t)(x + sx);
                if (x < -16 || x > 272) continue;
                if (attr & 0x80u) y = (int16_t)(-y - (large ? 16 : 8));
                y = (int16_t)(y + sy);
                if (y < -16 || y > 224) continue;

                s_c4ram[oam + 0u] = (uint8_t)x;
                s_c4ram[oam + 1u] = (uint8_t)y;
                s_c4ram[oam + 2u] = (uint8_t)(name + td);
                s_c4ram[oam + 3u] = (uint8_t)(attr ^ (flags & 0xc0u));

                uint8_t hv = s_c4ram[oam2];
                hv &= (uint8_t)~(3u << bitoff);
                if (x & 0x100) hv |= (uint8_t)(1u << bitoff);
                if (large) hv |= (uint8_t)(2u << bitoff);
                s_c4ram[oam2] = hv;

                oam += 4u; spr_count--; emitted++;
                bitoff = (uint8_t)((bitoff + 2u) & 6u);
                if (!bitoff) oam2++;
            }
        } else if (spr_count) {
            s_c4ram[oam + 0u] = (uint8_t)sx;
            s_c4ram[oam + 1u] = (uint8_t)sy;
            s_c4ram[oam + 2u] = name;
            s_c4ram[oam + 3u] = attr;
            uint8_t hv = s_c4ram[oam2];
            hv &= (uint8_t)~(3u << bitoff);
            hv |= (uint8_t)((sx & 0x100) ? (3u << bitoff) : (2u << bitoff));
            s_c4ram[oam2] = hv;
            oam += 4u; spr_count--; emitted++;
            bitoff = (uint8_t)((bitoff + 2u) & 6u);
            if (!bitoff) oam2++;
        }
    }

    /* If more than half of all groups are implausible, keep the previous OAM.
     * A good frame will replace it later; a bad capture cannot spray garbage. */
    if (groups && bad_parts_this_job * 2u > groups) {
        memcpy(s_c4ram, s_oam_backup, sizeof(s_oam_backup));
        s_hle_rejected_jobs++;
        return false;
    }

    s_hle_good_jobs++;
    s_hle_oam += emitted;
    return true;
}

static void hle_on_command(uint16_t off, uint8_t data) {
    if (off != 0x7f4fu) return;
    if (data == 0x00u && s_c4ram[0x1f4du] == 0x00u) {
        (void)hle_conv_oam();
    }
}

static void clear_runtime_state(void) {
    memset(s_c4ram, 0, sizeof(s_c4ram));
    memset(s_hle_input, 0, sizeof(s_hle_input));
    memset(s_oam_backup, 0, sizeof(s_oam_backup));
    s_hle_jobs = s_hle_good_jobs = s_hle_rejected_jobs = 0;
    s_hle_groups = s_hle_oam = s_hle_badparts = s_hle_zero_ptr = 0;
    s_hle_pmax = 0; s_hle_last_ptr = 0; s_hle_last_sub = 0xffu;
    s_inv625_bad = s_inv629_bad = s_inv627_bad = 0;
    s_inv_last_625 = s_inv_last_626 = s_inv_last_629 = 0; s_inv_last_627 = 0;
    s_trace_seq = 0;
    s_state_resets++;
}

void cx4bus_init(void) {
    rom_slot_probe();
    clear_runtime_state();
    s_armed = false; s_driving = false; s_reset_requested = false;
    const uint pins[8] = {2,3,4,6,7,8,9,10};
    for (unsigned i=0;i<8;i++) {
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
    data_release();
}

void cx4bus_arm(bool enabled) {
    s_armed = enabled && s_rom_slot_valid;
    if (!s_armed) data_release();
}
bool cx4bus_is_armed(void) { return s_armed; }
void cx4bus_reset_state(void) { s_reset_requested = true; }

static void __not_in_flash_func(core1_loop)(void) {
    s_core1_started = true;
    data_release();
    for (;;) {
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
        while (!(sio_hw->gpio_in & PHI2_MASK)) tight_loop_contents();
        __asm volatile("nop; nop;" ::: "memory");
        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool rd = (hi & RD_HI_MASK) == 0u;
        bool wr = (lo & WR_MASK) == 0u;

        if (rd) {
            if (raw_is_cx4(lo, hi)) {
                uint16_t off = raw_cx4_offset(lo, hi);
                uint8_t v = bus_read(off);
                s_cpu_reads++; s_last_read_addr = off; s_last_read_data = v;
                trace_io('R', off, v);
                if (s_armed) { data_drive(v); s_driven_reads++; }
            } else {
                uint32_t a = raw_address24(lo, hi);
                if (a == 0x00fffcu) s_reset_arm = 8;
                else if (a == 0x00fffdu && s_reset_arm) {
                    s_reset_vectors++; s_reset_requested = true; s_reset_arm = 0;
                } else if (s_reset_arm) --s_reset_arm;
            }
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            if (s_driving) data_release();
            continue;
        }

        if (wr && raw_is_cx4(lo, hi)) {
            uint16_t off = raw_cx4_offset(lo, hi);
            __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;" ::: "memory");
            uint8_t v = raw_data(sio_hw->gpio_in);
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            /* Diagnostic only: persistent state is PIO+DMA-authoritative. */
            s_cpu_writes++; s_last_write_addr = off; s_last_write_data = v;
            trace_io('W', off, v);
            continue;
        }
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
    }
}

void cx4bus_launch_core1(void) {
    if (s_core1_started) return;
    multicore_launch_core1(core1_loop);
    while (!s_core1_started) tight_loop_contents();
}

void cx4bus_pio_write(uint32_t address, uint8_t data) {
    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t off = (uint16_t)address;
    if (!cx4_bank(bank) || off < 0x6000u || off > 0x7fffu) return;

    bus_write_raw(off, data);
    s_pio_writes++;
    if ((off >= 0x6000u && off <= 0x6bffu) || (off >= 0x7000u && off <= 0x7bffu))
        s_pio_ram_writes++;
    else s_pio_io_writes++;

    hle_on_command(off, data);
}

void cx4bus_service(void) {
    if (s_reset_requested) {
        s_reset_requested = false;
        clear_runtime_state();
    }
}

void cx4bus_print_trace(void) {
    uint32_t end=s_trace_seq, n=end<TRACE_N?end:TRACE_N, start=end-n;
    printf("CX4TRACE count=%lu seq=%lu (oldest->newest)\n", (unsigned long)n, (unsigned long)end);
    for (uint32_t k=0;k<n;k++) {
        trace_t e=s_trace[(start+k)&(TRACE_N-1u)];
        printf("  %c %04X %02X\n", e.op?e.op:'?', e.off, e.data);
    }
}

void cx4bus_print_runs(void) {
    printf("CX4RUNS runtime_lle=0 (use CX4SELF for LLE validation)\n");
}

static void self_sync_until_idle(Cx4 *c, uint64_t *clk) {
    for (unsigned n=0;n<8192u;n++) {
        if ((cx4_read(c,0x7f5eu)&0xc0u)==0u) return;
        *clk += 4096u; cx4_sync(c,*clk);
    }
}

void cx4bus_selfcheck(void) {
    if (s_armed) { printf("CX4SELF REFUSED armed=1; use CX4DISARM first\n"); return; }
    if (!s_rom_slot_valid) { printf("CX4SELF FAIL rom_slot=0\n"); return; }
    Cx4 *t=cx4_create(s_game_rom,s_game_rom_size,NULL,0);
    if (!t) { printf("CX4SELF FAIL alloc\n"); return; }
    cx4_synthesize_data_rom(t);
    uint64_t clk=1;
    cx4_write(t,0x7f49u,0x00u); cx4_write(t,0x7f4au,0x80u); cx4_write(t,0x7f4bu,0x02u);
    cx4_write(t,0x7f4du,0x0eu); cx4_write(t,0x7f4eu,0x00u); cx4_write(t,0x7f48u,0x01u);
    self_sync_until_idle(t,&clk);
    cx4_write(t,0x7f4fu,0x5cu); self_sync_until_idle(t,&clk);
    cx4_write(t,0x7f4fu,0x89u); self_sync_until_idle(t,&clk);
    uint32_t lo=0,hi=0; cx4_rdrom_index_range(t,&lo,&hi);
    printf("CX4SELF runs=%lu insns=%llu rdrom=%lu distinct=%lu range=%lu-%lu firmware=%d locked=%d status=%02X\n",
        (unsigned long)cx4_run_ring_count(t),(unsigned long long)cx4_instructions_executed(t),
        (unsigned long)cx4_rdrom_hits(t),(unsigned long)cx4_rdrom_distinct(t),(unsigned long)lo,(unsigned long)hi,
        cx4_firmware_loaded(t),cx4_locked(t),cx4_read(t,0x7f5eu));
    Cx4RunEvent ring[4]; uint32_t n=cx4_run_ring_copy(t,ring,4u);
    for(uint32_t i=0;i<n;i++) printf("  self run seq=%lu base=%06lX pb=%04X pc=%02X\n",
        (unsigned long)ring[i].seq,(unsigned long)ring[i].base,ring[i].pb,ring[i].pc);
    cx4_destroy(t);
}

void cx4bus_print_status(void) {
    printf("CX4STAT mode=HLE_GAMEPLAY_ISOLATED_V0.5 armed=%u drive=%s core1=%u rom_slot=%u rom_bytes=%lu rom_crc=%08lX runtime_lle=0 "
           "hle_jobs=%llu hle_good=%llu hle_reject=%llu hle_groups=%llu hle_oam=%llu hle_pmax=%lu hle_badparts=%llu hle_zptr=%llu hle_ptr=%06lX hle_sub=%02X "
           "inv625_bad=%llu inv629_bad=%llu inv627_bad=%llu inv_last=%02X/%02X/%02X/%04X "
           "pio_w=%llu pio_ram=%llu pio_io=%llu cpu_r=%llu cpu_w=%llu driven=%llu vec_resets=%llu state_resets=%llu last_r=%04lX:%02X last_w=%04lX:%02X\n",
           s_armed?1u:0u,s_driving?"ON":"OFF",s_core1_started?1u:0u,s_rom_slot_valid?1u:0u,
           (unsigned long)s_game_rom_size,(unsigned long)s_game_rom_crc32,
           (unsigned long long)s_hle_jobs,(unsigned long long)s_hle_good_jobs,(unsigned long long)s_hle_rejected_jobs,
           (unsigned long long)s_hle_groups,(unsigned long long)s_hle_oam,(unsigned long)s_hle_pmax,
           (unsigned long long)s_hle_badparts,(unsigned long long)s_hle_zero_ptr,(unsigned long)s_hle_last_ptr,s_hle_last_sub,
           (unsigned long long)s_inv625_bad,(unsigned long long)s_inv629_bad,(unsigned long long)s_inv627_bad,
           s_inv_last_625,s_inv_last_626,s_inv_last_629,s_inv_last_627,
           (unsigned long long)s_pio_writes,(unsigned long long)s_pio_ram_writes,(unsigned long long)s_pio_io_writes,
           (unsigned long long)s_cpu_reads,(unsigned long long)s_cpu_writes,(unsigned long long)s_driven_reads,
           (unsigned long long)s_reset_vectors,(unsigned long long)s_state_resets,
           (unsigned long)s_last_read_addr,s_last_read_data,(unsigned long)s_last_write_addr,s_last_write_data);
}
