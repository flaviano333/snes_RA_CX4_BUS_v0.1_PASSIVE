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
 * CX4 FINAL GAMEPLAY V1 - QUALIFIED RD RESPONDER + PIO WRITE AUTHORITY
 * ---------------------------------
 * Active SNES-side interface backed by an instruction-level HG51B S169 core.
 * The core source is fetched at build time from the permissively licensed
 * RetroPortingToolKit/snesrecomp CX4 implementation; it is not redistributed
 * in this package. The user's own game ROM is injected locally into a reserved flash slot after build.
 *
 * Persistent state follows the v0.4.3 proven rule: the latency-
 * critical SIO observer no longer writes speculative samples into canonical
 * CX4 DRAM/register state. PIO+DMA is the sole authority for persistent state.
 * SIO provides only a short-lived read-after-write forwarding cache plus the
 * immediate status handshake. This prevents a bad SIO address sample from
 * permanently poisoning a second DRAM/register location.
 */

#define PIN_PHI2       0u
#define PIN_WR         1u
#define PIN_RD         35u
#define PIN_ROMSEL     38u
#define PIN_WRAMSEL    39u

#define DATA_MASK ((1u << 2) | (1u << 3) | (1u << 4) | \
                   (1u << 6) | (1u << 7) | (1u << 8) | \
                   (1u << 9) | (1u << 10))

static uint32_t s_packed_data[256];

#define PHI2_MASK (1u << PIN_PHI2)
#define WR_MASK   (1u << PIN_WR)
#define RD_HI_MASK      (1u << (PIN_RD - 32u))
#define ROMSEL_HI_MASK  (1u << (PIN_ROMSEL - 32u))
#define WRAMSEL_HI_MASK (1u << (PIN_WRAMSEL - 32u))

#define A13_MASK (1u << 25)
#define A14_MASK (1u << 27)
#define A15_MASK (1u << 28)
#define A22_HI_MASK (1u << (36u - 32u))

/* The final build keeps the game ROM in a reserved flash slot instead of linking the
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
/* Fast CPU-visible interface. Core1 NEVER calls cx4_read/cx4_write in v0.3.8.
 * Core0 is the sole owner of the HG51B control/register state; core1 serves
 * the physical SNES bus from this shadow plus the core dataRAM byte array. */
static uint8_t *s_dram = NULL;
static volatile uint8_t s_io_shadow[0x400];

typedef struct { uint16_t off; uint16_t epoch; uint8_t data; uint8_t status_evt; } cx4_io_write_evt_t;
#define CX4_IOQ_N 8192u
/* Deep SPSC queue: the SNES is allowed to touch CX4 interface registers while
 * cache/DMA/DSP work is still in flight. v0.3.8 blocked core0 until a job became
 * idle and the old 512-entry queue overflowed, losing the very PB/PC/base writes
 * needed by subsequent runs. */
static volatile cx4_io_write_evt_t s_ioq[CX4_IOQ_N];
static volatile uint32_t s_ioq_head = 0;
static volatile uint32_t s_ioq_tail = 0;
static volatile uint32_t s_ioq_dropped = 0;
static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile bool s_reset_requested = false;
static volatile bool s_work_pending = false;
static volatile bool s_halt_shadow = true; /* CPU-visible approximation of io.halt */
static volatile uint8_t s_reset_arm = 0;

/* CPU-visible status must change immediately when the SNES starts cache/DMA/DSP.
 * In v0.3.5 these bits were only published after core0 drained the IO queue, so
 * MMX2 could read $7F5E=00 immediately after $7F48/$7F4F and incorrectly assume
 * the operation had already finished. Pack value/mask/epoch into one atomic word:
 * bits 0..7=value, 8..15=mask, 16..31=epoch. */
static volatile uint32_t s_status_override = 0;
static volatile uint64_t s_status_override_sets = 0;
static volatile uint64_t s_status_forced_reads = 0;

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
static volatile uint64_t s_jobs_started = 0;
static volatile uint64_t s_jobs_finished = 0;
static volatile uint64_t s_job_chunks = 0;
static volatile uint64_t s_job_timeouts = 0;
static volatile uint64_t s_stream_slices = 0;
static volatile uint64_t s_stream_master = 0;
static volatile uint64_t s_idle_transitions = 0;
static volatile uint32_t s_ioq_highwater = 0;
static volatile uint64_t s_pio_writes = 0;
static volatile uint64_t s_pio_dram_writes = 0;
static volatile uint64_t s_pio_io_writes = 0;
static volatile uint64_t s_sio_shadow_writes = 0;

/* v0.4.3: SIO observations are never persistent state. Keep only a tiny
 * core1-local forwarding ring so an immediate CPU read-after-write can still
 * observe the just-written byte before PIO+DMA has reached core0. Entries
 * expire after a handful of bus transactions. */
#define CX4_FWD_N 16u
#define CX4_FWD_MAX_AGE 12u
typedef struct { uint16_t off; uint8_t data; uint8_t _pad; uint32_t seq; } cx4_fwd_t;
static cx4_fwd_t s_fwd[CX4_FWD_N];
static uint32_t s_fwd_head = 0;
static uint32_t s_bus_seq = 0;
static volatile uint64_t s_fwd_puts = 0;
static volatile uint64_t s_fwd_hits = 0;
static volatile uint64_t s_sio_state_blocked = 0;
static volatile uint64_t s_hle_badparts = 0;

static inline void fwd_put(uint16_t off, uint8_t data) {
    cx4_fwd_t *e = &s_fwd[s_fwd_head++ & (CX4_FWD_N - 1u)];
    e->off = off; e->data = data; e->seq = s_bus_seq;
    s_fwd_puts++;
}

static inline bool fwd_get(uint16_t off, uint8_t *out) {
    uint32_t n = s_fwd_head < CX4_FWD_N ? s_fwd_head : CX4_FWD_N;
    for (uint32_t k = 0; k < n; ++k) {
        const cx4_fwd_t *e = &s_fwd[(s_fwd_head - 1u - k) & (CX4_FWD_N - 1u)];
        if ((uint32_t)(s_bus_seq - e->seq) > CX4_FWD_MAX_AGE) break;
        if (e->off == off) { *out = e->data; s_fwd_hits++; return true; }
    }
    return false;
}

/* Legacy sprite-HLE code is retained only as an emergency diagnostic path.
 * FINAL GAMEPLAY keeps it disabled: the qualified PIO write path now feeds the
 * instruction-level HG51B core for all real X2 jobs. */
static const bool s_sprite_hle_enabled = false;
static volatile uint64_t s_hle_sprite_jobs = 0;
static volatile uint64_t s_hle_sprite_objects = 0;
static volatile uint64_t s_hle_oam_entries = 0;
static volatile uint64_t s_hle_rom_ptr_fail = 0;
static volatile uint64_t s_hle_rom_lowhalf = 0;
static volatile uint64_t s_hle_rom_oor = 0;
static volatile uint64_t s_hle_zero_pad_reads = 0;
static volatile uint64_t s_hle_zero_pad_ptrs = 0;
static volatile uint64_t s_hle_snapshot_jobs = 0;
static volatile uint32_t s_hle_parts_max = 0;
static volatile uint32_t s_hle_last_ptr = 0;
static volatile uint8_t s_hle_last_subcmd = 0xffu;
static uint8_t s_hle_snapshot[0x0c00];

static void status_override_update(uint8_t mask, uint8_t value);

static inline uint8_t hle_ram_get(uint16_t i) {
    i &= 0x1fffu;
    if (i < 0x0c00u) return s_dram ? s_dram[i] : 0u;
    if (i >= 0x1000u && i < 0x1c00u) return s_dram ? s_dram[i & 0x0fffu] : 0u;
    return s_io_shadow[i & 0x03ffu];
}

static inline void hle_ram_put(uint16_t i, uint8_t v) {
    i &= 0x1fffu;
    if (i < 0x0c00u) {
        if (s_dram) s_dram[i] = v;
        return;
    }
    if (i >= 0x1000u && i < 0x1c00u) {
        if (s_dram) s_dram[i & 0x0fffu] = v;
        return;
    }
    s_io_shadow[i & 0x03ffu] = v;
}

static inline uint16_t hle_ram16(uint16_t i) {
    return (uint16_t)hle_ram_get(i) | ((uint16_t)hle_ram_get((uint16_t)(i + 1u)) << 8);
}

static inline uint32_t hle_ram24(uint16_t i) {
    return (uint32_t)hle_ram_get(i) |
           ((uint32_t)hle_ram_get((uint16_t)(i + 1u)) << 8) |
           ((uint32_t)hle_ram_get((uint16_t)(i + 2u)) << 16);
}

/* C4 cartridge-ROM pointer mapping. Mature C4 HLE code indexes a large ROM
 * allocation directly. The loaded image occupies rom_size bytes; the rest of
 * that allocation is zero. v0.4.1 skipped an object when a pointer exceeded the
 * injected file, which does not match that behavior. */
static inline uint32_t hle_rom_offset(uint32_t snes_addr) {
    uint16_t addr = (uint16_t)snes_addr;
    if (addr < 0x8000u) s_hle_rom_lowhalf++;
    return ((snes_addr & 0xff0000u) >> 1) + (snes_addr & 0x7fffu);
}

static inline uint8_t hle_rom_byte_off(uint32_t off, bool *zero_padded) {
    if (s_rom_slot_valid && s_game_rom && off < s_game_rom_size)
        return s_game_rom[off];
    s_hle_zero_pad_reads++;
    if (zero_padded) *zero_padded = true;
    return 0u;
}

static inline uint8_t hle_in8(uint16_t i) {
    return (i < 0x0c00u) ? s_hle_snapshot[i] : 0u;
}
static inline uint16_t hle_in16(uint16_t i) {
    return (uint16_t)hle_in8(i) | ((uint16_t)hle_in8((uint16_t)(i + 1u)) << 8);
}
static inline uint32_t hle_in24(uint16_t i) {
    return (uint32_t)hle_in8(i) |
           ((uint32_t)hle_in8((uint16_t)(i + 1u)) << 8) |
           ((uint32_t)hle_in8((uint16_t)(i + 2u)) << 16);
}

/* Ordinary C4 OAM conversion, using a stable snapshot for all inputs. Outputs
 * are committed to live C4 RAM, exactly where the SNES expects the OAM staging
 * table ($6000-$621F). */
static bool hle_build_oam(void) {
    if (!s_dram || !s_game_rom) return false;

    memcpy(s_hle_snapshot, s_dram, sizeof(s_hle_snapshot));
    s_hle_snapshot_jobs++;

    const uint8_t first = hle_in8(0x0626u);
    uint8_t spr_count = (uint8_t)(128u - first);
    uint8_t offset = (uint8_t)((first & 3u) * 2u);
    uint16_t oam = (uint16_t)first << 2;
    uint16_t oam2 = (uint16_t)(0x0200u + (first >> 2));
    const int16_t global_x = (int16_t)hle_in16(0x0621u);
    const int16_t global_y = (int16_t)hle_in16(0x0623u);
    const uint8_t groups = hle_in8(0x0620u);

    /* Hide unused low-OAM entries by setting their Y coordinate to E0. */
    if (oam < 0x0200u) {
        for (int i = 0x01fd; i > (int)oam; i -= 4)
            hle_ram_put((uint16_t)i, 0xe0u);
    }

    uint16_t src = 0x0220u;
    uint16_t groups_done = 0;
    uint16_t emitted = 0;

    for (uint16_t gi = 0; gi < groups && spr_count > 0u; ++gi, src = (uint16_t)(src + 16u)) {
        if ((uint32_t)src + 15u >= sizeof(s_hle_snapshot)) break;
        groups_done++;

        int16_t spr_x = (int16_t)hle_in16(src) - global_x;
        int16_t spr_y = (int16_t)hle_in16((uint16_t)(src + 2u)) - global_y;
        uint8_t spr_name = hle_in8((uint16_t)(src + 5u));
        uint8_t spr_attr = (uint8_t)(hle_in8((uint16_t)(src + 4u)) |
                                     hle_in8((uint16_t)(src + 6u)));
        uint32_t ptr24 = hle_in24((uint16_t)(src + 7u));
        s_hle_last_ptr = ptr24;

        uint32_t rp = hle_rom_offset(ptr24);
        bool zero_ptr = false;
        uint8_t parts = hle_rom_byte_off(rp, &zero_ptr);
        if (zero_ptr) {
            s_hle_zero_pad_ptrs++;
            s_hle_rom_oor++;
        }
        if (parts > s_hle_parts_max) s_hle_parts_max = parts;

        /* X2 sprite assemblies are small. 0xFF is a strong signature of a
         * poisoned pointer/descriptor, not a plausible 255-piece object. Skip
         * pathological assemblies instead of turning one bad descriptor into
         * a screen full of garbage. This is only a bring-up guard; clean input
         * should make the counter remain zero. */
        if (parts > 64u) {
            s_hle_badparts++;
            continue;
        }

        if (parts != 0u) {
            rp++;
            for (uint16_t si = 0; si < parts && spr_count > 0u; ++si, rp += 4u) {
                uint8_t flags = hle_rom_byte_off(rp + 0u, NULL);
                int16_t x = (int8_t)hle_rom_byte_off(rp + 1u, NULL);
                int16_t y = (int8_t)hle_rom_byte_off(rp + 2u, NULL);
                uint8_t tile_delta = hle_rom_byte_off(rp + 3u, NULL);
                const bool large = (flags & 0x20u) != 0u;

                if (spr_attr & 0x40u) x = (int16_t)(-x - (large ? 16 : 8));
                x = (int16_t)(x + spr_x);
                if (x < -16 || x > 272) continue;

                if (spr_attr & 0x80u) y = (int16_t)(-y - (large ? 16 : 8));
                y = (int16_t)(y + spr_y);
                if (y < -16 || y > 224) continue;

                hle_ram_put(oam + 0u, (uint8_t)x);
                hle_ram_put(oam + 1u, (uint8_t)y);
                hle_ram_put(oam + 2u, (uint8_t)(spr_name + tile_delta));
                hle_ram_put(oam + 3u, (uint8_t)(spr_attr ^ (flags & 0xc0u)));

                uint8_t hv = hle_ram_get(oam2);
                hv &= (uint8_t)~(3u << offset);
                if (x & 0x100) hv |= (uint8_t)(1u << offset);
                if (large) hv |= (uint8_t)(2u << offset);
                hle_ram_put(oam2, hv);

                oam = (uint16_t)(oam + 4u);
                spr_count--;
                emitted++;
                offset = (uint8_t)((offset + 2u) & 6u);
                if (offset == 0u) oam2++;
            }
        } else if (spr_count > 0u) {
            /* A zero assembly count represents the base object itself. */
            hle_ram_put(oam + 0u, (uint8_t)spr_x);
            hle_ram_put(oam + 1u, (uint8_t)spr_y);
            hle_ram_put(oam + 2u, spr_name);
            hle_ram_put(oam + 3u, spr_attr);

            uint8_t hv = hle_ram_get(oam2);
            hv &= (uint8_t)~(3u << offset);
            hv |= (uint8_t)((spr_x & 0x100) ? (3u << offset) : (2u << offset));
            hle_ram_put(oam2, hv);

            oam = (uint16_t)(oam + 4u);
            spr_count--;
            emitted++;
            offset = (uint8_t)((offset + 2u) & 6u);
            if (offset == 0u) oam2++;
        }
    }

    s_hle_sprite_objects += groups_done;
    s_hle_oam_entries += emitted;
    return true;
}

static bool hle_try_runtime_job(uint16_t off, uint8_t data) {
    if (!s_sprite_hle_enabled || (off & 0x03ffu) != 0x034fu || data != 0x00u)
        return false;

    /* $7F4D=$0E belongs to the boot diagnostic/immediate tests. Only steal the
     * normal sprite function path ($7F4D=$00) from LLE. */
    uint8_t sub = s_io_shadow[0x34du];
    s_hle_last_subcmd = sub;
    if (sub != 0x00u) return false;

    /* Present BUSY/RUNNING while producing the OAM table, then complete. */
    status_override_update(0xc0u, 0xc0u);
    bool ok = hle_build_oam();
    s_hle_sprite_jobs++;
    s_halt_shadow = true;
    s_work_pending = false;
    s_io_shadow[0x35eu] = 0u;
    __atomic_store_n(&s_status_override, 0u, __ATOMIC_RELEASE);
    return ok;
}
static volatile uint64_t s_reset_vectors = 0;
static volatile uint64_t s_state_resets = 0;
static volatile uint32_t s_last_read_addr = 0;
static volatile uint32_t s_last_write_addr = 0;
static volatile uint8_t s_last_read_data = 0;
static volatile uint8_t s_last_write_data = 0;


/* Last IO operations as seen by the ACTIVE responder. This is intentionally
 * tiny and lock-free: core1 is the only writer, the USB command is a reader.
 * It lets us see whether the self-test wrote a value and then read the same
 * interface register back without perturbing the latency-critical path. */
static inline bool off_is_io(uint16_t off);

typedef struct {
    uint16_t off;
    uint8_t data;
    uint8_t op; /* 'R' or 'W' */
} cx4_trace_entry_t;
#define CX4_TRACE_N 64u
static volatile cx4_trace_entry_t s_trace[CX4_TRACE_N];
static volatile uint32_t s_trace_seq = 0;

static inline void trace_io(uint8_t op, uint16_t off, uint8_t data) {
    if (!off_is_io(off)) return;
    uint32_t i = s_trace_seq++ & (CX4_TRACE_N - 1u);
    s_trace[i].off = off;
    s_trace[i].data = data;
    s_trace[i].op = op;
}

static inline uint8_t raw_data(uint32_t lo) {
    return (uint8_t)(((lo >> 2) & 0x07u) | ((lo >> 3) & 0xf8u));
}

static inline uint32_t packed_data(uint8_t v) {
    return (((uint32_t)v & 0x07u) << 2) | (((uint32_t)v & 0xf8u) << 3);
}

static inline __attribute__((always_inline)) void data_release(void) {
    /* Direct SIO OE clear: one register write, all eight fragmented DATA pins. */
    sio_hw->gpio_oe_clr = DATA_MASK;
    s_driving = false;
}

static inline __attribute__((always_inline)) void data_drive(uint8_t v) {
    /* Only DATA pins ever use low-bank SIO output in this gameplay build.
     * Write the prepacked byte in one operation, then expose all 8 pins at once. */
    sio_hw->gpio_out = s_packed_data[v];
    sio_hw->gpio_oe_set = DATA_MASK;
    s_driving = true;
}

static inline __attribute__((always_inline)) bool raw_is_cx4(uint32_t lo, uint32_t hi) {
    if (hi & A22_HI_MASK) return false;
    if (lo & A15_MASK) return false;
    if (!(lo & A14_MASK)) return false;
    if (!(lo & A13_MASK)) return false;
    return true;
}

static inline __attribute__((always_inline)) uint16_t raw_cx4_offset(uint32_t lo, uint32_t hi) {
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

static inline __attribute__((always_inline)) bool off_is_io(uint16_t off) {
    return (off & 0x0fffu) >= 0x0c00u;
}

static inline __attribute__((always_inline)) uint16_t canonical_io(uint16_t off) {
    return (uint16_t)(0x7c00u | (off & 0x03ffu));
}

static inline bool write_starts_job(uint16_t off) {
    if (!off_is_io(off)) return false;
    uint16_t a = canonical_io(off);
    /* These are the three CPU-visible operations that launch autonomous work
     * while the HG51B is halted: GPDMA, program-cache fill and DSP start. */
    return a == 0x7f47u || a == 0x7f48u || a == 0x7f4fu;
}

static inline uint8_t job_status_mask(uint16_t off) {
    uint16_t a = canonical_io(off);
    if (a == 0x7f47u || a == 0x7f48u) return 0xc0u;
    if (a == 0x7f4fu) return 0x40u; /* do NOT clear BUSY from an in-flight cache */
    return 0u;
}
static inline uint8_t job_status_bits(uint16_t off) {
    uint16_t a = canonical_io(off);
    if (a == 0x7f47u || a == 0x7f48u) return 0xc0u;
    if (a == 0x7f4fu) return 0x40u;
    return 0u;
}

static inline void membar(void) { __asm volatile("dmb sy" ::: "memory"); }

static inline __attribute__((always_inline)) uint16_t io_canon(uint16_t off) { return canonical_io(off); }
static inline __attribute__((always_inline)) uint32_t io_index(uint16_t off) { return (uint32_t)(io_canon(off) & 0x03ffu); }

static inline __attribute__((always_inline)) bool io_is_status_read(uint16_t off) {
    switch (io_canon(off)) {
        case 0x7f53u: case 0x7f54u: case 0x7f55u: case 0x7f56u:
        case 0x7f57u: case 0x7f59u: case 0x7f5bu: case 0x7f5cu:
        case 0x7f5du: case 0x7f5eu: case 0x7f5fu:
            return true;
        default:
            return false;
    }
}

static inline uint8_t status_apply_override(uint8_t base) {
    uint32_t p = __atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE);
    uint8_t value = (uint8_t)(p & 0xffu);
    uint8_t mask = (uint8_t)((p >> 8) & 0xffu);
    if (mask) s_status_forced_reads++;
    return (uint8_t)((base & (uint8_t)~mask) | (value & mask));
}

static inline __attribute__((always_inline)) uint8_t status_apply_override_fast(uint8_t base) {
    uint32_t p = __atomic_load_n(&s_status_override, __ATOMIC_RELAXED);
    uint8_t value = (uint8_t)(p & 0xffu);
    uint8_t mask = (uint8_t)((p >> 8) & 0xffu);
    return (uint8_t)((base & (uint8_t)~mask) | (value & mask));
}

static inline uint8_t status_effective_now(void) {
    uint8_t base = s_io_shadow[0x353u]; /* canonical $7F53 */
    uint32_t p = __atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE);
    uint8_t value = (uint8_t)(p & 0xffu);
    uint8_t mask = (uint8_t)((p >> 8) & 0xffu);
    return (uint8_t)((base & (uint8_t)~mask) | (value & mask));
}

static void status_override_update(uint8_t mask, uint8_t value) {
    uint32_t oldv, newv;
    do {
        oldv = __atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE);
        uint8_t old_value = (uint8_t)(oldv & 0xffu);
        uint8_t old_mask = (uint8_t)((oldv >> 8) & 0xffu);
        uint16_t epoch = (uint16_t)((oldv >> 16) + 1u);
        uint8_t new_mask = (uint8_t)(old_mask | mask);
        uint8_t new_value = (uint8_t)((old_value & (uint8_t)~mask) | (value & mask));
        newv = (uint32_t)new_value | ((uint32_t)new_mask << 8) | ((uint32_t)epoch << 16);
    } while (!__atomic_compare_exchange_n(&s_status_override, &oldv, newv, false,
                                           __ATOMIC_RELEASE, __ATOMIC_RELAXED));
    s_status_override_sets++;
}

static inline uint16_t status_override_epoch(void) {
    return (uint16_t)(__atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE) >> 16);
}

static void status_override_ack_if_unchanged(uint16_t epoch) {
    uint32_t oldv = __atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE);
    if ((uint16_t)(oldv >> 16) != epoch) return;
    uint32_t desired = oldv & 0xffff0000u; /* preserve epoch, clear value+mask */
    (void)__atomic_compare_exchange_n(&s_status_override, &oldv, desired, false,
                                      __ATOMIC_RELEASE, __ATOMIC_RELAXED);
}

static void status_on_cpu_write(uint16_t off, uint8_t data) {
    uint16_t a = io_canon(off);

    /* IMPORTANT: CX4 "running" is not the same thing as !halt. A cache fill can
     * assert RUNNING/BUSY while io.halt is still true, and a following $7F4F
     * write is therefore allowed to unhalt/start the DSP. v0.3.5 inferred halt
     * from bit 6 and lost that distinction. */
    if ((a == 0x7f47u || a == 0x7f48u) && s_halt_shadow) {
        status_override_update(0xc0u, 0xc0u);
    } else if (a == 0x7f4fu && s_halt_shadow) {
        /* Only force RUNNING. If a cache/DMA is still busy, preserve bit 7. */
        status_override_update(0x40u, 0x40u);
        s_halt_shadow = false;
    } else if (a == 0x7f53u) {
        s_halt_shadow = true;
        status_override_update(0xc0u, 0x00u);
    } else if (a >= 0x7f55u && a <= 0x7f5cu) {
        status_override_update(0x01u, 0x01u);
    } else if (a == 0x7f5du) {
        status_override_update(0x01u, 0x00u);
    } else if (a == 0x7f5eu || (a == 0x7f51u && (data & 1u))) {
        status_override_update(0x02u, 0x00u);
    }
}

static inline void io_shadow_cpu_write(uint16_t off, uint8_t data) {
    uint16_t a = io_canon(off);
    uint32_t i = a & 0x03ffu;
    /* Status/suspend addresses are command/status ports, not latches that echo
     * the byte just written. Core0 republishes their true value. */
    if (a >= 0x7f53u && a <= 0x7f5fu) return;
    if (a == 0x7f48u) data &= 1u;
    else if (a == 0x7f4cu) data &= 3u;
    else if (a == 0x7f4eu) data &= 0x7fu;
    else if (a == 0x7f50u) data &= 0x77u;
    else if (a == 0x7f51u || a == 0x7f52u) data &= 1u;
    s_io_shadow[i] = data;

    /* GPR windows are mirrors of one another. Make immediate CPU readback
     * deterministic even before core0 drains the event queue. */
    if (a >= 0x7f80u && a <= 0x7fafu)
        s_io_shadow[(a + 0x40u) & 0x03ffu] = data;
    else if (a >= 0x7fc0u && a <= 0x7fefu)
        s_io_shadow[(a - 0x40u) & 0x03ffu] = data;
}

static inline bool write_changes_status(uint16_t off) {
    uint16_t a = io_canon(off);
    if (a == 0x7f47u || a == 0x7f48u || a == 0x7f4fu || a == 0x7f53u ||
        (a >= 0x7f55u && a <= 0x7f5eu) || (a == 0x7f51u)) return true;
    return false;
}

static inline void ioq_push(uint16_t off, uint8_t data, uint16_t epoch, bool status_evt) {
    uint32_t head = s_ioq_head;
    uint32_t next = (head + 1u) & (CX4_IOQ_N - 1u);
    if (next == s_ioq_tail) { s_ioq_dropped++; return; }
    s_ioq[head].off = off;
    s_ioq[head].epoch = epoch;
    s_ioq[head].data = data;
    s_ioq[head].status_evt = status_evt ? 1u : 0u;
    membar();
    s_ioq_head = next;
    uint32_t depth = (next - s_ioq_tail) & (CX4_IOQ_N - 1u);
    if (depth > s_ioq_highwater) s_ioq_highwater = depth;
}

static bool ioq_pop_core0(cx4_io_write_evt_t *out) {
    if (s_ioq_tail == s_ioq_head) return false;
    uint32_t tail = s_ioq_tail;
    membar();
    out->off = s_ioq[tail].off;
    out->epoch = s_ioq[tail].epoch;
    out->data = s_ioq[tail].data;
    out->status_evt = s_ioq[tail].status_evt;
    s_ioq_tail = (tail + 1u) & (CX4_IOQ_N - 1u);
    return true;
}

static void publish_io_shadow_core0(void) {
    if (!s_core) return;
    /* Only the implemented/high-value IO range. 176 byte reads are cheap on
     * core0 and remove all HG51B object access from the timing-critical core1. */
    for (uint16_t a = 0x7f40u; a <= 0x7fefu; ++a)
        s_io_shadow[a & 0x03ffu] = cx4_read(s_core, a);
    membar();
}

static inline void core_step_master(uint32_t master_cycles) {
    if (!s_core) return;
    s_master_clock += master_cycles;
    cx4_sync(s_core, s_master_clock);
}

static inline uint8_t core_status(void) {
    return s_core ? cx4_read(s_core, 0x7f5eu) : 0u;
}

/* IMPORTANT: the latency-critical bus responder runs on core1. It must never
 * execute a long DSP burst, otherwise SNES cycles would pass unanswered.
 * Core1 only records that work may have started; core0 services the LLE core
 * in the normal main loop while core1 keeps D0-D7 responsive. */
static void core_reset_now(void) {
    if (!s_core) return;
    cx4_reset(s_core);
    cx4_synthesize_data_rom(s_core);
    s_master_clock += 1;
    s_work_pending = false;
    s_halt_shadow = true;
    s_ioq_tail = s_ioq_head;
    __atomic_store_n(&s_status_override, 0u, __ATOMIC_RELEASE);
    publish_io_shadow_core0();
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
    s_dram = s_core ? cx4_ram_ptr(s_core, 0x6000u) : NULL;
    memset((void *)s_io_shadow, 0, sizeof(s_io_shadow));
    s_ioq_head = s_ioq_tail = 0;
    s_ioq_dropped = 0;
    s_pio_writes = s_pio_dram_writes = s_pio_io_writes = 0;
    s_sio_shadow_writes = 0;
    memset(s_fwd, 0, sizeof(s_fwd));
    s_fwd_head = 0; s_bus_seq = 0;
    s_fwd_puts = s_fwd_hits = s_sio_state_blocked = 0;
    s_hle_badparts = 0;
    s_hle_sprite_jobs = s_hle_sprite_objects = s_hle_oam_entries = s_hle_rom_ptr_fail = 0;
    s_hle_rom_lowhalf = s_hle_rom_oor = 0;
    s_hle_zero_pad_reads = s_hle_zero_pad_ptrs = s_hle_snapshot_jobs = 0;
    s_hle_parts_max = 0;
    s_hle_last_ptr = 0; s_hle_last_subcmd = 0xffu;
    if (s_core) publish_io_shadow_core0();

    s_master_clock = 1;
    for (unsigned v = 0; v < 256u; ++v)
        s_packed_data[v] = (((uint32_t)v & 0x07u) << 2) | (((uint32_t)v & 0xf8u) << 3);
    /* Gameplay build auto-arms when an injected ROM/core is available. */
    s_armed = (s_core != NULL);
    s_driving = false;
    s_reset_requested = false;
    s_work_pending = false;
    s_halt_shadow = true;
    __atomic_store_n(&s_status_override, 0u, __ATOMIC_RELEASE);
    s_status_override_sets = 0;
    s_status_forced_reads = 0;
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

    /* FINAL bus strategy:
     *   - follow physical /RD directly, not PHI2;
     *   - wait a few CPU clocks for /ROMSEL to settle;
     *   - only ever drive when /ROMSEL=HIGH, /WRAMSEL=HIGH and /WR=HIGH;
     *   - decode only the Cx4 $6000-$7FFF aperture;
     *   - hold DATA until the real /RD rising edge.
     *
     * The passive analyzer proved that real cartridge ROM reads have /ROMSEL
     * LOW while Cx4 aperture accesses have /ROMSEL HIGH. This qualifier is the
     * hard safety boundary that was missing from the early gameplay builds. */
    for (;;) {
        while (sio_hw->gpio_hi_in & RD_HI_MASK) tight_loop_contents();

        /* Roughly 20 ns at the 200 MHz gameplay clock. Address/control are
         * already valid by /RD, but /ROMSEL was observed to settle slightly
         * inside the read window on this exact prototype. */
        __asm volatile("nop; nop; nop; nop;" ::: "memory");

        uint32_t hi = sio_hw->gpio_hi_in;
        uint32_t lo = sio_hw->gpio_in;
        s_bus_seq++;

        const bool qualified =
            ((hi & RD_HI_MASK) == 0u) &&
            ((hi & ROMSEL_HI_MASK) != 0u) &&
            ((hi & WRAMSEL_HI_MASK) != 0u) &&
            ((lo & WR_MASK) != 0u) &&
            raw_is_cx4(lo, hi);

        if (qualified && s_armed) {
            const uint16_t off = raw_cx4_offset(lo, hi);
            uint8_t v;
            bool io_read = false;

            if (off_is_io(off)) {
                io_read = true;
                v = s_io_shadow[io_index(off)];
                if (io_is_status_read(off)) v = status_apply_override_fast(v);
            } else {
                const uint32_t di = (uint32_t)(off & 0x0fffu);
                v = (s_dram && di < 0x0c00u) ? s_dram[di] : 0u;
            }

            /* Timing rule: once the byte is known, make it visible immediately.
             * Diagnostics happen only after OE is already asserted. */
            data_drive(v);
            s_driven_reads++;
            s_cpu_reads++;
            if (io_read) s_io_reads++; else s_dram_reads++;
            s_last_read_addr = off;
            s_last_read_data = v;
        }

        /* Never release from PHI2. Keep the byte valid for the full physical
         * read strobe, then return to Hi-Z immediately at /RD rising edge. */
        while ((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u) tight_loop_contents();
        if (s_driving) data_release();
    }
}

/* Feed one write captured by the proven PIO+DMA path. This function runs on
 * core0 and is the ONLY producer of LLE write events in v0.3.8. */
void cx4bus_pio_write(uint32_t address, uint8_t data) {
    uint8_t bank = (uint8_t)(address >> 16);
    uint16_t off = (uint16_t)address;
    if (!(bank <= 0x3fu || (bank >= 0x80u && bank <= 0xbfu))) return;
    if (off < 0x6000u || off > 0x7fffu) return;

    /* CX4 exposes 3 KiB DRAM at $6000-$6BFF and mirror at $7000-$7BFF. */
    if ((off >= 0x6000u && off <= 0x6bffu) ||
        (off >= 0x7000u && off <= 0x7bffu)) {
        uint32_t di = (uint32_t)(off & 0x0fffu);
        if (di < 0x0c00u && s_dram) s_dram[di] = data;
        s_pio_writes++;
        s_pio_dram_writes++;
        return;
    }

    /* IO is mirrored through $6C00-$6FFF / $7C00-$7FFF. */
    if (!off_is_io(off)) return;

    io_shadow_cpu_write(off, data);
    const bool status_evt = write_changes_status(off);
    if (status_evt) status_on_cpu_write(off, data);

    /* FINAL GAMEPLAY normally leaves HLE disabled and sends every job to LLE. */
    if (hle_try_runtime_job(off, data)) {
        s_pio_writes++;
        s_pio_io_writes++;
        return;
    }

    const uint16_t epoch = status_evt ? status_override_epoch() : 0u;
    ioq_push(off, data, epoch, status_evt);
    s_pio_writes++;
    s_pio_io_writes++;
}

void cx4bus_service(void) {
    /* v0.3.8: replay only PIO/DMA-authoritative writes into the HG51B, while
     * streaming the core in bounded slices.
     * Real software may touch interface registers while cache/DMA/DSP work is
     * active. The upstream core already models whether a launch is accepted
     * (for example $7F4F only starts when HALT is true), so the correct host
     * behaviour is to replay every captured write in order and keep advancing
     * the core in bounded slices. This also guarantees core1 gets frequent
     * queue service and prevents PB/PC/base writes from being lost. */
    if (!s_core) return;

    if (s_reset_requested) {
        s_reset_requested = false;
        core_reset_now();
    }

    uint32_t depth = (s_ioq_head - s_ioq_tail) & (CX4_IOQ_N - 1u);
    unsigned budget = depth > 4096u ? 2048u : (depth > 1024u ? 1024u : 256u);
    bool touched = false;
    cx4_io_write_evt_t e;

    while (budget-- && ioq_pop_core0(&e)) {
        const uint16_t a = canonical_io(e.off);
        const bool launch = write_starts_job(e.off);
        if (launch) s_jobs_started++;

        /* The core itself decides if the operation is legal in the current
         * HALT/cache/DMA state. Do not artificially wait for a previous job. */
        cx4_write(s_core, e.off, e.data);
        touched = true;

        if (launch) {
            s_work_pending = true;
            if (a == 0x7f4fu) s_halt_shadow = false;
        }
        if (a == 0x7f53u) {
            s_halt_shadow = true;
            s_work_pending = false;
        } else if (a == 0x7f5du) {
            s_work_pending = true;
        }

        /* Once the write has reached the single-owner core, its true status can
         * replace the immediate core1 handshake. Epoch matching prevents an
         * older event from clearing a newer status override. */
        if (e.status_evt) {
            publish_io_shadow_core0();
            status_override_ack_if_unchanged(e.epoch);
        }
    }

    if (touched) publish_io_shadow_core0();

    uint8_t st = core_status();
    if ((st & 0xc0u) || (st & 0x01u)) s_work_pending = true;

    if (s_work_pending) {
        /* Keep slices short while the queue has traffic; turbo harder only when
         * core1 has nothing waiting. This is functional/timing bring-up, not yet
         * cycle-perfect 20 MHz scheduling. */
        depth = (s_ioq_head - s_ioq_tail) & (CX4_IOQ_N - 1u);
        uint32_t slice = depth ? 256u : 4096u;
        core_step_master(slice);
        s_turbo_runs++;
        s_job_chunks++;
        s_stream_slices++;
        s_stream_master += slice;
        publish_io_shadow_core0();

        st = core_status();
        if ((st & 0xc0u) == 0u && (st & 0x01u) == 0u) {
            s_work_pending = false;
            s_halt_shadow = true;
            s_jobs_finished++;
            s_idle_transitions++;
        }
    }
}

void cx4bus_launch_core1(void) {
    if (s_core1_started) return;
    multicore_launch_core1(cx4bus_core1);
    while (!s_core1_started) tight_loop_contents();
}

void cx4bus_print_trace(void) {
    uint32_t end = s_trace_seq;
    uint32_t n = end < CX4_TRACE_N ? end : CX4_TRACE_N;
    uint32_t start = end - n;
    printf("CX4TRACE count=%lu seq=%lu (oldest->newest)\n",
           (unsigned long)n, (unsigned long)end);
    for (uint32_t k = 0; k < n; ++k) {
        uint32_t seq = start + k;
        cx4_trace_entry_t e = s_trace[seq & (CX4_TRACE_N - 1u)];
        printf("  %c %04X %02X\n", e.op ? e.op : '?', e.off, e.data);
    }
}

void cx4bus_print_runs(void) {
    if (!s_core) { printf("CX4RUNS core=0\n"); return; }
    Cx4RunEvent ring[16];
    uint32_t n = cx4_run_ring_copy(s_core, ring, 16u);
    printf("CX4RUNS count=%lu showing=%lu\n",
           (unsigned long)cx4_run_ring_count(s_core), (unsigned long)n);
    for (uint32_t i = 0; i < n; ++i)
        printf("  run seq=%lu base=%06lX pb=%04X pc=%02X\n",
               (unsigned long)ring[i].seq, (unsigned long)ring[i].base,
               ring[i].pb, ring[i].pc);
}

static void self_sync_until_idle(Cx4 *c, uint64_t *clk) {
    for (unsigned n = 0; n < 8192u; ++n) {
        uint8_t st = cx4_read(c, 0x7f5eu);
        if ((st & 0xc0u) == 0u) return;
        *clk += 4096u;
        cx4_sync(c, *clk);
    }
}

void cx4bus_selfcheck(void) {
    if (s_armed) { printf("CX4SELF REFUSED armed=1; use CX4DISARM first\n"); return; }
    if (!s_rom_slot_valid) { printf("CX4SELF FAIL rom_slot=0\n"); return; }
    Cx4 *t = cx4_create(s_game_rom, s_game_rom_size, NULL, 0);
    if (!t) { printf("CX4SELF FAIL alloc\n"); return; }
    cx4_synthesize_data_rom(t);
    uint64_t clk = 1;

    /* Known X2 diagnostic program location: base $028000, PB $000E. Exercise
     * the two entry PCs used by the immediate-register/data-ROM tests. */
    cx4_write(t, 0x7f49u, 0x00u);
    cx4_write(t, 0x7f4au, 0x80u);
    cx4_write(t, 0x7f4bu, 0x02u);
    cx4_write(t, 0x7f4du, 0x0eu);
    cx4_write(t, 0x7f4eu, 0x00u);
    cx4_write(t, 0x7f48u, 0x01u);
    self_sync_until_idle(t, &clk);

    cx4_write(t, 0x7f4fu, 0x5cu);
    self_sync_until_idle(t, &clk);
    cx4_write(t, 0x7f4fu, 0x89u);
    self_sync_until_idle(t, &clk);

    uint32_t lo=0, hi=0;
    cx4_rdrom_index_range(t, &lo, &hi);
    printf("CX4SELF runs=%lu insns=%llu rdrom=%lu distinct=%lu range=%lu-%lu firmware=%d locked=%d status=%02X\n",
           (unsigned long)cx4_run_ring_count(t),
           (unsigned long long)cx4_instructions_executed(t),
           (unsigned long)cx4_rdrom_hits(t),
           (unsigned long)cx4_rdrom_distinct(t),
           (unsigned long)lo, (unsigned long)hi,
           cx4_firmware_loaded(t), cx4_locked(t), cx4_read(t, 0x7f5eu));
    Cx4RunEvent ring[4];
    uint32_t n = cx4_run_ring_copy(t, ring, 4u);
    for (uint32_t i=0; i<n; ++i)
        printf("  self run seq=%lu base=%06lX pb=%04X pc=%02X\n",
               (unsigned long)ring[i].seq, (unsigned long)ring[i].base,
               ring[i].pb, ring[i].pc);
    cx4_destroy(t);
}

void cx4bus_print_status(void) {
    size_t rom_size = (size_t)s_game_rom_size;
    uint64_t insns = s_core ? cx4_instructions_executed(s_core) : 0;
    uint32_t rdrom = s_core ? cx4_rdrom_hits(s_core) : 0;
    uint32_t runs = s_core ? cx4_run_ring_count(s_core) : 0;
    int fw = s_core ? cx4_firmware_loaded(s_core) : 0;
    int locked = s_core ? cx4_locked(s_core) : 0;

    printf("CX4STAT mode=FINAL_GAMEPLAY_V1 armed=%u drive=%s core1=%u core=%u gate=RD_ROMSELHI_WRAMSELHI_WRHI hold=RD_RISE "
           "rom_slot=%u rom_bytes=%lu rom_crc=%08lX firmware=%d locked=%d runs=%lu insns=%llu rdrom=%lu service=%llu pending=%u hle_sprite=%u hle_jobs=%llu hle_groups=%llu hle_oam=%llu hle_romfail=%llu hle_lowhalf=%llu hle_oor=%llu hle_zpad=%llu hle_zptr=%llu hle_snap=%llu hle_pmax=%lu hle_badparts=%llu hle_ptr=%06lX hle_sub=%02X "
           "fwd_put=%llu fwd_hit=%llu sio_block=%llu ioq=%lu ioq_drop=%lu ioq_hi=%lu pio_w=%llu pio_dram=%llu pio_io=%llu sio_shadow=%llu jobs=%llu/%llu job_chunks=%llu job_to=%llu stream=%llu master=%llu idle=%llu halt_sh=%u stat_ovr=%04lX stat_set=%llu stat_forced_r=%llu cpu_r=%llu cpu_w=%llu driven=%llu dram_r=%llu dram_w=%llu io_r=%llu io_w=%llu "
           "vec_resets=%llu state_resets=%llu last_r=%04lX:%02X last_w=%04lX:%02X\n",
           s_armed ? 1u : 0u, s_driving ? "ON" : "OFF", s_core1_started ? 1u : 0u,
           s_core ? 1u : 0u, s_rom_slot_valid ? 1u : 0u, (unsigned long)rom_size,
           (unsigned long)s_game_rom_crc32, fw, locked,
           (unsigned long)runs, (unsigned long long)insns, (unsigned long)rdrom,
           (unsigned long long)s_turbo_runs, s_work_pending ? 1u : 0u,
           s_sprite_hle_enabled ? 1u : 0u,
           (unsigned long long)s_hle_sprite_jobs,
           (unsigned long long)s_hle_sprite_objects,
           (unsigned long long)s_hle_oam_entries,
           (unsigned long long)s_hle_rom_ptr_fail,
           (unsigned long long)s_hle_rom_lowhalf,
           (unsigned long long)s_hle_rom_oor,
           (unsigned long long)s_hle_zero_pad_reads,
           (unsigned long long)s_hle_zero_pad_ptrs,
           (unsigned long long)s_hle_snapshot_jobs,
           (unsigned long)s_hle_parts_max,
           (unsigned long long)s_hle_badparts,
           (unsigned long)s_hle_last_ptr, s_hle_last_subcmd,
           (unsigned long long)s_fwd_puts,
           (unsigned long long)s_fwd_hits,
           (unsigned long long)s_sio_state_blocked,
           (unsigned long)((s_ioq_head - s_ioq_tail) & (CX4_IOQ_N - 1u)),
           (unsigned long)s_ioq_dropped,
           (unsigned long)s_ioq_highwater,
           (unsigned long long)s_pio_writes,
           (unsigned long long)s_pio_dram_writes,
           (unsigned long long)s_pio_io_writes,
           (unsigned long long)s_sio_shadow_writes,
           (unsigned long long)s_jobs_started,
           (unsigned long long)s_jobs_finished,
           (unsigned long long)s_job_chunks,
           (unsigned long long)s_job_timeouts,
           (unsigned long long)s_stream_slices,
           (unsigned long long)s_stream_master,
           (unsigned long long)s_idle_transitions,
           s_halt_shadow ? 1u : 0u,
           (unsigned long)(__atomic_load_n(&s_status_override, __ATOMIC_ACQUIRE) & 0xffffu),
           (unsigned long long)s_status_override_sets,
           (unsigned long long)s_status_forced_reads,
           (unsigned long long)s_cpu_reads, (unsigned long long)s_cpu_writes,
           (unsigned long long)s_driven_reads,
           (unsigned long long)s_dram_reads, (unsigned long long)s_dram_writes,
           (unsigned long long)s_io_reads, (unsigned long long)s_io_writes,
           (unsigned long long)s_reset_vectors, (unsigned long long)s_state_resets,
           (unsigned long)s_last_read_addr, s_last_read_data,
           (unsigned long)s_last_write_addr, s_last_write_data);
}
