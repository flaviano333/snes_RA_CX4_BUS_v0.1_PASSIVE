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
 * CX4 BUS v0.3.5 LLE FAST-STATUS HANDSHAKE
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

/* v0.3.3 keeps the game ROM in a reserved flash slot instead of linking the
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
/* Fast CPU-visible interface. Core1 NEVER calls cx4_read/cx4_write in v0.3.5.
 * Core0 is the sole owner of the HG51B control/register state; core1 serves
 * the physical SNES bus from this shadow plus the core dataRAM byte array. */
static uint8_t *s_dram = NULL;
static volatile uint8_t s_io_shadow[0x400];

typedef struct { uint16_t off; uint8_t data; } cx4_io_write_evt_t;
#define CX4_IOQ_N 512u
static volatile cx4_io_write_evt_t s_ioq[CX4_IOQ_N];
static volatile uint32_t s_ioq_head = 0;
static volatile uint32_t s_ioq_tail = 0;
static volatile uint32_t s_ioq_dropped = 0;
static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile bool s_reset_requested = false;
static volatile bool s_work_pending = false;
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
    /* Only operations that can actually make the engine advance. Halt and
     * suspend writes must not manufacture a fresh turbo run. */
    return a == 0x7f47u || a == 0x7f48u || a == 0x7f4fu ||
           (a >= 0x7f56u && a <= 0x7f5du);
}

static inline void membar(void) { __asm volatile("dmb sy" ::: "memory"); }

static inline uint16_t io_canon(uint16_t off) { return canonical_io(off); }
static inline uint32_t io_index(uint16_t off) { return (uint32_t)(io_canon(off) & 0x03ffu); }

static inline bool io_is_status_read(uint16_t off) {
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
    uint8_t st = status_effective_now();
    bool halted = (st & 0x40u) == 0u;

    if ((a == 0x7f47u || a == 0x7f48u) && halted) {
        /* Cache fill / DMA: both RUNNING and BUSY become visible immediately. */
        status_override_update(0xc0u, 0xc0u);
    } else if (a == 0x7f4fu && halted) {
        /* Program execution: RUNNING immediately; BUSY is only cache/DMA/bus. */
        status_override_update(0xc0u, 0x40u);
    } else if (a == 0x7f53u) {
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

static inline void ioq_push(uint16_t off, uint8_t data) {
    uint32_t head = s_ioq_head;
    uint32_t next = (head + 1u) & (CX4_IOQ_N - 1u);
    if (next == s_ioq_tail) { s_ioq_dropped++; return; }
    s_ioq[head].off = off;
    s_ioq[head].data = data;
    membar();
    s_ioq_head = next;
}

static uint32_t ioq_drain_core0(void) {
    uint32_t n = 0;
    while (s_ioq_tail != s_ioq_head) {
        uint32_t tail = s_ioq_tail;
        membar();
        uint16_t off = s_ioq[tail].off;
        uint8_t data = s_ioq[tail].data;
        s_ioq_tail = (tail + 1u) & (CX4_IOQ_N - 1u);
        if (s_core) cx4_write(s_core, off, data);
        uint16_t a = canonical_io(off);
        if (a == 0x7f53u || a == 0x7f55u)
            s_work_pending = false;  /* HALT / indefinite SUSPEND */
        else if (write_can_start_work(off))
            s_work_pending = true;
        ++n;
    }
    return n;
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
    if (s_core) publish_io_shadow_core0();

    s_master_clock = 1;
    s_armed = false;
    s_driving = false;
    s_reset_requested = false;
    s_work_pending = false;
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

    for (;;) {
        /* Start every transaction from PHI2 low, then sample shortly AFTER the
         * rising edge. v0.3.2 sampled /RD,/WR/address immediately at the edge,
         * before the SNES bus had the same settling margin used by our proven
         * PIO capture path. */
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();

        /* Reset is also core0-owned in v0.3.5. Core1 only raises the request. */
        while (!(sio_hw->gpio_in & PHI2_MASK)) tight_loop_contents();
        __asm volatile("nop; nop;" ::: "memory");

        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool is_read = (hi & RD_HI_MASK) == 0u;
        bool is_write = (lo & WR_MASK) == 0u;

        if (is_read) {
            if (raw_is_cx4(lo, hi)) {
                uint16_t off = raw_cx4_offset(lo, hi);

                /* v0.3.5: ZERO Cx4-core calls on core1. DRAM is a direct byte
                 * lookup; IO comes from a core0-published shadow. This removes
                 * the dual-core race and cuts physical read latency sharply. */
                uint8_t v = 0u;
                if (off_is_io(off)) {
                    v = s_io_shadow[io_index(off)];
                    if (io_is_status_read(off)) v = status_apply_override(v);
                    s_io_reads++;
                } else {
                    uint32_t di = (uint32_t)(off & 0x0fffu);
                    v = (s_dram && di < 0x0c00u) ? s_dram[di] : 0u;
                    s_dram_reads++;
                }
                s_cpu_reads++;
                s_last_read_addr = off;
                s_last_read_data = v;
                trace_io('R', off, v);
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

            /* Capture write data while PHI2 is still HIGH. The old responder
             * waited for the falling edge and only then executed C code to read
             * GPIO, which can miss the SNES data-hold window. */
            __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;" ::: "memory");
            uint8_t v = raw_data(sio_hw->gpio_in);
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();

            if (off_is_io(off)) {
                io_shadow_cpu_write(off, v);
                status_on_cpu_write(off, v);
                ioq_push(off, v);
                s_io_writes++;
            } else {
                uint32_t di = (uint32_t)(off & 0x0fffu);
                if (s_dram && di < 0x0c00u) s_dram[di] = v;
                s_dram_writes++;
            }
            s_cpu_writes++;
            s_last_write_addr = off;
            s_last_write_data = v;
            trace_io('W', off, v);
            continue;
        }

        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
    }
}

void cx4bus_service(void) {
    /* Core0 is the ONLY owner of HG51B IO/control mutation and execution.
     * Core1 only updates byte-addressable DRAM and pushes IO writes. */
    if (!s_core) return;

    if (s_reset_requested) {
        s_reset_requested = false;
        core_reset_now();
    }

    /* Snapshot the status-override epoch. If core1 receives another status-
     * changing write while we service this batch, the compare-exchange below
     * will deliberately leave that newer override in place. */
    uint16_t status_epoch = status_override_epoch();
    bool published = false;

    uint32_t drained = ioq_drain_core0();
    if (drained) {
        publish_io_shadow_core0();
        published = true;
    }

    if (s_work_pending) {
        /* Turbo-to-idle remains intentional for functional bring-up. Crucially,
         * the SNES now sees BUSY/RUNNING immediately and waits while these slices
         * execute, rather than racing ahead before core0 drains $7F48/$7F4F. */
        core_step_master(8192u);
        s_turbo_runs++;
        publish_io_shadow_core0();
        published = true;

        uint8_t st = cx4_read(s_core, 0x7f53u);
        if ((st & 0xc0u) == 0u && (st & 0x01u) == 0u) s_work_pending = false;
    }

    if (published) status_override_ack_if_unchanged(status_epoch);
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

void cx4bus_print_status(void) {
    size_t rom_size = (size_t)s_game_rom_size;
    uint64_t insns = s_core ? cx4_instructions_executed(s_core) : 0;
    uint32_t rdrom = s_core ? cx4_rdrom_hits(s_core) : 0;
    uint32_t runs = s_core ? cx4_run_ring_count(s_core) : 0;
    int fw = s_core ? cx4_firmware_loaded(s_core) : 0;
    int locked = s_core ? cx4_locked(s_core) : 0;

    printf("CX4STAT mode=LLE_FAST_STATUS_HANDSHAKE_V0.3.5 armed=%u drive=%s core1=%u core=%u "
           "rom_slot=%u rom_bytes=%lu rom_crc=%08lX firmware=%d locked=%d runs=%lu insns=%llu rdrom=%lu service=%llu pending=%u "
           "ioq=%lu ioq_drop=%lu stat_ovr=%04lX stat_set=%llu stat_forced_r=%llu cpu_r=%llu cpu_w=%llu driven=%llu dram_r=%llu dram_w=%llu io_r=%llu io_w=%llu "
           "vec_resets=%llu state_resets=%llu last_r=%04lX:%02X last_w=%04lX:%02X\n",
           s_armed ? 1u : 0u, s_driving ? "ON" : "OFF", s_core1_started ? 1u : 0u,
           s_core ? 1u : 0u, s_rom_slot_valid ? 1u : 0u, (unsigned long)rom_size,
           (unsigned long)s_game_rom_crc32, fw, locked,
           (unsigned long)runs, (unsigned long long)insns, (unsigned long)rdrom,
           (unsigned long long)s_turbo_runs, s_work_pending ? 1u : 0u,
           (unsigned long)((s_ioq_head - s_ioq_tail) & (CX4_IOQ_N - 1u)),
           (unsigned long)s_ioq_dropped,
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
