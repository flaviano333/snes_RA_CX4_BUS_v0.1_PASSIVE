#include "cx4_active_bus.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "hardware/pio.h"
#include "status42_drive.pio.h"
#include "hardware/regs/addressmap.h"
#include "cx4.h"

/*
 * CX4 PIO STATUS42 FINAL V9
 *
 * This build deliberately abandons command-by-command HLE.  Every physical write is captured as one coherent SIO snapshot by Core1 and
 * queued to Core0. This deliberately removes the old two-PIO/two-DMA pairing
 * path, which could pair the right address with the wrong data. Core0 feeds the
 * queued write into the instruction-level HG51B core and advances a synthetic
 * master timeline until the real Cx4 state reports idle.
 *
 * No external SYSTEM CLK and no /RESET wire are used. Core1 remains the physical
 * read responder and the sole write-event producer; Core0 is the sole Cx4 owner.
 */

#define PIN_PHI2     0u
#define PIN_WR       1u
#define PIN_RD       35u
#define PIN_ROMSEL   38u
#define PIN_WRAMSEL  39u

#define DATA_MASK ((1u << 2) | (1u << 3) | (1u << 4) | \
                   (1u << 6) | (1u << 7) | (1u << 8) | \
                   (1u << 9) | (1u << 10))
#define PHI2_MASK       (1u << PIN_PHI2)
#define WR_MASK         (1u << PIN_WR)
#define RD_HI_MASK      (1u << (PIN_RD - 32u))
#define ROMSEL_HI_MASK  (1u << (PIN_ROMSEL - 32u))
#define WRAMSEL_HI_MASK (1u << (PIN_WRAMSEL - 32u))
#define A13_MASK        (1u << 25)
#define A14_MASK        (1u << 27)
#define A15_MASK        (1u << 28)
#define A22_HI_MASK     (1u << (36u - 32u))

#define CX4_ROM_SLOT_FLASH_OFFSET (4u * 1024u * 1024u)
#define CX4_ROM_SLOT_HEADER_SIZE 256u
#define CX4_ROM_SLOT_MAX_BYTES (4u * 1024u * 1024u - CX4_ROM_SLOT_HEADER_SIZE)
#define CX4_ROM_SLOT_MAGIC 0x52345843u
#define CX4_ROM_SLOT_VERSION 1u

/* A transaction advances the HG51B in 4096 SNES-master-cycle chunks.  This is
 * not wall-clock timing: it is a monotonic virtual timeline used only by the
 * upstream 20 MHz / 21.477 MHz rate converter.  4096 chunks gives a generous
 * ~0.78 s of emulated time before declaring a wedge. */
#define TX_MASTER_CHUNK 512u
/* V8.2 no longer aborts a live DSP after an arbitrary number of chunks.
 * Long-running detection is diagnostic only; CPU writes continue to be
 * consumed and can legitimately change state while the DSP is running. */
#define TX_LONGRUN_CHUNKS 32768u
#define SELF_MAX_CHUNKS 32768u
#define SERVICE_WRITE_BUDGET 384u
#define SERVICE_TX_CHUNK_BUDGET 2u

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
static Cx4 *s_cx4 = NULL;

/* Core1 never dereferences the Cx4 object.  It serves this byte mirror so the
 * physical read responder remains lock-free while Core0 executes HG51B code. */
static volatile uint8_t s_shadow[0x2000];
static volatile uint8_t s_status_proxy = 0;
static volatile uint8_t s_tx_active = 0;
static volatile uint32_t s_pending_run = 0;   /* queued $7F4F */
static volatile uint32_t s_pending_busy = 0;  /* queued $7F47/$7F48 */
static uint64_t s_virtual_master = 1;

static volatile bool s_armed = false;
static volatile bool s_core1_started = false;
static volatile bool s_driving = false;
static volatile bool s_reset_requested = false;
static volatile uint8_t s_reset_arm = 0;

static volatile uint64_t s_cpu_reads = 0;
static volatile uint64_t s_cpu_writes = 0;
static volatile uint64_t s_driven_reads = 0;
static volatile uint64_t s_reset_vectors = 0;

/* Physical read timing diagnostics.  A predrive is prepared while PHI2 is low,
 * before the CPU sampling window.  late is the fallback path where the cycle
 * was not stable soon enough and we had to drive after PHI2 rose. */
static volatile uint64_t s_read_predrive = 0;
static volatile uint64_t s_read_late = 0;
static volatile uint64_t s_read_retarget = 0;

/* FINAL binary read-path diagnostic.
 * Core1 only decides that the address is $xx7F5E while PHI2 is LOW.  PIO0 then
 * owns the electrical timing and drives 0x42 exactly from PHI2 rise to fall.
 * This deliberately avoids the late software responder for the critical test. */
#define STAT42_VALUE 0x42u
#define STAT42_PIO_IRQ 0u
static PIO s_stat_pio = pio0;
static int s_stat_sm = -1;
static uint s_stat_prog_off = 0;
static volatile bool s_stat42_mode = true;
static volatile bool s_stat42_pio_ready = false;
static volatile uint64_t s_stat42_arms = 0;
static volatile uint64_t s_stat42_forced_reads = 0;
static volatile uint64_t s_stat42_false_arms = 0;
static volatile uint64_t s_stat42_pin_ok = 0;
static volatile uint64_t s_stat42_pin_bad = 0;
static volatile uint8_t s_stat42_pin_last = 0xffu;
static volatile uint64_t s_stat42_cmd_after = 0;
static volatile bool s_stat42_saw_read = false;

/* Single-producer (Core1), single-consumer (Core0) write queue. The queue is
 * deliberately large enough to absorb short bursts while HG51B is executing. */
typedef struct { uint16_t off; uint8_t data; uint8_t pad; } write_evt_t;
#define WRITE_Q_N 16384u
#define WRITE_Q_MASK (WRITE_Q_N - 1u)
static write_evt_t s_write_q[WRITE_Q_N];
static volatile uint32_t s_write_head = 0; /* producer-owned */
static volatile uint32_t s_write_tail = 0; /* consumer-owned */
static volatile uint64_t s_q_pushed = 0;
static volatile uint64_t s_q_popped = 0;
static volatile uint64_t s_q_dropped = 0;
static volatile uint32_t s_q_peak = 0;
static volatile uint8_t s_q_last_phys_cmd = 0xffu;
static volatile uint8_t s_q_last_core_cmd = 0xffu;
static volatile uint64_t s_cmd_phys_00 = 0, s_cmd_phys_af = 0, s_cmd_phys_other = 0;
static volatile uint64_t s_cmd_core_00 = 0, s_cmd_core_af = 0, s_cmd_core_other = 0;
static volatile uint64_t s_cmd_core_idle = 0, s_cmd_core_running = 0;

static volatile uint64_t s_state_resets = 0;
static volatile uint32_t s_last_read_addr = 0;
static volatile uint32_t s_last_write_addr = 0;
static volatile uint8_t s_last_read_data = 0;
static volatile uint8_t s_last_write_data = 0;

static volatile uint64_t s_tx_jobs = 0;
static volatile uint64_t s_tx_done = 0;
static volatile uint64_t s_tx_timeout = 0;
static volatile uint64_t s_tx_chunks = 0;
static volatile uint64_t s_tx_busy_reads = 0;
static volatile uint64_t s_tx_early_arms = 0;
static volatile uint64_t s_tx_locked = 0;
static volatile uint8_t s_tx_last_entry = 0xffu;
static volatile uint8_t s_tx_last_status = 0;
static volatile uint32_t s_tx_last_chunks = 0;
static volatile uint32_t s_tx_progress_chunks = 0;
static volatile uint64_t s_service_calls = 0;
static volatile uint64_t s_service_yields = 0;
static volatile uint64_t s_service_write_budget_hits = 0;
static volatile uint64_t s_service_tx_budget_hits = 0;

/* Tiny physical IO trace. */
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


static void stat42_pio_set_pinmux(bool use_pio) {
    const uint pins[8] = {2,3,4,6,7,8,9,10};
    for (unsigned i=0;i<8;i++) {
        gpio_set_function(pins[i], use_pio ? GPIO_FUNC_PIO0 : GPIO_FUNC_SIO);
        if (!use_pio) {
            gpio_set_dir(pins[i], GPIO_IN);
            gpio_disable_pulls(pins[i]);
        }
    }
    /* GP5 is intentionally never muxed to PIO. */
    gpio_set_function(5u, GPIO_FUNC_SIO);
    gpio_set_dir(5u, GPIO_IN);
    gpio_disable_pulls(5u);
}

static void stat42_pio_prepare(void) {
    if (!s_stat42_pio_ready) return;
    pio_sm_set_enabled(s_stat_pio, (uint)s_stat_sm, false);
    pio_interrupt_clear(s_stat_pio, STAT42_PIO_IRQ);
    pio_sm_clear_fifos(s_stat_pio, (uint)s_stat_sm);
    pio_sm_restart(s_stat_pio, (uint)s_stat_sm);
    pio_sm_clkdiv_restart(s_stat_pio, (uint)s_stat_sm);
    pio_sm_set_consecutive_pindirs(s_stat_pio, (uint)s_stat_sm, 2u, 9u, false);
    stat42_pio_set_pinmux(true);
    /* Relative GP2..GP10 packed form of D=0x42, with GP5 hole at bit3. */
    pio_sm_put_blocking(s_stat_pio, (uint)s_stat_sm, 0x082u);
    /* Direction mask for GP2..GP10 excluding GP5 (relative bit3). */
    pio_sm_put_blocking(s_stat_pio, (uint)s_stat_sm, 0x1f7u);
    pio_sm_set_enabled(s_stat_pio, (uint)s_stat_sm, true);
}

static void stat42_pio_disable(void) {
    if (!s_stat42_pio_ready) return;
    pio_interrupt_clear(s_stat_pio, STAT42_PIO_IRQ);
    pio_sm_set_enabled(s_stat_pio, (uint)s_stat_sm, false);
    pio_sm_set_consecutive_pindirs(s_stat_pio, (uint)s_stat_sm, 2u, 9u, false);
    stat42_pio_set_pinmux(false);
    data_release();
}

static void stat42_pio_init(void) {
    s_stat_prog_off = pio_add_program(s_stat_pio, &status42_drive_program);
    s_stat_sm = pio_claim_unused_sm(s_stat_pio, true);
    pio_sm_config c = status42_drive_program_get_default_config(s_stat_prog_off);
    sm_config_set_out_pins(&c, 2u, 9u);
    sm_config_set_out_shift(&c, true, false, 32u);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(s_stat_pio, (uint)s_stat_sm, s_stat_prog_off, &c);
    s_stat42_pio_ready = true;
    stat42_pio_prepare();
}

void cx4bus_status42(bool enabled) {
    s_stat42_mode = enabled;
    s_stat42_saw_read = false;
    pio_interrupt_clear(s_stat_pio, STAT42_PIO_IRQ);
    if (enabled) stat42_pio_prepare();
    else stat42_pio_disable();
}

void cx4bus_status42_clear(void) {
    s_stat42_arms = s_stat42_forced_reads = s_stat42_false_arms = 0;
    s_stat42_pin_ok = s_stat42_pin_bad = 0;
    s_stat42_pin_last = 0xffu;
    s_stat42_cmd_after = 0;
    s_stat42_saw_read = false;
}

void cx4bus_print_status42(void) {
    const char *verdict = "WAIT";
    if (!s_stat42_mode) verdict = "OFF";
    else if (s_stat42_arms == 0 || s_stat42_forced_reads == 0) verdict = "FAIL_NO_STATUS_ARM";
    else if (s_stat42_pin_bad && !s_stat42_pin_ok) verdict = "FAIL_DATA_DRIVE";
    else if (s_stat42_forced_reads >= 4 && s_stat42_cmd_after <= 1) verdict = "PASS_CPU_WAITED";
    else if (s_stat42_forced_reads >= 4 && s_stat42_cmd_after >= 8) verdict = "FAIL_CPU_CONTINUED";
    printf("STAT42 mode=%s pio=%u arms=%llu reads=%llu false=%llu pin_ok=%llu pin_bad=%llu pin_last=%02X cmd_after=%llu verdict=%s\n",
           s_stat42_mode?"ON":"OFF", s_stat42_pio_ready?1u:0u,
           (unsigned long long)s_stat42_arms,
           (unsigned long long)s_stat42_forced_reads,
           (unsigned long long)s_stat42_false_arms,
           (unsigned long long)s_stat42_pin_ok,
           (unsigned long long)s_stat42_pin_bad,
           s_stat42_pin_last,
           (unsigned long long)s_stat42_cmd_after,
           verdict);
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

static inline uint16_t canonical_off(uint16_t off) {
    if (off >= 0x7000u && off <= 0x7bffu)
        return (uint16_t)(0x6000u + (off - 0x7000u));
    if (off >= 0x7c00u && off <= 0x7fffu)
        return (uint16_t)(0x6c00u + (off - 0x7c00u));
    return off;
}

static inline void shadow_store(uint16_t off, uint8_t v) {
    uint16_t c = canonical_off(off);
    if (c < 0x6000u || c > 0x6fffu) return;
    s_shadow[c - 0x6000u] = v;
    if (c <= 0x6bffu)
        s_shadow[(0x7000u + (c - 0x6000u)) - 0x6000u] = v;
    else
        s_shadow[(0x7c00u + (c - 0x6c00u)) - 0x6000u] = v;
}

static inline bool is_status_offset(uint16_t off) {
    uint16_t c = canonical_off(off);
    return c == 0x6f5eu;  /* mirror of $7F5E */
}

static inline uint8_t bus_read(uint16_t off) {
    if (is_status_offset(off)) {
        uint8_t v = __atomic_load_n(&s_status_proxy, __ATOMIC_ACQUIRE);
        /* Preserve the real CX4 distinction:
           bit 6 = HG51B running, bit 7 = cache/DMA/bus busy.
           Only synthesize bits for trigger writes that Core1 has physically
           accepted but Core0 has not applied yet. Never promote RUNNING to
           BUSY just because our cooperative service still has tx_active=1. */
        if (__atomic_load_n(&s_pending_run, __ATOMIC_ACQUIRE) ||
            __atomic_load_n(&s_pending_busy, __ATOMIC_ACQUIRE)) v |= 0x40u;
        if (__atomic_load_n(&s_pending_busy, __ATOMIC_ACQUIRE)) v |= 0x80u;
        if (v & 0xc0u) s_tx_busy_reads++;
        return v;
    }
    return s_shadow[(off - 0x6000u) & 0x1fffu];
}

static inline bool is_start_trigger(uint16_t off) {
    return off == 0x7f47u || off == 0x7f48u || off == 0x7f4fu ||
           off == 0x6f47u || off == 0x6f48u || off == 0x6f4fu;
}
static inline bool is_busy_trigger(uint16_t off) {
    return off == 0x7f47u || off == 0x7f48u ||
           off == 0x6f47u || off == 0x6f48u;
}
static inline bool is_run_trigger(uint16_t off) {
    return off == 0x7f4fu || off == 0x6f4fu;
}

static inline uint32_t write_q_depth(void) {
    uint32_t h = __atomic_load_n(&s_write_head, __ATOMIC_ACQUIRE);
    uint32_t t = __atomic_load_n(&s_write_tail, __ATOMIC_ACQUIRE);
    return h - t;
}

static inline bool write_q_push(uint16_t off, uint8_t data) {
    uint32_t h = __atomic_load_n(&s_write_head, __ATOMIC_RELAXED);
    uint32_t t = __atomic_load_n(&s_write_tail, __ATOMIC_ACQUIRE);
    if ((h - t) >= WRITE_Q_N) { s_q_dropped++; return false; }
    s_write_q[h & WRITE_Q_MASK].off = off;
    s_write_q[h & WRITE_Q_MASK].data = data;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&s_write_head, h + 1u, __ATOMIC_RELEASE);
    s_q_pushed++;
    uint32_t depth = (h + 1u) - t;
    if (depth > s_q_peak) s_q_peak = depth;
    if (off == 0x7f4fu) {
        s_q_last_phys_cmd = data;
        if (data == 0x00u) s_cmd_phys_00++;
        else if (data == 0xafu) s_cmd_phys_af++;
        else s_cmd_phys_other++;
    }
    return true;
}

static inline bool write_q_pop(write_evt_t *e) {
    uint32_t t = __atomic_load_n(&s_write_tail, __ATOMIC_RELAXED);
    uint32_t h = __atomic_load_n(&s_write_head, __ATOMIC_ACQUIRE);
    if (t == h) return false;
    *e = s_write_q[t & WRITE_Q_MASK];
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    __atomic_store_n(&s_write_tail, t + 1u, __ATOMIC_RELEASE);
    s_q_popped++;
    return true;
}

static inline void trace_io(uint8_t op, uint16_t off, uint8_t data) {
    if (off < 0x7f40u) return;
    uint32_t i = s_trace_seq++ & (TRACE_N - 1u);
    s_trace[i].off = off; s_trace[i].data = data; s_trace[i].op = op;
}

static void mirror_refresh_all(void) {
    if (!s_cx4) return;

    /* 3 KiB Cx4 data RAM and its $7000 mirror. */
    for (uint16_t i = 0; i < 0x0c00u; ++i) {
        uint8_t *p = cx4_ram_ptr(s_cx4, (uint16_t)(0x6000u + i));
        uint8_t v = p ? *p : 0u;
        s_shadow[i] = v;
        s_shadow[0x1000u + i] = v;
    }

    /* 1 KiB IO window and its $7C00 mirror.  CPU reads have no destructive
       side effects in this core, so a snapshot is safe. */
    for (uint16_t i = 0; i < 0x0400u; ++i) {
        uint8_t v = cx4_read(s_cx4, (uint16_t)(0x6c00u + i));
        s_shadow[0x0c00u + i] = v;
        s_shadow[0x1c00u + i] = v;
    }

    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&s_status_proxy, s_shadow[0x1f5eu], __ATOMIC_RELEASE);
}

static bool transaction_begin_if_needed(void) {
    if (!s_cx4) return false;

    uint8_t st = cx4_read(s_cx4, 0x7f5eu);
    __atomic_store_n(&s_status_proxy, st, __ATOMIC_RELEASE);
    if ((st & 0xc0u) == 0u) return false;

    if (!__atomic_load_n(&s_tx_active, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&s_tx_active, 1u, __ATOMIC_RELEASE);
        s_tx_jobs++;
        s_tx_progress_chunks = 0;
    }
    return true;
}

/* Advance only a small bounded slice of HG51B time.  This MUST return to the
   main loop frequently so TinyUSB/stdio cannot be starved by a long Cx4 job. */
static void transaction_step_budget(uint32_t budget) {
    if (!s_cx4 || !__atomic_load_n(&s_tx_active, __ATOMIC_ACQUIRE)) return;

    uint8_t st = cx4_read(s_cx4, 0x7f5eu);
    uint32_t used = 0;
    while ((st & 0xc0u) != 0u && used < budget) {
        s_virtual_master += TX_MASTER_CHUNK;
        cx4_sync(s_cx4, s_virtual_master);
        s_tx_chunks++;
        s_tx_progress_chunks++;
        used++;
        st = cx4_read(s_cx4, 0x7f5eu);
        __atomic_store_n(&s_status_proxy, st, __ATOMIC_RELEASE);
        if (cx4_locked(s_cx4)) {
            s_tx_locked++;
            break;
        }
    }

    if ((st & 0xc0u) != 0u && !cx4_locked(s_cx4)) {
        /* Do NOT abort a long run.  Real CPU writes may arrive while the HG51B
           is running; V8.2 keeps draining/applying them on later service calls. */
        if (s_tx_progress_chunks == TX_LONGRUN_CHUNKS) s_tx_timeout++;
        s_service_tx_budget_hits++;
        return;
    }

    s_tx_last_chunks = s_tx_progress_chunks;
    s_tx_last_status = st;
    mirror_refresh_all();
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&s_tx_active, 0u, __ATOMIC_RELEASE);

    if ((st & 0xc0u) == 0u) s_tx_done++;
}

static void clear_runtime_state(void) {
    memset((void *)s_shadow, 0, sizeof(s_shadow));
    s_virtual_master = 1;
    __atomic_store_n(&s_tx_active, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&s_pending_run, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&s_pending_busy, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&s_status_proxy, 0u, __ATOMIC_RELEASE);
    s_tx_jobs = s_tx_done = s_tx_timeout = s_tx_chunks = 0;
    s_tx_busy_reads = s_tx_early_arms = s_tx_locked = 0;
    s_tx_last_entry = 0xffu;
    s_tx_last_status = 0;
    s_tx_last_chunks = 0;
    s_tx_progress_chunks = 0;
    s_service_calls = s_service_yields = 0;
    s_service_write_budget_hits = s_service_tx_budget_hits = 0;
    s_trace_seq = 0;
    /* Flush any physical writes that belonged to the pre-reset machine state. */
    uint32_t h = __atomic_load_n(&s_write_head, __ATOMIC_ACQUIRE);
    __atomic_store_n(&s_write_tail, h, __ATOMIC_RELEASE);
    if (s_cx4) {
        cx4_reset(s_cx4);
        mirror_refresh_all();
    }
    s_state_resets++;
}

void cx4bus_init(void) {
    rom_slot_probe();
    s_armed = false;
    s_driving = false;
    s_reset_requested = false;

    if (s_rom_slot_valid) {
        s_cx4 = cx4_create(s_game_rom, s_game_rom_size, NULL, 0);
        if (s_cx4) cx4_synthesize_data_rom(s_cx4);
    }

    clear_runtime_state();
    s_armed = s_rom_slot_valid && s_cx4 && cx4_firmware_loaded(s_cx4);

    const uint pins[8] = {2,3,4,6,7,8,9,10};
    for (unsigned i=0;i<8;i++) {
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
    data_release();
    stat42_pio_init();
    s_stat42_mode = true;
}

void cx4bus_arm(bool enabled) {
    s_armed = enabled && s_rom_slot_valid && s_cx4;
    if (!s_armed) {
        pio_interrupt_clear(s_stat_pio, STAT42_PIO_IRQ);
        if (!s_stat42_mode) data_release();
    }
}
bool cx4bus_is_armed(void) { return s_armed; }
void cx4bus_reset_state(void) { s_reset_requested = true; }

static void __not_in_flash_func(core1_loop)(void) {
    s_core1_started = true;
    if (!s_stat42_mode) data_release();

    for (;;) {
        while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();

        bool stat_armed = false;
        uint16_t pre_off = 0;

        /* FINAL TEST: do NOT wait for /RD or /ROMSEL during PHI2-low.  Those
           strobes were the reason V8.3 never pre-drove.  We only require the
           full CX4 address to be stable twice and /WR high.  PIO then waits for
           the PHI2 rising edge and drives 0x42 in hardware. */
        if (s_stat42_mode && s_armed && s_stat42_pio_ready) {
            while (!(sio_hw->gpio_in & PHI2_MASK)) {
                uint32_t lo1 = sio_hw->gpio_in;
                uint32_t hi1 = sio_hw->gpio_hi_in;
                if ((lo1 & WR_MASK) == 0u || !raw_is_cx4(lo1, hi1)) continue;
                uint16_t o1 = raw_cx4_offset(lo1, hi1);
                if (!is_status_offset(o1)) continue;
                __asm volatile("nop; nop;" ::: "memory");
                uint32_t lo2 = sio_hw->gpio_in;
                uint32_t hi2 = sio_hw->gpio_hi_in;
                if (lo2 & PHI2_MASK) break;
                if ((lo2 & WR_MASK) == 0u || !raw_is_cx4(lo2, hi2)) continue;
                uint16_t o2 = raw_cx4_offset(lo2, hi2);
                if (o2 != o1 || !is_status_offset(o2)) continue;
                pre_off = o2;
                pio_interrupt_set(s_stat_pio, STAT42_PIO_IRQ);
                stat_armed = true;
                s_stat42_arms++;
                break;
            }
        }

        while (!(sio_hw->gpio_in & PHI2_MASK)) tight_loop_contents();
        __asm volatile("nop; nop;" ::: "memory");
        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool rd = (hi & RD_HI_MASK) == 0u;
        bool wr = (lo & WR_MASK) == 0u;

        if (rd) {
            const bool ctrl_ok =
                ((hi & ROMSEL_HI_MASK) != 0u) &&
                ((hi & WRAMSEL_HI_MASK) != 0u) &&
                ((lo & WR_MASK) != 0u) &&
                raw_is_cx4(lo, hi);

            if (ctrl_ok) {
                uint16_t off = raw_cx4_offset(lo, hi);
                if (s_stat42_mode && stat_armed && is_status_offset(off) && off == pre_off) {
                    /* PIO is already driving 0x42 from the exact rising edge. */
                    __asm volatile("nop; nop; nop; nop;" ::: "memory");
                    uint8_t observed = raw_data(sio_hw->gpio_in);
                    s_stat42_pin_last = observed;
                    if (observed == STAT42_VALUE) s_stat42_pin_ok++;
                    else s_stat42_pin_bad++;
                    s_stat42_forced_reads++;
                    s_stat42_saw_read = true;
                    s_cpu_reads++;
                    s_driven_reads++;
                    s_last_read_addr = off;
                    s_last_read_data = STAT42_VALUE;
                    trace_io('R', off, STAT42_VALUE);
                } else if (!s_stat42_mode) {
                    uint8_t v = bus_read(off);
                    if (s_armed) { data_drive(v); s_driven_reads++; }
                    s_cpu_reads++;
                    s_last_read_addr = off;
                    s_last_read_data = v;
                    trace_io('R', off, v);
                    s_read_late++;
                } else {
                    /* Diagnostic mode intentionally leaves non-status reads Hi-Z. */
                    s_cpu_reads++;
                    s_last_read_addr = off;
                    s_last_read_data = bus_read(off);
                }
            } else {
                if (stat_armed) s_stat42_false_arms++;
                uint32_t a = raw_address24(lo, hi);
                if (a == 0x00fffcu) s_reset_arm = 8;
                else if (a == 0x00fffdu && s_reset_arm) {
                    s_reset_vectors++; s_reset_requested = true; s_reset_arm = 0;
                } else if (s_reset_arm) --s_reset_arm;
            }

            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
            if (!s_stat42_mode && s_driving) data_release();
            /* PIO releases its own output at this same falling edge. */
            continue;
        }

        if (stat_armed) {
            /* Stable status address turned out not to be a qualified read. */
            s_stat42_false_arms++;
        }

        if (wr && raw_is_cx4(lo, hi)) {
            uint16_t off = raw_cx4_offset(lo, hi);
            const bool ctrl_ok_w =
                ((hi & ROMSEL_HI_MASK) != 0u) &&
                ((hi & WRAMSEL_HI_MASK) != 0u);

            __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;" ::: "memory");
            uint8_t v = raw_data(sio_hw->gpio_in);

            if (ctrl_ok_w) {
                bool queued = write_q_push(off, v);
                if (queued) {
                    if (is_busy_trigger(off)) {
                        __atomic_fetch_add(&s_pending_busy, 1u, __ATOMIC_ACQ_REL);
                        s_tx_early_arms++;
                    } else if (is_run_trigger(off)) {
                        __atomic_fetch_add(&s_pending_run, 1u, __ATOMIC_ACQ_REL);
                        s_tx_early_arms++;
                    }
                }
                if ((off == 0x7f4fu || off == 0x6f4fu) && s_stat42_mode && s_stat42_saw_read)
                    s_stat42_cmd_after++;
                s_cpu_writes++;
                s_last_write_addr = off;
                s_last_write_data = v;
                trace_io('W', off, v);
            }
            while (sio_hw->gpio_in & PHI2_MASK) tight_loop_contents();
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

static void apply_write(uint16_t off, uint8_t data) {
    if (!s_cx4) return;

    uint16_t c = canonical_off(off);

    /* Record whether a physical entry-PC write reached Core0 while the actual
       HG51B was idle or already running. This directly diagnoses lost/mutated
       $7F4F bytes without guessing from the picture. */
    if (off == 0x7f4fu || off == 0x6f4fu) {
        uint8_t pre = cx4_read(s_cx4, 0x7f5eu);
        s_q_last_core_cmd = data;
        if (data == 0x00u) s_cmd_core_00++;
        else if (data == 0xafu) s_cmd_core_af++;
        else s_cmd_core_other++;
        if (pre & 0x40u) s_cmd_core_running++;
        else s_cmd_core_idle++;
        s_tx_last_entry = data;
    }

    shadow_store(off, data);
    cx4_write(s_cx4, off, data);

    /* Any write may legally start cache fill, DMA, or program execution. */
    uint8_t st = cx4_read(s_cx4, 0x7f5eu);
    __atomic_store_n(&s_status_proxy, st, __ATOMIC_RELEASE);
    if (st & 0xc0u) (void)transaction_begin_if_needed();
    else if (c >= 0x6c00u) shadow_store(c, cx4_read(s_cx4, c));
}

void cx4bus_service(void) {
    s_service_calls++;

    if (s_reset_requested) {
        s_reset_requested = false;
        clear_runtime_state();
        return;
    }

    /* V8.2 LIVE INTERLEAVE:
       CPU-side writes do not stop merely because the HG51B is running.  The
       upstream core explicitly allows register/DRAM writes while RUNNING; only
       its own internal busy state decides what those writes do.  V8.1 froze the
       queue for the whole transaction, which overflowed and destroyed command
       input. */
    write_evt_t e;
    uint32_t processed = 0;
    while (processed < SERVICE_WRITE_BUDGET && write_q_pop(&e)) {
        apply_write(e.off, e.data);
        processed++;

        if (is_busy_trigger(e.off)) {
            uint32_t p = __atomic_load_n(&s_pending_busy, __ATOMIC_ACQUIRE);
            if (p) __atomic_fetch_sub(&s_pending_busy, 1u, __ATOMIC_ACQ_REL);
        } else if (is_run_trigger(e.off)) {
            uint32_t p = __atomic_load_n(&s_pending_run, __ATOMIC_ACQUIRE);
            if (p) __atomic_fetch_sub(&s_pending_run, 1u, __ATOMIC_ACQ_REL);
        }

        /* Interleave the DSP with dense CPU traffic instead of letting either
           side monopolize Core0. */
        if (__atomic_load_n(&s_tx_active, __ATOMIC_ACQUIRE) &&
            ((processed & 31u) == 0u)) {
            transaction_step_budget(1u);
        }
    }

    if (processed == SERVICE_WRITE_BUDGET && write_q_depth())
        s_service_write_budget_hits++;

    if (__atomic_load_n(&s_tx_active, __ATOMIC_ACQUIRE))
        transaction_step_budget(SERVICE_TX_CHUNK_BUDGET);

    s_service_yields++;
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
    if (!s_cx4) { printf("CX4RUNS core=0\n"); return; }
    Cx4RunEvent ring[16];
    uint32_t n = cx4_run_ring_copy(s_cx4, ring, 16u);
    printf("CX4RUNS total=%lu showing=%lu\n",
           (unsigned long)cx4_run_ring_count(s_cx4), (unsigned long)n);
    for (uint32_t i=0;i<n;i++)
        printf("  seq=%lu base=%06lX pb=%04X pc=%02X\n",
               (unsigned long)ring[i].seq, (unsigned long)ring[i].base,
               ring[i].pb, ring[i].pc);
}

void cx4bus_print_queue(void) {
    printf("CX4Q depth=%lu pending_run=%lu pending_busy=%lu pushed=%llu popped=%llu dropped=%llu peak=%lu predrive=%llu late=%llu retarget=%llu last_phys=%02X last_core=%02X\n",
           (unsigned long)write_q_depth(),
           (unsigned long)__atomic_load_n(&s_pending_run,__ATOMIC_ACQUIRE),
           (unsigned long)__atomic_load_n(&s_pending_busy,__ATOMIC_ACQUIRE),
           (unsigned long long)s_q_pushed,(unsigned long long)s_q_popped,
           (unsigned long long)s_q_dropped,(unsigned long)s_q_peak,
           (unsigned long long)s_read_predrive,(unsigned long long)s_read_late,
           (unsigned long long)s_read_retarget,
           s_q_last_phys_cmd,s_q_last_core_cmd);
    printf("CX4Q cmd_phys 00=%llu AF=%llu other=%llu | cmd_core 00=%llu AF=%llu other=%llu idle=%llu running=%llu\n",
           (unsigned long long)s_cmd_phys_00,(unsigned long long)s_cmd_phys_af,(unsigned long long)s_cmd_phys_other,
           (unsigned long long)s_cmd_core_00,(unsigned long long)s_cmd_core_af,(unsigned long long)s_cmd_core_other,
           (unsigned long long)s_cmd_core_idle,(unsigned long long)s_cmd_core_running);
}

static bool self_run_until_idle(Cx4 *c, uint64_t *master, uint32_t *chunks_out) {
    uint32_t chunks = 0;
    for (; chunks < SELF_MAX_CHUNKS; ++chunks) {
        uint8_t st = cx4_read(c, 0x7f5eu);
        if ((st & 0xc0u) == 0u) break;
        *master += TX_MASTER_CHUNK;
        cx4_sync(c, *master);
        if (cx4_locked(c)) break;
    }
    if (chunks_out) *chunks_out = chunks;
    return (cx4_read(c,0x7f5eu) & 0xc0u) == 0u;
    cx4bus_print_status42();
}

void cx4bus_selfcheck(void) {
    if (s_armed) { printf("CX4SELF REFUSED armed=1; use CX4DISARM first\n"); return; }
    if (!s_rom_slot_valid) { printf("CX4SELF FAIL rom_slot=0\n"); return; }
    Cx4 *t = cx4_create(s_game_rom, s_game_rom_size, NULL, 0);
    if (!t) { printf("CX4SELF FAIL alloc\n"); return; }
    cx4_synthesize_data_rom(t);

    uint64_t master = 1;
    uint32_t chunks = 0, total_chunks = 0;
    /* Same entry setup used by the earlier independent validator. */
    cx4_write(t,0x7f49u,0x00u);
    cx4_write(t,0x7f4au,0x80u);
    cx4_write(t,0x7f4bu,0x02u);
    cx4_write(t,0x7f4du,0x0eu);
    cx4_write(t,0x7f4eu,0x00u);
    cx4_write(t,0x7f48u,0x01u);
    (void)self_run_until_idle(t,&master,&chunks); total_chunks += chunks;
    cx4_write(t,0x7f4fu,0x5cu);
    (void)self_run_until_idle(t,&master,&chunks); total_chunks += chunks;
    cx4_write(t,0x7f4fu,0x89u);
    bool ok = self_run_until_idle(t,&master,&chunks); total_chunks += chunks;

    uint32_t lo=0,hi=0;
    cx4_rdrom_index_range(t,&lo,&hi);
    printf("CX4SELF ok=%u runs=%lu insns=%llu rdrom=%lu distinct=%lu range=%lu-%lu firmware=%d locked=%d status=%02X chunks=%lu\n",
        ok?1u:0u, (unsigned long)cx4_run_ring_count(t),
        (unsigned long long)cx4_instructions_executed(t),
        (unsigned long)cx4_rdrom_hits(t),(unsigned long)cx4_rdrom_distinct(t),
        (unsigned long)lo,(unsigned long)hi,cx4_firmware_loaded(t),cx4_locked(t),
        cx4_read(t,0x7f5eu),(unsigned long)total_chunks);
    cx4_destroy(t);
}

void cx4bus_print_status(void) {
    uint32_t lo=0,hi=0;
    uint32_t runs=0, rdrom=0, distinct=0;
    uint64_t insns=0;
    int firmware=0, locked=0;
    if (s_cx4) {
        cx4_rdrom_index_range(s_cx4,&lo,&hi);
        runs=cx4_run_ring_count(s_cx4);
        insns=cx4_instructions_executed(s_cx4);
        rdrom=cx4_rdrom_hits(s_cx4);
        distinct=cx4_rdrom_distinct(s_cx4);
        firmware=cx4_firmware_loaded(s_cx4);
        locked=cx4_locked(s_cx4);
    }
    printf("CX4STAT mode=PIO_STATUS42_FINAL_V9 armed=%u drive=%s core1=%u core=%u rom_slot=%u rom_bytes=%lu rom_crc=%08lX "
           "q_depth=%lu q_prun=%lu q_pbusy=%lu q_push=%llu q_pop=%llu q_drop=%llu q_peak=%lu "
           "svc=%llu yields=%llu wb_hit=%llu txb_hit=%llu tx_prog=%lu "
           "tx_active=%u tx_jobs=%llu tx_done=%llu tx_to=%llu tx_chunks=%llu tx_busy_reads=%llu tx_early=%llu tx_locked=%llu tx_last=%02X/%02X/%lu "
           "lle_runs=%lu lle_insns=%llu lle_rdrom=%lu lle_distinct=%lu lle_range=%lu-%lu firmware=%d locked=%d master=%llu status=%02X "
           "cpu_r=%llu cpu_w=%llu driven=%llu predrive=%llu late=%llu retarget=%llu vec_resets=%llu state_resets=%llu last_r=%04lX:%02X last_w=%04lX:%02X "
           "cmd_phys=%02X cmd_core=%02X cmd00=%llu/%llu cmdAF=%llu/%llu cmdOther=%llu/%llu cmdIdle=%llu cmdRunning=%llu\n",
           s_armed?1u:0u,s_driving?"ON":"OFF",s_core1_started?1u:0u,s_cx4?1u:0u,
           s_rom_slot_valid?1u:0u,(unsigned long)s_game_rom_size,(unsigned long)s_game_rom_crc32,
           (unsigned long)write_q_depth(),
           (unsigned long)__atomic_load_n(&s_pending_run,__ATOMIC_ACQUIRE),
           (unsigned long)__atomic_load_n(&s_pending_busy,__ATOMIC_ACQUIRE),
           (unsigned long long)s_q_pushed,(unsigned long long)s_q_popped,
           (unsigned long long)s_q_dropped,(unsigned long)s_q_peak,
           (unsigned long long)s_service_calls,(unsigned long long)s_service_yields,
           (unsigned long long)s_service_write_budget_hits,(unsigned long long)s_service_tx_budget_hits,
           (unsigned long)s_tx_progress_chunks,
           __atomic_load_n(&s_tx_active,__ATOMIC_ACQUIRE)?1u:0u,
           (unsigned long long)s_tx_jobs,(unsigned long long)s_tx_done,(unsigned long long)s_tx_timeout,
           (unsigned long long)s_tx_chunks,(unsigned long long)s_tx_busy_reads,
           (unsigned long long)s_tx_early_arms,(unsigned long long)s_tx_locked,
           s_tx_last_entry,s_tx_last_status,(unsigned long)s_tx_last_chunks,
           (unsigned long)runs,(unsigned long long)insns,(unsigned long)rdrom,(unsigned long)distinct,
           (unsigned long)lo,(unsigned long)hi,firmware,locked,(unsigned long long)s_virtual_master,
           __atomic_load_n(&s_status_proxy,__ATOMIC_ACQUIRE),
           (unsigned long long)s_cpu_reads,(unsigned long long)s_cpu_writes,(unsigned long long)s_driven_reads,
           (unsigned long long)s_read_predrive,(unsigned long long)s_read_late,(unsigned long long)s_read_retarget,
           (unsigned long long)s_reset_vectors,(unsigned long long)s_state_resets,
           (unsigned long)s_last_read_addr,s_last_read_data,(unsigned long)s_last_write_addr,s_last_write_data,
           s_q_last_phys_cmd,s_q_last_core_cmd,
           (unsigned long long)s_cmd_phys_00,(unsigned long long)s_cmd_core_00,
           (unsigned long long)s_cmd_phys_af,(unsigned long long)s_cmd_core_af,
           (unsigned long long)s_cmd_phys_other,(unsigned long long)s_cmd_core_other,
           (unsigned long long)s_cmd_core_idle,(unsigned long long)s_cmd_core_running);
}
