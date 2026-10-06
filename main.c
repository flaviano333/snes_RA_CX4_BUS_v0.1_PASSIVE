#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "capture_low.pio.h"
#include "capture_high.pio.h"
#include "master_clock.pio.h"
#include "cx4_active_bus.h"

#define PIN_PHI2       0u
#define PIN_WR         1u
#define PIN_DATA_BASE  2u
#define PIN_HIGH_BASE  21u
#define PIN_RD         35u
#define PIN_ROMSEL     38u
#define PIN_WRAMSEL    39u
#define PIN_MASTER_CLK  41u
#define PIN_RESET       42u

#define SAMPLE_COUNT 1024u
#define GAMEPLAY_CLOCK_KHZ 150000u

static uint32_t low_samples[2][SAMPLE_COUNT];
static uint32_t high_samples[2][SAMPLE_COUNT];
static uint32_t scan_pos[2] = {0,0};

static int dma_lo = -1, dma_hi = -1, dma_clk = -1;
static volatile uint32_t master_dma_sink = 0;
static uint64_t master_blocks_base = 0;
#define MASTER_BLOCK_CYCLES 1024ull
#define MASTER_DMA_COUNT 0x0fffffffu
static uint active_buf = 0;
static uint64_t write_pairs_seen = 0;
static uint64_t write_pairs_qualified = 0;
static uint64_t write_pairs_rejected_ctrl = 0;
static uint64_t dma_rearms = 0;

static char cmd_buf[96];
static size_t cmd_len = 0;

static inline uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
}

static uint64_t master_clock_ticks(void) {
    if (dma_clk < 0) return 0;
    uint32_t rem = dma_remaining((uint)dma_clk);
    uint64_t done = (uint64_t)(MASTER_DMA_COUNT - rem);
    return (master_blocks_base + done) * MASTER_BLOCK_CYCLES;
}

static void master_clock_rearm_if_needed(PIO pio_clk, uint sm_clk) {
    if (dma_clk < 0) return;
    if (dma_remaining((uint)dma_clk) != 0u) return;
    master_blocks_base += MASTER_DMA_COUNT;
    dma_channel_set_read_addr((uint)dma_clk, &pio_clk->rxf[sm_clk], false);
    dma_channel_set_write_addr((uint)dma_clk, (void *)&master_dma_sink, false);
    dma_channel_set_trans_count((uint)dma_clk, MASTER_DMA_COUNT, true);
}

static inline uint32_t unpack_low18(uint32_t raw) {
    return (raw >> 14) & 0x3ffffu;
}

static inline uint32_t unpack_high20(uint32_t raw) {
    return (raw >> 12) & 0xfffffu;
}

static inline uint8_t reconstruct_data(uint32_t low18) {
    return (uint8_t)((low18 & 0x07u) | ((low18 >> 1) & 0xf8u));
}

static inline uint32_t sampled_gpio_bit(uint32_t low18, uint32_t high20, uint gpio) {
    if (gpio >= 11u && gpio <= 19u) return (low18 >> (gpio - 2u)) & 1u;
    if (gpio >= 21u && gpio <= 40u) return (high20 >> (gpio - 21u)) & 1u;
    return 0u;
}

static inline uint32_t reconstruct_address(uint32_t low18, uint32_t high20) {
    static const uint8_t address_gpio[24] = {
        11,12,13,14,15,16,17,18,
        19,21,40,23,24,25,27,28,
        29,30,31,32,33,34,36,37
    };
    uint32_t a = 0;
    for (uint bit=0; bit<24u; ++bit)
        a |= sampled_gpio_bit(low18, high20, address_gpio[bit]) << bit;
    return a & 0xffffffu;
}

static inline bool cx4_controls_open(uint32_t high20) {
    /* High sample starts at GP21: GP38=/ROMSEL bit17, GP39=/WRAMSEL bit18.
       Analyzer result on this exact prototype:
         real ROM => /ROMSEL LOW
         Cx4 $6000-$7FFF => /ROMSEL HIGH
       /WRAMSEL must also remain HIGH so WRAM cycles can never enter Cx4 state. */
    const bool romsel_high = ((high20 >> (PIN_ROMSEL - PIN_HIGH_BASE)) & 1u) != 0u;
    const bool wramsel_high = ((high20 >> (PIN_WRAMSEL - PIN_HIGH_BASE)) & 1u) != 0u;
    return romsel_high && wramsel_high;
}

static inline uint32_t captured_pair_count(void) {
    uint32_t rl = dma_remaining((uint)dma_lo);
    uint32_t rh = dma_remaining((uint)dma_hi);
    uint32_t cl = (rl <= SAMPLE_COUNT) ? (SAMPLE_COUNT - rl) : 0u;
    uint32_t ch = (rh <= SAMPLE_COUNT) ? (SAMPLE_COUNT - rh) : 0u;
    return cl < ch ? cl : ch;
}

static void feed_write_prefix(uint buf) {
    uint32_t count = captured_pair_count();
    if (count > SAMPLE_COUNT) count = SAMPLE_COUNT;
    uint32_t pos = scan_pos[buf];
    if (count < pos) pos = 0;
    __asm volatile("dmb sy" ::: "memory");

    for (uint32_t i=pos; i<count; ++i) {
        uint32_t lo = unpack_low18(low_samples[buf][i]);
        uint32_t hi = unpack_high20(high_samples[buf][i]);
        write_pairs_seen++;
        if (!cx4_controls_open(hi)) {
            write_pairs_rejected_ctrl++;
            continue;
        }
        uint32_t address = reconstruct_address(lo, hi);
        uint8_t data = reconstruct_data(lo);
        cx4bus_pio_write(address, data);
        write_pairs_qualified++;
    }
    scan_pos[buf] = count;
}

static void configure_input(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
    if (pull_up) gpio_pull_up(pin);
}

static void strtoupper_inplace(char *s) {
    while (*s) { *s = (char)toupper((unsigned char)*s); ++s; }
}

static void command_info(void) {
    const uint64_t mt = master_clock_ticks();
    printf("LASTBUS clock_khz=%u write_pairs=%llu qualified=%llu rejected_ctrl=%llu rearms=%llu active_buf=%u scan=%lu/%u "
           "sysclk_master=%llu reset=%s\n",
           GAMEPLAY_CLOCK_KHZ,
           (unsigned long long)write_pairs_seen,
           (unsigned long long)write_pairs_qualified,
           (unsigned long long)write_pairs_rejected_ctrl,
           (unsigned long long)dma_rearms,
           active_buf, (unsigned long)scan_pos[active_buf], SAMPLE_COUNT,
           (unsigned long long)mt,
           gpio_get(PIN_RESET) ? "HIGH" : "LOW");
    cx4bus_print_status();
}

static void execute_command(char *line) {
    while (*line==' ' || *line=='\t') ++line;
    char *end = line + strlen(line);
    while (end>line && (end[-1]==' ' || end[-1]=='\t')) *--end='\0';
    strtoupper_inplace(line);

    if (!strcmp(line,"CX4STAT") || !strcmp(line,"INFO")) command_info();
    else if (!strcmp(line,"CX4TRACE")) cx4bus_print_trace();
    else if (!strcmp(line,"CX4RUNS")) cx4bus_print_runs();
    else if (!strcmp(line,"CX4SELF")) cx4bus_selfcheck();
    else if (!strcmp(line,"CX4ARM")) { cx4bus_arm(true); printf("OK CX4 armed\n"); }
    else if (!strcmp(line,"CX4DISARM")) { cx4bus_arm(false); printf("OK CX4 disarmed; DATA Hi-Z\n"); }
    else if (!strcmp(line,"CX4RESET")) { cx4bus_reset_state(); printf("OK virtual CX4 reset requested\n"); }
    else if (!strcmp(line,"HELP")) {
        printf("Commands: INFO/CX4STAT CX4TRACE CX4RUNS CX4SELF CX4ARM CX4DISARM CX4RESET\n");
    } else if (*line) printf("ERR unknown command\n");
    fflush(stdout);
}

static void poll_serial(void) {
    int c;
    while ((c=getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c=='\r' || c=='\n') {
            if (cmd_len) { cmd_buf[cmd_len]='\0'; execute_command(cmd_buf); cmd_len=0; }
        } else if (c==8 || c==127) {
            if (cmd_len) --cmd_len;
        } else if (c>=32 && c<=126 && cmd_len+1<sizeof(cmd_buf)) {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}

int main(void) {
    /* This is intentionally a gameplay-only build. The extra clock headroom is
       dedicated to the Core1 physical read responder. */
    set_sys_clock_khz(GAMEPLAY_CLOCK_KHZ, true);
    stdio_init_all();
    sleep_ms(350);

    printf("\n=== SNES RP2350B CX4 SYSCLK+RESET HYBRID V6 ===\n");
    printf("clock=%u kHz | proven HLE boot path + exact LLE fallback\n", GAMEPLAY_CLOCK_KHZ);
    printf("existing pinout preserved; NEW: SYSTEM CLK=GP41, /RESET=GP42 (BOTH LEVEL-SHIFTED TO 3.3V)\n");
    printf("write authority: PIO+DMA, qualified by /ROMSEL HIGH + /WRAMSEL HIGH\n");
    printf("read responder: preserved v0.5 PHI2 timing + /ROMSEL HIGH + /WRAMSEL HIGH + /WR HIGH\n");
    printf("known commands use HLE; unhandled commands fall back to exact HG51B clocked from real SNES SYSTEM CLK; /RESET is authoritative.\n\n");
    fflush(stdout);

    for (uint pin=0; pin<=40u; ++pin) {
        configure_input(pin, pin==PIN_WR || pin==PIN_RD || pin==PIN_ROMSEL || pin==PIN_WRAMSEL);
    }
    /* GP41/GP42 are NOT 5V-tolerant on RP2350B. The hardware connection must
       level-shift the SNES signals to 3.3 V before they reach these GPIOs. */
    configure_input(PIN_RESET, false);

    PIO pio_lo = pio0;
    PIO pio_hi = pio1;
    PIO pio_clk = pio2;
    const uint sm_lo = 0, sm_hi = 0, sm_clk = 0;
    int base_lo = pio_set_gpio_base(pio_lo, 0);
    int base_hi = pio_set_gpio_base(pio_hi, 16);
    int base_clk = pio_set_gpio_base(pio_clk, 16);

    for (uint pin=0; pin<=19u; ++pin) pio_gpio_init(pio_lo, pin);
    for (uint pin=21; pin<=40u; ++pin) pio_gpio_init(pio_hi, pin);
    pio_sm_set_consecutive_pindirs(pio_lo, sm_lo, 0, 20, false);
    pio_sm_set_consecutive_pindirs(pio_hi, sm_hi, 21, 20, false);

    uint off_lo = pio_add_program(pio_lo, &snes_capture_low_program);
    pio_sm_config cfg_lo = snes_capture_low_program_get_default_config(off_lo);
    sm_config_set_in_pins(&cfg_lo, PIN_DATA_BASE);
    sm_config_set_jmp_pin(&cfg_lo, PIN_WR);
    sm_config_set_in_shift(&cfg_lo, true, false, 32);
    sm_config_set_fifo_join(&cfg_lo, PIO_FIFO_JOIN_RX);
    int init_lo = pio_sm_init(pio_lo, sm_lo, off_lo, &cfg_lo);

    uint off_hi = pio_add_program(pio_hi, &snes_capture_high_program);
    pio_sm_config cfg_hi = snes_capture_high_program_get_default_config(off_hi);
    sm_config_set_in_pins(&cfg_hi, PIN_HIGH_BASE);
    sm_config_set_in_shift(&cfg_hi, true, false, 32);
    sm_config_set_fifo_join(&cfg_hi, PIO_FIFO_JOIN_RX);
    int init_hi = pio_sm_init(pio_hi, sm_hi, off_hi, &cfg_hi);

    /* PIO2 counts the physical 21.477 MHz SNES master clock in blocks of 1024.
       The RX FIFO is drained by DMA, so no CPU interrupt is needed per edge. */
    gpio_disable_pulls(PIN_MASTER_CLK);
    pio_gpio_init(pio_clk, PIN_MASTER_CLK);
    pio_sm_set_consecutive_pindirs(pio_clk, sm_clk, PIN_MASTER_CLK, 1, false);
    uint off_clk = pio_add_program(pio_clk, &snes_master_clock_counter_program);
    pio_sm_config cfg_clk = snes_master_clock_counter_program_get_default_config(off_clk);
    sm_config_set_in_pins(&cfg_clk, PIN_MASTER_CLK);
    sm_config_set_fifo_join(&cfg_clk, PIO_FIFO_JOIN_RX);
    int init_clk = pio_sm_init(pio_clk, sm_clk, off_clk, &cfg_clk);

    printf("PIO write capture: base=%d/%d init=%d/%d | SYSCLK base=%d init=%d GP41\n",
           base_lo,base_hi,init_lo,init_hi,base_clk,init_clk);
    if (base_lo || base_hi || base_clk || init_lo || init_hi || init_clk) {
        printf("FATAL PIO init failed; DATA remains input.\n");
        while (true) { poll_serial(); sleep_ms(10); }
    }

    /* Configure CX4 state before capture starts. It also hands only DATA output
       ownership to SIO; PIO can still sample the physical input paths. */
    cx4bus_init();

    dma_lo = dma_claim_unused_channel(true);
    dma_hi = dma_claim_unused_channel(true);
    dma_clk = dma_claim_unused_channel(true);

    dma_channel_config dc_lo = dma_channel_get_default_config((uint)dma_lo);
    channel_config_set_transfer_data_size(&dc_lo, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_lo, false);
    channel_config_set_write_increment(&dc_lo, true);
    channel_config_set_dreq(&dc_lo, pio_get_dreq(pio_lo, sm_lo, false));

    dma_channel_config dc_hi = dma_channel_get_default_config((uint)dma_hi);
    channel_config_set_transfer_data_size(&dc_hi, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_hi, false);
    channel_config_set_write_increment(&dc_hi, true);
    channel_config_set_dreq(&dc_hi, pio_get_dreq(pio_hi, sm_hi, false));

    dma_channel_config dc_clk = dma_channel_get_default_config((uint)dma_clk);
    channel_config_set_transfer_data_size(&dc_clk, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_clk, false);
    channel_config_set_write_increment(&dc_clk, false);
    channel_config_set_dreq(&dc_clk, pio_get_dreq(pio_clk, sm_clk, false));

    pio_sm_set_enabled(pio_lo,sm_lo,false);
    pio_sm_set_enabled(pio_hi,sm_hi,false);
    pio_sm_set_enabled(pio_clk,sm_clk,false);
    pio_sm_restart(pio_lo,sm_lo);
    pio_sm_restart(pio_hi,sm_hi);
    pio_sm_restart(pio_clk,sm_clk);
    pio_sm_clear_fifos(pio_lo,sm_lo);
    pio_sm_clear_fifos(pio_hi,sm_hi);
    pio_sm_clear_fifos(pio_clk,sm_clk);
    pio_interrupt_clear(pio_hi,0);

    dma_channel_configure((uint)dma_lo,&dc_lo,low_samples[0],&pio_lo->rxf[sm_lo],SAMPLE_COUNT,false);
    dma_channel_configure((uint)dma_hi,&dc_hi,high_samples[0],&pio_hi->rxf[sm_hi],SAMPLE_COUNT,false);
    dma_channel_configure((uint)dma_clk,&dc_clk,(void *)&master_dma_sink,&pio_clk->rxf[sm_clk],MASTER_DMA_COUNT,false);
    dma_start_channel_mask((1u<<(uint)dma_lo)|(1u<<(uint)dma_hi)|(1u<<(uint)dma_clk));
    pio_sm_set_enabled(pio_clk,sm_clk,true);
    pio_sm_set_enabled(pio_hi,sm_hi,true);
    pio_sm_set_enabled(pio_lo,sm_lo,true);

    cx4bus_launch_core1();
    printf("READY SYSCLK+RESET HYBRID V6. Power/reset the SNES with Mega Man X2 selected.\n");
    fflush(stdout);

    for (;;) {
        /* Drain new authoritative write pairs continuously. This normally makes
           register/RAM writes visible long before the 1024-sample DMA block fills. */
        feed_write_prefix(active_buf);
        master_clock_rearm_if_needed(pio_clk, sm_clk);
        cx4bus_service(master_clock_ticks());

        if (dma_remaining((uint)dma_lo)==0u && dma_remaining((uint)dma_hi)==0u) {
            uint done=active_buf, next=done^1u;
            feed_write_prefix(done);

            dma_channel_set_write_addr((uint)dma_lo,low_samples[next],false);
            dma_channel_set_trans_count((uint)dma_lo,SAMPLE_COUNT,false);
            dma_channel_set_write_addr((uint)dma_hi,high_samples[next],false);
            dma_channel_set_trans_count((uint)dma_hi,SAMPLE_COUNT,false);
            dma_start_channel_mask((1u<<(uint)dma_lo)|(1u<<(uint)dma_hi));
            active_buf=next;
            scan_pos[next]=0;
            dma_rearms++;
        }

        master_clock_rearm_if_needed(pio_clk, sm_clk);
        cx4bus_service(master_clock_ticks());
        poll_serial();
        tight_loop_contents();
    }
}
