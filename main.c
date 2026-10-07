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
#include "cx4_active_bus.h"

#define PIN_PHI2       0u
#define PIN_WR         1u
#define PIN_DATA_BASE  2u
#define PIN_HIGH_BASE  21u
#define PIN_RD         35u
#define PIN_ROMSEL     38u
#define PIN_WRAMSEL    39u

#define SAMPLE_COUNT 1024u
#define GAMEPLAY_CLOCK_KHZ 150000u

static uint32_t low_samples[2][SAMPLE_COUNT];
static uint32_t high_samples[2][SAMPLE_COUNT];
static uint32_t scan_pos[2] = {0,0};

static int dma_lo = -1, dma_hi = -1;
static uint active_buf = 0;
static uint64_t write_pairs_seen = 0;
static uint64_t write_pairs_qualified = 0;
static uint64_t write_pairs_rejected_ctrl = 0;
static uint64_t dma_rearms = 0;

/* Address forensics for control-qualified writes.  The Cx4 coarse window has
 * fixed A13=1, A14=1, A15=0, A22=0.  If one sampled line is wrong we can
 * identify it statistically and optionally force that bit in software. */
static uint64_t diag_raw_valid = 0;
static uint64_t diag_pattern[16] = {0};
static uint64_t diag_onefix[4] = {0}; /* A13=1,A14=1,A15=0,A22=0 */
static uint32_t diag_examples[16] = {0};
static uint8_t diag_example_data[16] = {0};
static uint32_t diag_example_n = 0;

static char cmd_buf[96];
static size_t cmd_len = 0;

static inline uint32_t dma_remaining(uint channel) {
    return dma_channel_hw_addr(channel)->transfer_count & 0x0fffffffu;
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

static inline bool addr24_is_cx4(uint32_t a) {
    return ((a & (1u<<22)) == 0u) && ((a & (1u<<15)) == 0u) &&
           ((a & (1u<<14)) != 0u) && ((a & (1u<<13)) != 0u);
}

static inline uint32_t force_addr_bit(uint32_t a, uint bit, bool one) {
    uint32_t m = 1u << bit;
    return one ? (a | m) : (a & ~m);
}

static void diag_address(uint32_t a, uint8_t data) {
    uint pattern = 0u;
    if (a & (1u<<13)) pattern |= 1u;
    if (a & (1u<<14)) pattern |= 2u;
    if (a & (1u<<15)) pattern |= 4u;
    if (a & (1u<<22)) pattern |= 8u;
    diag_pattern[pattern]++;
    if (addr24_is_cx4(a)) diag_raw_valid++;
    if (addr24_is_cx4(force_addr_bit(a,13u,true)))  diag_onefix[0]++;
    if (addr24_is_cx4(force_addr_bit(a,14u,true)))  diag_onefix[1]++;
    if (addr24_is_cx4(force_addr_bit(a,15u,false))) diag_onefix[2]++;
    if (addr24_is_cx4(force_addr_bit(a,22u,false))) diag_onefix[3]++;
    if (diag_example_n < 16u) {
        diag_examples[diag_example_n] = a;
        diag_example_data[diag_example_n] = data;
        diag_example_n++;
    }
}

static void command_busmap(void) {
    uint32_t fm=0,fv=0;
    cx4bus_get_addr_fix(&fm,&fv);
    printf("BUSMAP qualified=%llu raw_valid=%llu fix=%06lX/%06lX onefix A13=1:%llu A14=1:%llu A15=0:%llu A22=0:%llu\n",
           (unsigned long long)write_pairs_qualified,
           (unsigned long long)diag_raw_valid,
           (unsigned long)fm,(unsigned long)fv,
           (unsigned long long)diag_onefix[0],(unsigned long long)diag_onefix[1],
           (unsigned long long)diag_onefix[2],(unsigned long long)diag_onefix[3]);
    printf("BUSMAP pattern[A22 A15 A14 A13] 0000..1111:");
    for (unsigned i=0;i<16;i++) printf(" %X=%llu",i,(unsigned long long)diag_pattern[i]);
    printf("\nBUSMAP examples:");
    for (uint32_t i=0;i<diag_example_n;i++) printf(" %06lX:%02X",(unsigned long)diag_examples[i],diag_example_data[i]);
    printf("\n");
}

static void command_busclr(void) {
    diag_raw_valid=0;
    memset(diag_pattern,0,sizeof(diag_pattern));
    memset(diag_onefix,0,sizeof(diag_onefix));
    diag_example_n=0;
    printf("OK BUSMAP counters cleared\n");
}

static void command_addrfix(const char *arg) {
    uint32_t mask=0,value=0;
    if (!strcmp(arg,"OFF")) {
        cx4bus_set_addr_fix(0,0);
        printf("OK ADDRFIX OFF; reset/power-cycle SNES before judging gameplay\n");
        return;
    }
    if (!strcmp(arg,"A13") || !strcmp(arg,"13")) { mask=1u<<13; value=1u<<13; }
    else if (!strcmp(arg,"A14") || !strcmp(arg,"14")) { mask=1u<<14; value=1u<<14; }
    else if (!strcmp(arg,"A15") || !strcmp(arg,"15")) { mask=1u<<15; value=0; }
    else if (!strcmp(arg,"A22") || !strcmp(arg,"22")) { mask=1u<<22; value=0; }
    else if (!strcmp(arg,"AUTO")) {
        unsigned best=0, second=1;
        for (unsigned i=0;i<4;i++) if (diag_onefix[i]>diag_onefix[best]) best=i;
        second = (best==0)?1:0;
        for (unsigned i=0;i<4;i++) if (i!=best && diag_onefix[i]>diag_onefix[second]) second=i;
        uint64_t b=diag_onefix[best], s2=diag_onefix[second];
        if (b < 100u || b < (s2*2u + 1u) || b <= diag_raw_valid) {
            printf("ERR ADDRFIX AUTO low confidence best=%llu second=%llu raw=%llu; use BUSMAP\n",
                   (unsigned long long)b,(unsigned long long)s2,(unsigned long long)diag_raw_valid);
            return;
        }
        const uint bits[4]={13,14,15,22};
        const bool ones[4]={true,true,false,false};
        mask=1u<<bits[best]; value=ones[best]?mask:0u;
        printf("AUTO selected A%u=%u score=%llu second=%llu\n",bits[best],ones[best]?1u:0u,
               (unsigned long long)b,(unsigned long long)s2);
    } else {
        printf("ERR ADDRFIX use OFF/A13/A14/A15/A22/AUTO\n");
        return;
    }
    cx4bus_set_addr_fix(mask,value);
    printf("OK ADDRFIX mask=%06lX value=%06lX; now reset/power-cycle SNES\n",
           (unsigned long)mask,(unsigned long)value);
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
        diag_address(address, data);
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
    printf("LASTBUS clock_khz=%u write_pairs=%llu qualified=%llu rejected_ctrl=%llu rearms=%llu active_buf=%u scan=%lu/%u\n",
           GAMEPLAY_CLOCK_KHZ,
           (unsigned long long)write_pairs_seen,
           (unsigned long long)write_pairs_qualified,
           (unsigned long long)write_pairs_rejected_ctrl,
           (unsigned long long)dma_rearms,
           active_buf, (unsigned long)scan_pos[active_buf], SAMPLE_COUNT);
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
    else if (!strcmp(line,"BUSMAP")) command_busmap();
    else if (!strcmp(line,"BUSCLR")) command_busclr();
    else if (!strncmp(line,"ADDRFIX ",8)) command_addrfix(line+8);
    else if (!strcmp(line,"HELP")) {
        printf("Commands: INFO/CX4STAT CX4TRACE CX4RUNS CX4SELF CX4ARM CX4DISARM CX4RESET BUSMAP BUSCLR ADDRFIX OFF/A13/A14/A15/A22/AUTO\n");
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

    printf("\n=== SNES RP2350B CX4 LLE TRANSACTIONAL V7.1 BUSMAP ===\n");
    printf("clock=%u kHz | V4 physical bus + instruction-level HG51B transactional engine\n", GAMEPLAY_CLOCK_KHZ);
    printf("pinout unchanged: PHI2=GP0 /WR=GP1 D0-2=GP2-4 GP5=SKIP D3-7=GP6-10 /RD=GP35 /ROMSEL=GP38 /WRAMSEL=GP39 A10=GP40\n");
    printf("write authority: PIO+DMA, qualified by /ROMSEL HIGH + /WRAMSEL HIGH\n");
    printf("read responder: preserved v0.5 PHI2 timing + /ROMSEL HIGH + /WRAMSEL HIGH + /WR HIGH\n");
    printf("CX4 response auto-arms; HLE removed; HG51B LLE runs transactions on a synthetic master timeline.\n\n");
    fflush(stdout);

    for (uint pin=0; pin<=40u; ++pin) {
        configure_input(pin, pin==PIN_WR || pin==PIN_RD || pin==PIN_ROMSEL || pin==PIN_WRAMSEL);
    }

    PIO pio_lo = pio0;
    PIO pio_hi = pio1;
    const uint sm_lo = 0, sm_hi = 0;
    int base_lo = pio_set_gpio_base(pio_lo, 0);
    int base_hi = pio_set_gpio_base(pio_hi, 16);

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

    printf("PIO write capture: base=%d/%d init=%d/%d\n",base_lo,base_hi,init_lo,init_hi);
    if (base_lo || base_hi || init_lo || init_hi) {
        printf("FATAL PIO init failed; DATA remains input.\n");
        while (true) { poll_serial(); sleep_ms(10); }
    }

    /* Configure CX4 state before capture starts. It also hands only DATA output
       ownership to SIO; PIO can still sample the physical input paths. */
    cx4bus_init();

    dma_lo = dma_claim_unused_channel(true);
    dma_hi = dma_claim_unused_channel(true);

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

    pio_sm_set_enabled(pio_lo,sm_lo,false);
    pio_sm_set_enabled(pio_hi,sm_hi,false);
    pio_sm_restart(pio_lo,sm_lo);
    pio_sm_restart(pio_hi,sm_hi);
    pio_sm_clear_fifos(pio_lo,sm_lo);
    pio_sm_clear_fifos(pio_hi,sm_hi);
    pio_interrupt_clear(pio_hi,0);

    dma_channel_configure((uint)dma_lo,&dc_lo,low_samples[0],&pio_lo->rxf[sm_lo],SAMPLE_COUNT,false);
    dma_channel_configure((uint)dma_hi,&dc_hi,high_samples[0],&pio_hi->rxf[sm_hi],SAMPLE_COUNT,false);
    dma_start_channel_mask((1u<<(uint)dma_lo)|(1u<<(uint)dma_hi));
    pio_sm_set_enabled(pio_hi,sm_hi,true);
    pio_sm_set_enabled(pio_lo,sm_lo,true);

    cx4bus_launch_core1();
    printf("READY LLE TRANSACTIONAL V7.1 BUSMAP. GP41/GP42 are unused; power/reset SNES with Mega Man X2 selected.\n");
    fflush(stdout);

    for (;;) {
        /* Drain new authoritative write pairs continuously. This normally makes
           register/RAM writes visible long before the 1024-sample DMA block fills. */
        feed_write_prefix(active_buf);
        cx4bus_service();

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

        cx4bus_service();
        poll_serial();
        tight_loop_contents();
    }
}
