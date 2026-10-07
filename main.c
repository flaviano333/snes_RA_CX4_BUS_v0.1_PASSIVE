#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "cx4_active_bus.h"

#define PIN_WR         1u
#define PIN_RD         35u
#define PIN_ROMSEL     38u
#define PIN_WRAMSEL    39u
#define GAMEPLAY_CLOCK_KHZ 150000u

static char cmd_buf[96];
static size_t cmd_len = 0;

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
    printf("LASTBUS clock_khz=%u capture=SIO_SINGLE_SNAPSHOT\n", GAMEPLAY_CLOCK_KHZ);
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
    else if (!strcmp(line,"CX4Q")) cx4bus_print_queue();
    else if (!strcmp(line,"CX4SELF")) cx4bus_selfcheck();
    else if (!strcmp(line,"CX4ARM")) { cx4bus_arm(true); printf("OK CX4 armed\n"); }
    else if (!strcmp(line,"CX4DISARM")) { cx4bus_arm(false); printf("OK CX4 disarmed; DATA Hi-Z\n"); }
    else if (!strcmp(line,"CX4RESET")) { cx4bus_reset_state(); printf("OK virtual CX4 reset requested\n"); }
    else if (!strcmp(line,"HELP")) {
        printf("Commands: INFO/CX4STAT CX4TRACE CX4RUNS CX4Q CX4SELF CX4ARM CX4DISARM CX4RESET\n");
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
    set_sys_clock_khz(GAMEPLAY_CLOCK_KHZ, true);
    stdio_init_all();
    sleep_ms(350);

    printf("\n=== SNES RP2350B CX4 SINGLE-SNAPSHOT LLE V8.2 LIVE INTERLEAVE ===\n");
    printf("clock=%u kHz | no PIO/DMA write pairing; Core1 captures one coherent A-bus write event per PHI2 cycle\n", GAMEPLAY_CLOCK_KHZ);
    printf("GP41/GP42 unused. Original A-bus only. GP25=A13 must be electrically sound.\n");
    printf("Core1: physical read responder + SPSC write producer; Core0: HG51B LLE consumer.\n\n");
    fflush(stdout);

    for (uint pin=0; pin<=40u; ++pin) {
        configure_input(pin, pin==PIN_WR || pin==PIN_RD || pin==PIN_ROMSEL || pin==PIN_WRAMSEL);
    }

    cx4bus_init();
    cx4bus_launch_core1();
    printf("READY SINGLE-SNAPSHOT LLE V8.2 LIVE INTERLEAVE. Power/reset SNES with Mega Man X2 selected.\n");
    fflush(stdout);

    for (;;) {
        cx4bus_service();
        poll_serial();
        tight_loop_contents();
    }
}
