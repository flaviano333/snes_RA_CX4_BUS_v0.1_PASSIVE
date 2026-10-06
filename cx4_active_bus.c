#include "cx4_active_bus.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "bus_drive.pio.h"

/*
 * BUS VALIDATOR v1.0.7 SPLIT-PIO-DIRECT
 *
 * This is intentionally NOT a CX4 emulator.  It is the final electrical
 * readback validator.  The companion ROM writes/reads $00:6000-$6BFF with
 * DATA == A0..A7.  Two PIO0 state machines therefore derive the return byte
 * directly from the address pins, avoiding all Core1 latency.
 *
 * PIO2 (configured in main.c) qualifies the /RD event in hardware:
 *   /ROMSEL == HIGH   (Flash/ROM is NOT selected)
 *   /WRAMSEL == HIGH  (SNES WRAM is NOT selected)
 * Only then are PIO0 IRQ2/IRQ3 published to the split data responders.
 */

#define PIN_PHI2 0u
#define PIN_RD   35u

static uint8_t s_ram[0x0c00];

static volatile bool s_arm_request=false;
static volatile bool s_arm_pending=false;
static volatile bool s_arm_wait_pass=false;
static volatile bool s_reset_requested=false;
static volatile bool s_magic=false;
static volatile bool s_drive_enabled=false;
static volatile bool s_drive_ready=false;

static volatile uint8_t s_magic_mask=0, s_phase=0, s_seq=0;
static volatile uint64_t s_magic_hits=0, s_magic_bad=0;

static volatile uint64_t s_pio_w=0;
static volatile uint64_t s_passes=0, s_write_passes=0, s_reg_passes=0, s_read_passes=0;
static volatile uint32_t s_write_seen=0, s_write_bad_data=0, s_write_bad_addr=0, s_write_count_bad=0;
static volatile uint64_t s_write_bad_data_total=0, s_write_bad_addr_total=0, s_write_missing_total=0, s_write_extra_total=0;
static volatile uint32_t s_first_bad_exp=0xffffffffu, s_first_bad_got=0xffffffffu;
static volatile uint8_t s_first_bad_data=0, s_first_bad_expected_data=0;
static volatile uint32_t s_reg_index=0, s_reg_bad=0;
static volatile uint16_t s_read_errors=0xffffu, s_last_read_errors=0xffffu;
static volatile uint64_t s_read_pass_good=0, s_read_pass_bad=0;

static volatile uint64_t s_active_passes=0, s_active_good=0, s_active_bad=0;
static volatile uint16_t s_active_last_err=0xffffu;
static volatile uint32_t s_active_good_bytes=0;
static volatile uint64_t s_drive_windows=0, s_force_off=0;
static volatile uint16_t s_expected_off=0x6000u;
static volatile uint16_t s_last_off=0;
static volatile uint8_t s_last_data=0;

static const uint16_t k_reg_off[6]={0x7f49,0x7f4a,0x7f4b,0x7f4d,0x7f4e,0x7f4f};
static const uint8_t  k_reg_val[6]={0x00,0x80,0x02,0x0e,0x00,0x5c};

static PIO s_drive_pio = pio0;
static const uint s_drive_sm_lo = 2u;
static const uint s_drive_sm_hi = 3u;
static uint s_drive_off = 0u;

static inline uint8_t expected_data(uint16_t off){
    return (uint8_t)off; /* companion ROM: DATA == A0..A7 */
}

static void drive_hw_off(void){
    if(s_drive_ready){
        pio_sm_set_enabled(s_drive_pio, s_drive_sm_lo, false);
        pio_sm_set_enabled(s_drive_pio, s_drive_sm_hi, false);

        /* Force every connected DATA GPIO back to input regardless of where a
           state machine was interrupted. GP5 is intentionally untouched. */
        pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm_lo, 2, 3, false);
        pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm_hi, 6, 5, false);

        pio_interrupt_clear(s_drive_pio, 2);
        pio_interrupt_clear(s_drive_pio, 3);
        pio_sm_restart(s_drive_pio, s_drive_sm_lo);
        pio_sm_restart(s_drive_pio, s_drive_sm_hi);

        /* The shared program uses X as a permanent SM role selector. */
        pio_sm_exec(s_drive_pio, s_drive_sm_lo, pio_encode_set(pio_x, 0));
        pio_sm_exec(s_drive_pio, s_drive_sm_hi, pio_encode_set(pio_x, 1));
    }
    if(s_drive_enabled) s_force_off++;
    s_drive_enabled=false;
}

static void drive_hw_on(void){
    if(!s_drive_ready) return;
    drive_hw_off();

    /* Never enable the responder in the middle of an already-active bus cycle.
       BUSARM is scheduled a full pass early, so waiting for this clean boundary
       costs nothing but removes the only stale-IRQ enable race. */
    while(gpio_get(PIN_PHI2) || !gpio_get(PIN_RD)) tight_loop_contents();

    pio_interrupt_clear(s_drive_pio, 2);
    pio_interrupt_clear(s_drive_pio, 3);
    pio_sm_set_enabled(s_drive_pio, s_drive_sm_lo, true);
    pio_sm_set_enabled(s_drive_pio, s_drive_sm_hi, true);
    s_drive_enabled=true;
    s_drive_windows++;
}

static void clear_state(void){
    memset(s_ram,0,sizeof(s_ram));
    s_magic=false; s_magic_mask=0; s_phase=0; s_seq=0;
    s_magic_hits=s_magic_bad=0;
    s_pio_w=0;
    s_passes=s_write_passes=s_reg_passes=s_read_passes=0;
    s_write_seen=s_write_bad_data=s_write_bad_addr=s_write_count_bad=0;
    s_write_bad_data_total=s_write_bad_addr_total=s_write_missing_total=s_write_extra_total=0;
    s_first_bad_exp=s_first_bad_got=0xffffffffu; s_first_bad_data=s_first_bad_expected_data=0;
    s_reg_index=s_reg_bad=0;
    s_read_errors=s_last_read_errors=0xffffu;
    s_read_pass_good=s_read_pass_bad=0;
    s_active_passes=s_active_good=s_active_bad=0;
    s_active_last_err=0xffffu; s_active_good_bytes=0;
    s_drive_windows=s_force_off=0;
    s_expected_off=0x6000u; s_last_off=0; s_last_data=0;
    s_arm_wait_pass=false;
}

void cx4bus_init(void){
    clear_state();
    s_arm_request=false; s_arm_pending=false; s_reset_requested=false;

    /* capture_low.pio is already resident in PIO0. One shared 18-instruction
       program services BOTH remaining state machines, keeping total PIO0
       instruction use below 32 words. */
    if(!pio_can_add_program(s_drive_pio, &snes_bus_drive_split_program)){
        s_drive_ready=false;
        printf("PIO drive: ERROR no instruction memory; active readback disabled\n");
        return;
    }

    s_drive_off=pio_add_program(s_drive_pio, &snes_bus_drive_split_program);

    pio_sm_config c_lo=snes_bus_drive_split_program_get_default_config(s_drive_off);
    sm_config_set_in_pins(&c_lo, 11);      /* A0 at GP11 */
    sm_config_set_out_pins(&c_lo, 2, 3);   /* D0..D2 */
    sm_config_set_set_pins(&c_lo, 2, 3);
    sm_config_set_jmp_pin(&c_lo, PIN_PHI2);
    sm_config_set_in_shift(&c_lo, false, false, 32); /* shift left: sampled bits stay in LSBs */
    sm_config_set_out_shift(&c_lo, true, false, 32);

    pio_sm_config c_hi=snes_bus_drive_split_program_get_default_config(s_drive_off);
    sm_config_set_in_pins(&c_hi, 14);      /* A3 at GP14 */
    sm_config_set_out_pins(&c_hi, 6, 5);   /* D3..D7 */
    sm_config_set_set_pins(&c_hi, 6, 5);
    sm_config_set_jmp_pin(&c_hi, PIN_PHI2);
    sm_config_set_in_shift(&c_hi, false, false, 32);
    sm_config_set_out_shift(&c_hi, true, false, 32);

    int rc_lo=pio_sm_init(s_drive_pio, s_drive_sm_lo, s_drive_off, &c_lo);
    int rc_hi=pio_sm_init(s_drive_pio, s_drive_sm_hi, s_drive_off, &c_hi);
    s_drive_ready=(rc_lo==0 && rc_hi==0);

    pio_sm_set_enabled(s_drive_pio, s_drive_sm_lo, false);
    pio_sm_set_enabled(s_drive_pio, s_drive_sm_hi, false);
    pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm_lo, 2, 3, false);
    pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm_hi, 6, 5, false);
    pio_sm_exec(s_drive_pio, s_drive_sm_lo, pio_encode_set(pio_x, 0));
    pio_sm_exec(s_drive_pio, s_drive_sm_hi, pio_encode_set(pio_x, 1));
    pio_interrupt_clear(s_drive_pio, 2);
    pio_interrupt_clear(s_drive_pio, 3);

    printf("PIO drive: split-direct sm=%u/%u off=%u init=%d/%d ready=%u\n",
           s_drive_sm_lo,s_drive_sm_hi,s_drive_off,rc_lo,rc_hi,s_drive_ready?1u:0u);
}

/* No latency-critical CPU loop exists in v1.0.7. Kept for main.c compatibility. */
void cx4bus_launch_core1(void){ }

void cx4bus_arm(bool e){
    if(e){
        s_arm_pending=true;
        s_arm_request=false;
        s_arm_wait_pass=false;
        drive_hw_off();
    }else{
        s_arm_pending=false;
        s_arm_request=false;
        s_arm_wait_pass=false;
        drive_hw_off();
    }
}

bool cx4bus_is_armed(void){ return s_arm_request || s_arm_pending; }
void cx4bus_reset_state(void){ s_reset_requested=true; }

static void finish_write_phase(void){
    s_write_passes++;
    if(s_write_seen!=0x0c00u){
        s_write_count_bad++;
        if(s_write_seen<0x0c00u) s_write_missing_total += (uint64_t)(0x0c00u-s_write_seen);
        else s_write_extra_total += (uint64_t)(s_write_seen-0x0c00u);
    }
}

static void mailbox(uint16_t off,uint8_t data){
    if(off>=0x7ff0u && off<=0x7ff3u){
        const uint8_t m[4]={'B','U','S','7'};
        uint8_t idx=(uint8_t)(off-0x7ff0u);
        if(data==m[idx]){
            s_magic_mask |= (uint8_t)(1u<<idx);
            s_magic_hits++;
            if(s_magic_mask==0x0fu) s_magic=true;
        }else{
            s_magic_bad++;
            s_magic_mask &= (uint8_t)~(1u<<idx);
        }
        return;
    }
    if(!s_magic)return;
    if(off==0x7ff4u){ s_seq=data; return; }
    if(off==0x7ff6u){ s_read_errors=(uint16_t)((s_read_errors&0xff00u)|data); return; }
    if(off==0x7ff7u){ s_read_errors=(uint16_t)((s_read_errors&0x00ffu)|((uint16_t)data<<8)); return; }
    if(off!=0x7ff5u)return;

    if(data==0x10u){
        s_phase=0x10u;
        s_write_seen=s_write_bad_data=s_write_bad_addr=0;
        s_expected_off=0x6000u;
        s_read_errors=0xffffu;
    }else if(data==0x20u){
        if(s_phase==0x10u) finish_write_phase();
        s_phase=0x20u; s_reg_index=0; s_reg_bad=0;
    }else if(data==0x30u){
        s_phase=0x30u;
        if(s_reg_index==6u && s_reg_bad==0u) s_reg_passes++;
    }else if(data==0x40u){
        s_phase=0x40u;
        s_last_read_errors=s_read_errors;
        if(s_last_read_errors==0u) s_read_pass_good++; else s_read_pass_bad++;
        s_read_passes++; s_passes++;

        /* Close the ONE active pass first. */
        if(s_arm_request && s_arm_wait_pass){
            s_active_last_err=s_read_errors;
            s_active_good_bytes=(s_read_errors<=0x0c00u) ? (0x0c00u-s_read_errors) : 0u;
            s_active_passes++;
            if(s_read_errors==0u) s_active_good++; else s_active_bad++;
            s_arm_request=false;
            s_arm_wait_pass=false;
            drive_hw_off();
        }

        /* BUSARM is deliberately activated at the END of the preceding pass.
           The entire next WRITE+REG phase gives the delayed PIO+DMA mailbox
           scanner ample time before the first target read. */
        if(s_arm_pending){
            s_arm_pending=false;
            s_arm_request=true;
            s_arm_wait_pass=true;
            s_active_last_err=0xffffu;
            s_active_good_bytes=0;
            drive_hw_on();
        }
    }
}

void cx4bus_pio_write(uint32_t address,uint8_t data){
    if((address>>16)!=0x00u)return;
    uint16_t off=(uint16_t)address;
    if(off<0x6000u || off>0x7fffu)return;
    s_pio_w++; s_last_off=off; s_last_data=data;

    if(off>=0x7ff0u){ mailbox(off,data); return; }

    if(off>=0x6000u && off<=0x6bffu){
        s_ram[off-0x6000u]=data;
        if(s_magic && s_phase==0x10u){
            if(off!=s_expected_off){
                s_write_bad_addr++; s_write_bad_addr_total++;
                if(s_first_bad_exp==0xffffffffu){ s_first_bad_exp=s_expected_off; s_first_bad_got=off; }
            }
            uint8_t ev=expected_data(off);
            if(data!=ev){
                s_write_bad_data++; s_write_bad_data_total++;
                if(s_first_bad_exp==0xffffffffu){
                    s_first_bad_exp=off; s_first_bad_got=off;
                    s_first_bad_data=data; s_first_bad_expected_data=ev;
                }
            }
            s_write_seen++;
            s_expected_off=(uint16_t)(off+1u);
        }
        return;
    }

    if(s_magic && s_phase==0x20u && off>=0x7f40u && off<=0x7fafu){
        if(s_reg_index<6u){
            if(off!=k_reg_off[s_reg_index] || data!=k_reg_val[s_reg_index]) s_reg_bad++;
            s_reg_index++;
        }else s_reg_bad++;
    }
}

/* Read DMA is intentionally disabled in this validator. The authoritative
   result is the SNES ROM's own 3072-byte compare, reported through $7FF6/7. */
void cx4bus_pio_read(uint32_t address,uint8_t data){ (void)address; (void)data; }

void cx4bus_service(void){
    if(s_reset_requested){
        s_reset_requested=false;
        s_arm_request=false; s_arm_pending=false; s_arm_wait_pass=false;
        drive_hw_off();
        clear_state();
    }
}

void cx4bus_print_trace(void){
    printf("BUSTRACE last=%04X:%02X phase=%02X magic=%u mask=%X hits=%llu bad=%llu drive=%s\n",
           s_last_off,s_last_data,s_phase,s_magic?1u:0u,s_magic_mask,
           (unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad,
           s_drive_enabled?"ON":"OFF");
}

void cx4bus_print_runs(void){
    printf("BUSREG seq=%u index=%lu bad=%lu expected=7F49:00 7F4A:80 7F4B:02 7F4D:0E 7F4E:00 7F4F:5C\n",
           s_seq,(unsigned long)s_reg_index,(unsigned long)s_reg_bad);
}

void cx4bus_selfcheck(void){
    printf("BUSTEST v1.0.7 requires bundled test_rom/CX4_BUS_TEST.sfc (BUS7, DATA=A0..A7)\n");
}

void cx4bus_print_status(void){
    bool ready=(s_active_passes>0u && s_active_last_err==0u && s_drive_ready);
    uint irq2=pio_interrupt_get(s_drive_pio, 2)?1u:0u;
    uint irq3=pio_interrupt_get(s_drive_pio, 3)?1u:0u;

    printf("BUSSTAT mode=BUS_VALIDATOR_V1.0.7_SPLIT_PIO_DIRECT verdict=%s "
           "arm=%u pending=%u drive=%s ready=%u windows=%llu forced_off=%llu irq=%u/%u "
           "magic=%u seq=%u phase=%02X passes=%llu "
           "write_passes=%llu write_seen=%lu bad_data=%lu bad_addr=%lu count_bad=%lu "
           "bad_data_total=%llu bad_addr_total=%llu missing_total=%llu extra_total=%llu "
           "reg_passes=%llu reg_index=%lu reg_bad=%lu "
           "read_passes=%llu cur_err=%u last_err=%u read_good=%llu read_bad=%llu "
           "active_passes=%llu active_last_err=%u active_good_bytes=%lu/3072 active_good=%llu active_bad=%llu "
           "guard=ROMSEL_HIGH+WRAMSEL_HIGH_PIO data=A0-A7_DIRECT magic_mask=%X magic_hits=%llu magic_bad=%llu pio_w=%llu "
           "first_bad=%04lX/%04lX:%02X/%02X last=%04X:%02X\n",
           ready?"READY_FOR_GAME":"NOT_YET",
           s_arm_request?1u:0u,s_arm_pending?1u:0u,s_drive_enabled?"ON":"OFF",s_drive_ready?1u:0u,
           (unsigned long long)s_drive_windows,(unsigned long long)s_force_off,irq2,irq3,
           s_magic?1u:0u,s_seq,s_phase,(unsigned long long)s_passes,
           (unsigned long long)s_write_passes,(unsigned long)s_write_seen,
           (unsigned long)s_write_bad_data,(unsigned long)s_write_bad_addr,(unsigned long)s_write_count_bad,
           (unsigned long long)s_write_bad_data_total,(unsigned long long)s_write_bad_addr_total,
           (unsigned long long)s_write_missing_total,(unsigned long long)s_write_extra_total,
           (unsigned long long)s_reg_passes,(unsigned long)s_reg_index,(unsigned long)s_reg_bad,
           (unsigned long long)s_read_passes,(unsigned)s_read_errors,(unsigned)s_last_read_errors,
           (unsigned long long)s_read_pass_good,(unsigned long long)s_read_pass_bad,
           (unsigned long long)s_active_passes,(unsigned)s_active_last_err,(unsigned long)s_active_good_bytes,
           (unsigned long long)s_active_good,(unsigned long long)s_active_bad,
           s_magic_mask,(unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad,(unsigned long long)s_pio_w,
           (unsigned long)s_first_bad_exp,(unsigned long)s_first_bad_got,s_first_bad_data,s_first_bad_expected_data,
           s_last_off,s_last_data);
}
