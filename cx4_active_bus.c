#include "cx4_active_bus.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "bus_drive.pio.h"

/*
 * v1.0.6 PRELOAD + RD + ROMSEL HARDWARE-GATE ONE-PASS RESPONDER
 *
 * This deliberately does NOT emulate CX4. It validates the electrical/data
 * path with a deterministic test ROM. PIO+DMA is authoritative for writes.
 * Active readback is restricted to bank $00, $6000-$6BFF, only after the
 * test ROM's BUS6 magic has been observed, and only while /ROMSEL is HIGH.
 * PIO2 enforces /ROMSEL HIGH in hardware at the /RD edge before it can publish
 * the IRQ that lets PIO0 enable D0-D7.
 */
#define PIN_PHI2 0u
#define PIN_WR   1u
#define PIN_RD   35u
#define PIN_ROMSEL 38u
#define PIN_WRAMSEL 39u
#define DATA_MASK ((1u << 2) | (1u << 3) | (1u << 4) | (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 10))
#define PHI2_MASK (1u << PIN_PHI2)
#define WR_MASK   (1u << PIN_WR)
#define RD_HI_MASK (1u << (PIN_RD - 32u))
#define ROMSEL_HI_MASK (1u << (PIN_ROMSEL - 32u))
#define WRAMSEL_HI_MASK (1u << (PIN_WRAMSEL - 32u))

static uint8_t s_ram[0x0c00];
static volatile bool s_arm_request=false, s_arm_pending=false, s_driving=false, s_core1_started=false;
static volatile bool s_reset_requested=false, s_magic=false;
static volatile uint8_t s_magic_step=0, s_magic_mask=0, s_phase=0, s_seq=0;
static volatile uint64_t s_magic_hits=0, s_magic_bad=0;

static volatile uint64_t s_pio_w=0, s_cpu_r=0, s_driven=0;
static volatile uint64_t s_rd_edges=0, s_rd_safe=0, s_rd_bank_miss=0, s_rd_range_miss=0;
static volatile uint64_t s_phi_cycles=0, s_phi_target=0, s_phi_rd=0, s_phi_target_rd=0, s_phi_no_rd=0, s_phi_late=0;
static volatile uint64_t s_passes=0, s_write_passes=0, s_reg_passes=0, s_read_passes=0;
static volatile uint32_t s_write_seen=0, s_write_bad_data=0, s_write_bad_addr=0, s_write_count_bad=0;
static volatile uint64_t s_write_bad_data_total=0, s_write_bad_addr_total=0, s_write_missing_total=0, s_write_extra_total=0;
static volatile uint32_t s_first_bad_exp=0xffffffffu, s_first_bad_got=0xffffffffu;
static volatile uint8_t s_first_bad_data=0, s_first_bad_expected_data=0;
static volatile uint32_t s_reg_index=0, s_reg_bad=0;
static volatile uint16_t s_read_errors=0xffffu, s_last_read_errors=0xffffu;
static volatile uint64_t s_read_pass_good=0, s_read_pass_bad=0;
static volatile uint64_t s_fixed_hold=0, s_hold_abort=0;
static volatile uint64_t s_sample_target=0, s_sample_match=0, s_sample_bad=0;
static volatile uint64_t s_addr_double=0, s_addr_unstable=0, s_too_late=0;
static volatile uint64_t s_echo_target=0, s_echo_match=0, s_echo_bad=0;
static volatile uint64_t s_auto_disarm=0, s_budget_cut=0;
static volatile uint64_t s_active_passes=0, s_active_target=0, s_active_echo_bad=0;
static volatile uint16_t s_active_last_err=0xffffu;
static volatile uint64_t s_pio_queued=0, s_pio_fifo_full=0, s_pio_stale=0;
static volatile uint64_t s_preload_seen=0, s_preload_cancel=0, s_preload_addr_change=0;
static volatile uint64_t s_preload_no_rd=0, s_preload_miss=0, s_preload_unstable=0;
static volatile uint64_t s_post_ok=0, s_post_bad=0, s_post_addr_mismatch=0;
static volatile bool s_preload_live=false;
static volatile uint64_t s_gate_romsel=0, s_gate_wramsel=0, s_gate_ctrl=0;
static volatile uint64_t s_romsel_pre_high=0, s_romsel_pre_low=0;
static volatile uint64_t s_romsel_rd_high=0, s_romsel_rd_low=0;
static volatile uint64_t s_drive_during_rom=0;
static volatile uint32_t s_drive_budget=0;
static PIO s_drive_pio = pio0;
static const uint s_drive_sm = 2u;
static bool s_drive_ready = false;
static volatile bool s_arm_wait_pass=false;
static volatile uint16_t s_sample_last_off=0; static volatile uint8_t s_sample_last_got=0, s_sample_last_exp=0;
static volatile uint16_t s_expected_off=0x6000u;
static volatile uint16_t s_last_off=0; static volatile uint8_t s_last_data=0;

static const uint16_t k_reg_off[6]={0x7f49,0x7f4a,0x7f4b,0x7f4d,0x7f4e,0x7f4f};
static const uint8_t  k_reg_val[6]={0x00,0x80,0x02,0x0e,0x00,0x5c};

static inline uint8_t expected_data(uint16_t off){
    uint16_t rel=(uint16_t)(off-0x6000u);
    return (uint8_t)((rel & 0xffu) ^ ((rel>>8)&0xffu) ^ 0x5au);
}
static inline uint8_t raw_data(uint32_t lo){ return (uint8_t)(((lo>>2)&7u)|((lo>>3)&0xf8u)); }
/* PIO0 drives GP2..GP10 as a 9-pin window; GP5 is an unconnected dummy bit. */
static inline uint32_t packed9_data(uint8_t v){ return ((uint32_t)v & 7u) | (((uint32_t)v & 0xf8u) << 1); }
static inline uint32_t drive_word(uint8_t v){ return packed9_data(v) | (0x1ffu << 9); }
static void data_release(void){
    if(s_drive_ready){
        pio_sm_set_enabled(s_drive_pio, s_drive_sm, false);
        pio_sm_clear_fifos(s_drive_pio, s_drive_sm);
        pio_interrupt_clear(s_drive_pio, 2);   /* discard any stale /RD gate */
        pio_sm_restart(s_drive_pio, s_drive_sm);
        pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm, 2, 9, false);
        pio_sm_set_enabled(s_drive_pio, s_drive_sm, true);
    }
    s_preload_live=false;
    s_driving=false;
}
static inline bool queue_drive(uint8_t v){
    if(!s_drive_ready || pio_sm_is_tx_fifo_full(s_drive_pio, s_drive_sm)){
        s_pio_fifo_full++;
        return false;
    }

    /* IRQ2 is cleared once at the beginning of each PHI2 cycle, before any
       current-cycle /RD can occur.  Do not clear it here: doing so after the
       physical /RD edge would erase the very hardware gate we need. */
    pio_sm_put(s_drive_pio, s_drive_sm, drive_word(v));
    s_pio_queued++;
    s_preload_live=true;
    return true;
}

static inline uint32_t raw_address24(uint32_t lo,uint32_t hi){
    uint32_t a=0;
    a|=((lo>>11)&0x1ffu);
    a|=((lo>>21)&1u)<<9;
    a|=((hi>>8)&1u)<<10;
    a|=((lo>>23)&0x7u)<<11;
    a|=((lo>>27)&0x1fu)<<14;
    a|=(hi&0x7u)<<19;
    a|=((hi>>4)&1u)<<22;
    a|=((hi>>5)&1u)<<23;
    return a&0xffffffu;
}
static inline bool safe_test_read(uint32_t a){
    return s_arm_request && s_magic && ((a>>16)==0x00u) && ((uint16_t)a>=0x6000u) && ((uint16_t)a<=0x6bffu);
}

static void clear_state(void){
    memset(s_ram,0,sizeof(s_ram));
    s_magic=false; s_magic_step=0; s_magic_mask=0; s_phase=0; s_seq=0;
    s_magic_hits=s_magic_bad=0;
    s_rd_edges=s_rd_safe=s_rd_bank_miss=s_rd_range_miss=0;
    s_phi_cycles=s_phi_target=s_phi_rd=s_phi_target_rd=s_phi_no_rd=s_phi_late=0;
    s_passes=s_write_passes=s_reg_passes=s_read_passes=0;
    s_write_seen=s_write_bad_data=s_write_bad_addr=s_write_count_bad=0;
    s_write_bad_data_total=s_write_bad_addr_total=s_write_missing_total=s_write_extra_total=0;
    s_first_bad_exp=s_first_bad_got=0xffffffffu; s_first_bad_data=s_first_bad_expected_data=0;
    s_reg_index=s_reg_bad=0; s_read_errors=0xffffu; s_last_read_errors=0xffffu; s_read_pass_good=s_read_pass_bad=0;
    s_fixed_hold=s_hold_abort=0; s_sample_target=s_sample_match=s_sample_bad=0;
    s_addr_double=s_addr_unstable=s_too_late=0;
    s_echo_target=s_echo_match=s_echo_bad=0; s_auto_disarm=s_budget_cut=0;
    s_active_passes=s_active_target=s_active_echo_bad=0; s_active_last_err=0xffffu;
    s_pio_queued=s_pio_fifo_full=s_pio_stale=0;
    s_preload_seen=s_preload_cancel=s_preload_addr_change=0;
    s_preload_no_rd=s_preload_miss=s_preload_unstable=0;
    s_post_ok=s_post_bad=s_post_addr_mismatch=0; s_preload_live=false;
    s_gate_romsel=s_gate_wramsel=s_gate_ctrl=0;
    s_romsel_pre_high=s_romsel_pre_low=0;
    s_romsel_rd_high=s_romsel_rd_low=0;
    s_drive_during_rom=0;
    s_drive_budget=0; s_arm_wait_pass=false;
    s_sample_last_off=0; s_sample_last_got=s_sample_last_exp=0; s_expected_off=0x6000u;
    s_last_off=0; s_last_data=0;
}

void cx4bus_init(void){
    clear_state(); s_arm_request=false; s_arm_pending=false; s_driving=false; s_reset_requested=false;

    /* PIO0 SM2 owns only the DATA output path. The existing PIO0 capture SMs
       can still observe the pad inputs while SM2 controls output value/OE. */
    for(unsigned pin=2; pin<=10; ++pin){
        gpio_disable_pulls(pin);
        pio_gpio_init(s_drive_pio, pin);
    }
    if(pio_can_add_program(s_drive_pio, &snes_bus_drive_program)){
        uint off = pio_add_program(s_drive_pio, &snes_bus_drive_program);
        pio_sm_config c = snes_bus_drive_program_get_default_config(off);
        sm_config_set_out_pins(&c, 2, 9);
        sm_config_set_jmp_pin(&c, PIN_PHI2);
        sm_config_set_out_shift(&c, true, false, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        int rc = pio_sm_init(s_drive_pio, s_drive_sm, off, &c);
        pio_sm_set_consecutive_pindirs(s_drive_pio, s_drive_sm, 2, 9, false);
        pio_sm_clear_fifos(s_drive_pio, s_drive_sm);
        pio_sm_set_enabled(s_drive_pio, s_drive_sm, rc == 0);
        s_drive_ready = (rc == 0);
        printf("PIO drive: sm=%u off=%u init=%d ready=%u\n", s_drive_sm, off, rc, s_drive_ready?1u:0u);
    }else{
        s_drive_ready=false;
        printf("PIO drive: ERROR no instruction memory; active readback disabled\n");
    }
    data_release();
}
void cx4bus_arm(bool e){
    if(e){
        /* Do NOT begin driving in the middle of a ROM pass. BUSARM only
           requests one test pass. The mailbox phase $30 marker atomically
           activates the responder at the beginning of the NEXT read phase. */
        s_arm_pending=true;
        s_arm_request=false;
        s_drive_budget=0;
        s_arm_wait_pass=true;
        data_release();
    }else{
        s_arm_pending=false;
        s_arm_request=false;
        s_drive_budget=0;
        s_arm_wait_pass=false;
        data_release();
    }
}
bool cx4bus_is_armed(void){ return s_arm_request || s_arm_pending; }
void cx4bus_reset_state(void){ s_reset_requested=true; }

static void __not_in_flash_func(core1_loop)(void){
    /*
     * v1.0.6: PRELOAD + RD + ROMSEL-HARDWARE-GATE.
     *
     * v1.0.4 proved that the authoritative address must be sampled in the
     * actual /RD window.  Its remaining failure was latency: Core1 only queued
     * the byte AFTER /RD, leaving too little setup time on D0-D7.
     *
     * v1.0.6 uses a two-stage qualification:
     *   1) PRE-RD candidate: while PHI2 is high and /RD is still high, watch
     *      for a stable $00:6000-$6BFF address with /WR and /WRAMSEL inactive.
     *      Queue the expected byte into PIO0 SM2, but keep DATA pins inputs.
     *   2) HARDWARE GATE: PIO2 sees physical /RD and checks physical /ROMSEL.
     *      It raises PIO0 IRQ2 ONLY when /ROMSEL is HIGH. SM2 is already waiting
     *      with the byte in OSR, so it can place data and assert OE deterministically.
     *      Genuine ROM reads (/ROMSEL LOW) never receive the drive IRQ.
     *
     * After /RD, Core1 repeats the exact v1.0.4 double address sample and
     * control checks.  That post-RD sample remains authoritative for stats and
     * safety.  If it disagrees with the preloaded candidate, DATA is released
     * immediately and the mismatch is counted.
     */
    s_core1_started=true;
    data_release();
    bool prev_phi = (sio_hw->gpio_in & PHI2_MASK) != 0u;

    for(;;){
        bool phi;
        do {
            phi = (sio_hw->gpio_in & PHI2_MASK) != 0u;
            if(!phi) prev_phi=false;
        } while(!phi || prev_phi);
        prev_phi=true;
        s_phi_cycles++;

        if(!(s_arm_request && s_magic)){
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }

        /* Remove an IRQ2 left behind by an unrelated read from an earlier
           cycle.  This happens before current-cycle predecode, avoiding the
           clear-vs-/RD race that would exist inside queue_drive(). */
        if(s_drive_ready) pio_interrupt_clear(s_drive_pio, 2);

        bool preloaded=false;
        bool abandon_cycle=false;
        uint32_t pre_addr=0;
        uint8_t pre_value=0;

        /* Find a target address BEFORE /RD.  Unlike v1.0.3, we do not assume
           the address at PHI2 rising belongs to the later read.  We keep
           watching the live bus until either a stable target appears or /RD
           itself arrives. */
        while((sio_hw->gpio_in & PHI2_MASK) != 0u){
            uint32_t hi_now=sio_hw->gpio_hi_in;
            if((hi_now & RD_HI_MASK)==0u) break;

            uint32_t lo1=sio_hw->gpio_in;
            uint32_t hi1=sio_hw->gpio_hi_in;
            if((lo1 & PHI2_MASK)==0u) break;

            /* Writes, WRAM cycles and genuine ROM cycles are never eligible. */
            if((lo1 & WR_MASK)==0u || (hi1 & WRAMSEL_HI_MASK)==0u){
                tight_loop_contents();
                continue;
            }
            if((hi1 & ROMSEL_HI_MASK)==0u){
                s_romsel_pre_low++;
                tight_loop_contents();
                continue;
            }

            uint32_t a1=raw_address24(lo1,hi1);
            if(!safe_test_read(a1)){
                tight_loop_contents();
                continue;
            }

            __asm volatile("nop\n nop\n");
            if((sio_hw->gpio_in & PHI2_MASK)==0u) break;
            if((sio_hw->gpio_hi_in & RD_HI_MASK)==0u) break;

            uint32_t lo2=sio_hw->gpio_in;
            uint32_t hi2=sio_hw->gpio_hi_in;
            uint32_t a2=raw_address24(lo2,hi2);
            if(a1!=a2){
                s_preload_unstable++;
                continue;
            }
            if((lo2 & WR_MASK)==0u || (hi2 & WRAMSEL_HI_MASK)==0u) continue;
            if((hi2 & ROMSEL_HI_MASK)==0u){
                s_romsel_pre_low++;
                continue;
            }
            s_romsel_pre_high++;

            if(s_drive_budget==0u){
                s_budget_cut++;
                s_arm_request=false;
                data_release();
                abandon_cycle=true;
                break;
            }

            /* Final pre-queue strobe check.  If /RD arrived during the few
               instructions above, do not publish a late word; let the post-RD
               path record a clean preload miss instead. */
            if((sio_hw->gpio_hi_in & RD_HI_MASK)==0u) break;

            pre_addr=a2;
            pre_value=expected_data((uint16_t)a2);
            if(!queue_drive(pre_value)){
                abandon_cycle=true;
                break;
            }
            preloaded=true;
            s_preload_seen++;
            s_last_off=(uint16_t)pre_addr;
            s_last_data=pre_value;

            /* A preloaded word is valid only while THIS address/control tuple
               remains present.  If it changes before /RD, cancel the PIO word
               rather than letting a future unrelated /RD consume it. */
            while((sio_hw->gpio_in & PHI2_MASK) != 0u){
                uint32_t hiw=sio_hw->gpio_hi_in;
                if((hiw & RD_HI_MASK)==0u) break;
                uint32_t low=sio_hw->gpio_in;
                uint32_t high=sio_hw->gpio_hi_in;
                if((low & WR_MASK)==0u || (high & WRAMSEL_HI_MASK)==0u ||
                   (high & ROMSEL_HI_MASK)==0u ||
                   raw_address24(low,high)!=pre_addr){
                    s_preload_cancel++;
                    s_preload_addr_change++;
                    s_pio_stale++;
                    data_release();
                    preloaded=false;
                    abandon_cycle=true;
                    break;
                }
            }
            break;
        }

        if(abandon_cycle){
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }

        /* If PHI2 ended before /RD, the queued word must not survive into the
           next bus cycle. */
        if((sio_hw->gpio_in & PHI2_MASK)==0u){
            s_phi_no_rd++;
            if(preloaded){
                s_preload_no_rd++;
                s_preload_cancel++;
                s_pio_stale++;
                data_release();
            }
            prev_phi=false;
            continue;
        }

        /* /RD is now active.  If we preloaded successfully, PIO2 has already
           raised IRQ2 and SM2 is enabling DATA in hardware while Core1 performs
           the authoritative v1.0.4 post-RD validation below. */
        s_rd_edges++;
        s_phi_rd++;
        if(preloaded){
            s_driving=true;
            s_driven++;
            s_drive_budget--;
        }

        __asm volatile("nop\n nop\n");
        if((sio_hw->gpio_in & PHI2_MASK)==0u){
            s_too_late++;
            if(preloaded){ s_post_bad++; data_release(); }
            prev_phi=false;
            continue;
        }
        uint32_t rd_lo1=sio_hw->gpio_in;
        uint32_t rd_hi1=sio_hw->gpio_hi_in;
        uint32_t a1=raw_address24(rd_lo1,rd_hi1);

        __asm volatile("nop\n nop\n");
        if((sio_hw->gpio_in & PHI2_MASK)==0u){
            s_too_late++;
            if(preloaded){ s_post_bad++; data_release(); }
            prev_phi=false;
            continue;
        }
        uint32_t rd_lo2=sio_hw->gpio_in;
        uint32_t rd_hi2=sio_hw->gpio_hi_in;
        uint32_t a2=raw_address24(rd_lo2,rd_hi2);
        s_addr_double++;

        if(a1!=a2){
            s_addr_unstable++;
            if(preloaded){ s_post_bad++; data_release(); }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }

        bool bank_ok=((a2>>16)==0x00u);
        uint16_t off=(uint16_t)a2;
        bool range_ok=(off>=0x6000u && off<=0x6bffu);
        if(!bank_ok){
            s_rd_bank_miss++;
            if(preloaded){ s_post_bad++; data_release(); }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }
        if(!range_ok){
            s_rd_range_miss++;
            if(preloaded){ s_post_bad++; data_release(); }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }
        s_phi_target++;
        s_phi_target_rd++;

        uint32_t lo_ctrl=sio_hw->gpio_in;
        uint32_t hi_ctrl=sio_hw->gpio_hi_in;
        if((lo_ctrl & PHI2_MASK)==0u){
            s_too_late++;
            if(preloaded){ s_post_bad++; data_release(); }
            prev_phi=false;
            continue;
        }
        if((lo_ctrl & WR_MASK)==0u){
            s_gate_ctrl++;
            if(preloaded){ s_post_bad++; data_release(); }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }
        if((hi_ctrl & WRAMSEL_HI_MASK)==0u){
            s_gate_wramsel++;
            if(preloaded){ s_post_bad++; data_release(); }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }
        if((hi_ctrl & ROMSEL_HI_MASK)==0u){
            s_gate_romsel++;
            s_romsel_rd_low++;
            /* PIO2 should have suppressed IRQ2, so this must never have driven.
               If software believed it had preloaded/driven, record the hazard and
               force the pins back to input immediately. */
            if(preloaded){
                s_drive_during_rom++;
                s_post_bad++;
                data_release();
            }
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }
        s_romsel_rd_high++;

        /* A target read that reached /RD without a preload is intentionally NOT
           driven.  This is safer than falling back to the late v1.0.4 path and
           gives us a clean preload_miss counter for timing diagnosis. */
        if(!preloaded){
            s_preload_miss++;
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }

        if(a2!=pre_addr){
            s_post_addr_mismatch++;
            s_post_bad++;
            data_release();
            while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
            prev_phi=false;
            continue;
        }

        s_post_ok++;
        s_cpu_r++;
        s_rd_safe++;
        s_active_target++;
        s_last_off=off;
        s_last_data=pre_value;

        /* By the time the post-RD double sample finishes, SM2 should already be
           driving.  This echo is now a useful direct measure of the hardware
           gate rather than of Core1->PIO queue latency. */
        __asm volatile("nop\n nop\n");
        if((sio_hw->gpio_in & PHI2_MASK)!=0u){
            uint8_t echo=raw_data(sio_hw->gpio_in);
            s_echo_target++;
            if(echo==pre_value) s_echo_match++;
            else { s_echo_bad++; s_active_echo_bad++; }
        }else{
            s_pio_stale++;
        }

        while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
        s_preload_live=false;
        s_driving=false;
        s_fixed_hold++;
        prev_phi=false;
    }
}
void cx4bus_launch_core1(void){ if(s_core1_started)return; multicore_launch_core1(core1_loop); while(!s_core1_started)tight_loop_contents(); }

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
        const uint8_t m[4]={'B','U','S','6'};
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
        s_phase=0x10u; s_write_seen=s_write_bad_data=s_write_bad_addr=0; s_expected_off=0x6000u; s_read_errors=0xffffu;
    }else if(data==0x20u){
        if(s_phase==0x10u) finish_write_phase();
        s_phase=0x20u; s_reg_index=0; s_reg_bad=0;
    }else if(data==0x30u){
        s_phase=0x30u;
        if(s_reg_index==6u && s_reg_bad==0u)s_reg_passes++;
        /* v1.0.2: DO NOT arm here.  This marker arrives through the
           PIO+DMA write-capture path and can be processed after most of the
           following read loop has already happened.  Arming here was why
           v0.9.1/v1.0.1 only saw 19/0 target reads. */
    }else if(data==0x40u){
        s_phase=0x40u;
        s_last_read_errors=s_read_errors;
        if(s_last_read_errors==0u) s_read_pass_good++; else s_read_pass_bad++;
        s_read_passes++; s_passes++;

        /* First close an active validation pass, if one was running. */
        if(s_arm_request && s_arm_wait_pass){
            s_active_last_err=s_read_errors;
            s_active_passes++;
            s_arm_request=false;
            s_arm_wait_pass=false;
            s_drive_budget=0;
            s_auto_disarm++;
            data_release();
        }

        /* A queued BUSARM begins HERE, at the end marker of the preceding
           pass.  There is then an entire display delay + WRITE phase + REG
           phase before the next READ phase starts, so core1 is already armed
           long before the first $6000 read.  This deliberately compensates
           for the latency of the PIO+DMA mailbox scanner. */
        if(s_arm_pending){
            s_arm_pending=false;
            s_arm_request=true;
            s_arm_wait_pass=true;
            s_drive_budget=4096u;
            s_active_target=0;
            s_active_echo_bad=0;
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
                    s_first_bad_exp=off; s_first_bad_got=off; s_first_bad_data=data; s_first_bad_expected_data=ev;
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

void cx4bus_pio_read(uint32_t address,uint8_t data){
    if(!s_arm_request || !s_magic || (address>>16)!=0x00u) return;
    uint16_t off=(uint16_t)address;
    if(off<0x6000u || off>0x6bffu) return;
    uint8_t exp=expected_data(off);
    s_sample_target++;
    s_sample_last_off=off; s_sample_last_got=data; s_sample_last_exp=exp;
    if(data==exp) s_sample_match++; else s_sample_bad++;
}

void cx4bus_service(void){
    if(s_reset_requested){
        s_reset_requested=false;
        s_arm_request=false;
        s_arm_pending=false;
        s_arm_wait_pass=false;
        s_drive_budget=0;
        data_release();
        clear_state();
    }
}
void cx4bus_print_trace(void){ printf("BUSTRACE last=%04X:%02X phase=%02X magic=%u mask=%X hits=%llu bad=%llu\n",s_last_off,s_last_data,s_phase,s_magic?1u:0u,s_magic_mask,(unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad); }
void cx4bus_print_runs(void){ printf("BUSREG seq=%u index=%lu bad=%lu expected=7F49:00 7F4A:80 7F4B:02 7F4D:0E 7F4E:00 7F4F:5C\n",s_seq,(unsigned long)s_reg_index,(unsigned long)s_reg_bad); }
void cx4bus_selfcheck(void){ printf("BUSTEST ROM expected: test_rom/CX4_BUS_TEST.sfc (no injected game ROM required)\n"); }
void cx4bus_print_status(void){
    bool ready = (s_active_passes > 0u) && (s_active_last_err == 0u) &&
                 (s_active_echo_bad == 0u) && (s_drive_during_rom == 0u) &&
                 (s_post_bad == 0u);
    printf("BUSSTAT mode=BUS_VALIDATOR_V1.0.6_ROMSEL_HW_GATE verdict=%s arm=%u pending=%u gate=%u drive=%s magic=%u seq=%u phase=%02X passes=%llu "
           "write_passes=%llu write_seen=%lu bad_data=%lu bad_addr=%lu count_bad=%lu bad_data_total=%llu bad_addr_total=%llu missing_total=%llu extra_total=%llu "
           "reg_passes=%llu reg_index=%lu reg_bad=%lu read_passes=%llu cur_err=%u last_err=%u read_good=%llu read_bad=%llu "
           "magic_mask=%X magic_hits=%llu magic_bad=%llu pio_w=%llu phi=%llu phi_target=%llu phi_rd=%llu phi_target_rd=%llu phi_no_rd=%llu rd_edges=%llu rd_safe=%llu rd_bank_miss=%llu rd_range_miss=%llu "
           "addr2=%llu unstable=%llu too_late=%llu cpu_r=%llu driven=%llu budget=%lu budget_cut=%llu auto_disarm=%llu edge_release=%llu "
           "active_passes=%llu active_last_err=%u active_target=%llu active_echo_bad=%llu "
           "pio_ready=%u pio_q=%llu pio_full=%llu pio_stale=%llu "
           "pre_live=%u pre=%llu pre_cancel=%llu pre_addrchg=%llu pre_no_rd=%llu pre_miss=%llu pre_unstable=%llu "
           "post_ok=%llu post_bad=%llu post_addr_mismatch=%llu gate_rom=%llu gate_wram=%llu gate_ctrl=%llu "
           "romsel_pre=%llu/%llu romsel_rd=%llu/%llu drive_during_rom=%llu "
           "echo=%llu/%llu bad=%llu sample=%llu/%llu bad=%llu sample_last=%04X:%02X/%02X "
           "first_bad=%04lX/%04lX:%02X/%02X last=%04X:%02X\n",
           ready?"READY_FOR_GAME":"NOT_YET",
           s_arm_request?1u:0u,s_arm_pending?1u:0u,(s_arm_request&&s_magic)?1u:0u,s_driving?"ON":"OFF",s_magic?1u:0u,s_seq,s_phase,
           (unsigned long long)s_passes,(unsigned long long)s_write_passes,(unsigned long)s_write_seen,
           (unsigned long)s_write_bad_data,(unsigned long)s_write_bad_addr,(unsigned long)s_write_count_bad,
           (unsigned long long)s_write_bad_data_total,(unsigned long long)s_write_bad_addr_total,
           (unsigned long long)s_write_missing_total,(unsigned long long)s_write_extra_total,
           (unsigned long long)s_reg_passes,(unsigned long)s_reg_index,(unsigned long)s_reg_bad,
           (unsigned long long)s_read_passes,(unsigned)s_read_errors,(unsigned)s_last_read_errors,
           (unsigned long long)s_read_pass_good,(unsigned long long)s_read_pass_bad,
           s_magic_mask,(unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad,(unsigned long long)s_pio_w,
           (unsigned long long)s_phi_cycles,(unsigned long long)s_phi_target,(unsigned long long)s_phi_rd,
           (unsigned long long)s_phi_target_rd,(unsigned long long)s_phi_no_rd,
           (unsigned long long)s_rd_edges,(unsigned long long)s_rd_safe,
           (unsigned long long)s_rd_bank_miss,(unsigned long long)s_rd_range_miss,
           (unsigned long long)s_addr_double,(unsigned long long)s_addr_unstable,(unsigned long long)s_too_late,
           (unsigned long long)s_cpu_r,(unsigned long long)s_driven,(unsigned long)s_drive_budget,
           (unsigned long long)s_budget_cut,(unsigned long long)s_auto_disarm,(unsigned long long)s_fixed_hold,
           (unsigned long long)s_active_passes,(unsigned)s_active_last_err,(unsigned long long)s_active_target,(unsigned long long)s_active_echo_bad,
           s_drive_ready?1u:0u,(unsigned long long)s_pio_queued,(unsigned long long)s_pio_fifo_full,(unsigned long long)s_pio_stale,
           s_preload_live?1u:0u,(unsigned long long)s_preload_seen,(unsigned long long)s_preload_cancel,
           (unsigned long long)s_preload_addr_change,(unsigned long long)s_preload_no_rd,
           (unsigned long long)s_preload_miss,(unsigned long long)s_preload_unstable,
           (unsigned long long)s_post_ok,(unsigned long long)s_post_bad,(unsigned long long)s_post_addr_mismatch,
           (unsigned long long)s_gate_romsel,(unsigned long long)s_gate_wramsel,(unsigned long long)s_gate_ctrl,
           (unsigned long long)s_romsel_pre_high,(unsigned long long)s_romsel_pre_low,
           (unsigned long long)s_romsel_rd_high,(unsigned long long)s_romsel_rd_low,
           (unsigned long long)s_drive_during_rom,
           (unsigned long long)s_echo_match,(unsigned long long)s_echo_target,(unsigned long long)s_echo_bad,
           (unsigned long long)s_sample_match,(unsigned long long)s_sample_target,(unsigned long long)s_sample_bad,
           s_sample_last_off,s_sample_last_got,s_sample_last_exp,
           (unsigned long)s_first_bad_exp,(unsigned long)s_first_bad_got,s_first_bad_data,s_first_bad_expected_data,
           s_last_off,s_last_data);
}
