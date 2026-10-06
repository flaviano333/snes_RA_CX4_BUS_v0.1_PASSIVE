#include "cx4_active_bus.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

/*
 * v0.8.1 PHI2-WINDOW + READ-HOLD RESPONDER
 *
 * This deliberately does NOT emulate CX4. It validates the electrical/data
 * path with a deterministic test ROM. PIO+DMA is authoritative for writes.
 * Active readback is restricted to bank $00, $6000-$6BFF, and only after the
 * test ROM's BUS6 magic has been observed. This minimizes accidental drive.
 */
#define PIN_PHI2 0u
#define PIN_WR   1u
#define PIN_RD   35u
#define DATA_MASK ((1u << 2) | (1u << 3) | (1u << 4) | (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 10))
#define PHI2_MASK (1u << PIN_PHI2)
#define RD_HI_MASK (1u << (PIN_RD - 32u))

static uint8_t s_ram[0x0c00];
static volatile bool s_arm_request=false, s_driving=false, s_core1_started=false;
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
static volatile uint64_t s_hold_wait=0, s_hold_timeout=0;
static volatile uint64_t s_sample_target=0, s_sample_match=0, s_sample_bad=0;
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
static inline uint32_t packed_data(uint8_t v){ return (((uint32_t)v&7u)<<2)|(((uint32_t)v&0xf8u)<<3); }
static inline void data_release(void){ gpio_set_dir_in_masked(DATA_MASK); s_driving=false; }
static inline void data_drive(uint8_t v){ gpio_put_masked(DATA_MASK,packed_data(v)); gpio_set_dir_out_masked(DATA_MASK); s_driving=true; }

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
    s_hold_wait=s_hold_timeout=0; s_sample_target=s_sample_match=s_sample_bad=0;
    s_sample_last_off=0; s_sample_last_got=s_sample_last_exp=0; s_expected_off=0x6000u;
    s_last_off=0; s_last_data=0;
}

void cx4bus_init(void){
    clear_state(); s_arm_request=false; s_driving=false; s_reset_requested=false;
    const unsigned pins[8]={2,3,4,6,7,8,9,10};
    for(unsigned i=0;i<8;i++){ gpio_set_function(pins[i],GPIO_FUNC_SIO); gpio_set_dir(pins[i],GPIO_IN); gpio_disable_pulls(pins[i]); }
    data_release();
}
void cx4bus_arm(bool e){ s_arm_request=e; if(!e)data_release(); }
bool cx4bus_is_armed(void){ return s_arm_request; }
void cx4bus_reset_state(void){ s_reset_requested=true; }

static void __not_in_flash_func(core1_loop)(void){
    /*
     * v0.8: qualify the response inside the PHI2-high window.
     *
     * SNES cartridge timing guarantees that the CPU A-bus address is stable
     * while PHI2 is high, and read data only has to be valid at the falling
     * edge of PHI2.  v0.7 instead sampled the address immediately after /RD
     * fell; on hardware this caught only a tiny fraction of $6000-$6BFF reads.
     *
     * Sequence per bus cycle:
     *   1. wait for PHI2 rising edge;
     *   2. sample the now-stable address;
     *   3. if the address is our deliberately gated test window, wait for /RD
     *      to assert at any point during PHI2 high (not just at one instant);
     *   4. drive the preselected byte and KEEP it valid through PHI2 falling;
     *   5. immediately return D0-D7 to input.
     *
     * This remains a validator, not CX4 emulation.  It drives only bank $00,
     * $6000-$6BFF, after BUS6 magic and BUSARM.
     */
    s_core1_started=true;
    data_release();
    bool prev_phi = (sio_hw->gpio_in & PHI2_MASK) != 0u;

    for(;;){
        /* Find a real low -> high PHI2 edge. */
        bool phi;
        do {
            phi = (sio_hw->gpio_in & PHI2_MASK) != 0u;
            if(!phi) prev_phi=false;
        } while(!phi || prev_phi);
        prev_phi=true;
        s_phi_cycles++;

        /* Address is guaranteed stable during PHI2 high. */
        uint32_t lo=sio_hw->gpio_in;
        uint32_t hi=sio_hw->gpio_hi_in;
        uint32_t a=raw_address24(lo,hi);
        bool target = s_arm_request && s_magic &&
                      ((a>>16)==0x00u) &&
                      ((uint16_t)a>=0x6000u) && ((uint16_t)a<=0x6bffu);
        if(target) s_phi_target++;

        bool saw_rd = ((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u);
        /* /RD may assert after PHI2 has already risen.  Wait for it, but only
           while the same PHI2-high bus window remains active. */
        while(!saw_rd && ((sio_hw->gpio_in & PHI2_MASK) != 0u)) {
            saw_rd = ((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u);
        }

        if(saw_rd){
            s_rd_edges++;
            s_phi_rd++;
            if(s_arm_request && s_magic){
                if((a>>16)!=0x00u){
                    s_rd_bank_miss++;
                }else if((uint16_t)a<0x6000u || (uint16_t)a>0x6bffu){
                    s_rd_range_miss++;
                }else{
                    uint16_t off=(uint16_t)a;
                    uint8_t v=s_ram[off-0x6000u];
                    s_cpu_r++;
                    s_rd_safe++;
                    s_phi_target_rd++;
                    s_last_off=off;
                    s_last_data=v;
                    data_drive(v);
                    s_driven++;
                }
            }
        }else if(target){
            s_phi_no_rd++;
        }

        /* Data must remain valid through the sampling edge *and* for the
           cartridge-bus hold interval after PHI2 falls.  v0.8 released the
           bus immediately on the falling edge, which can be too early for
           the 5A22/input path.  Keep driving until /RD is released.  A hard
           loop bound prevents a stuck /RD from leaving the RP driving forever. */
        while((sio_hw->gpio_in & PHI2_MASK) != 0u) tight_loop_contents();
        if(s_driving){
            uint32_t hold=0;
            while((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u && hold < 256u){
                ++hold;
                tight_loop_contents();
            }
            s_hold_wait += hold;
            if((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u) s_hold_timeout++;
            data_release();
        }

        /* If /RD only became visible after PHI2 fell, count it as late. */
        if(target && !saw_rd && ((sio_hw->gpio_hi_in & RD_HI_MASK) == 0u)) s_phi_late++;
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
        if(s_phase==0x10u)finish_write_phase(); s_phase=0x20u; s_reg_index=0; s_reg_bad=0;
    }else if(data==0x30u){
        s_phase=0x30u; if(s_reg_index==6u && s_reg_bad==0u)s_reg_passes++;
    }else if(data==0x40u){
        s_phase=0x40u;
        s_last_read_errors=s_read_errors;
        if(s_last_read_errors==0u) s_read_pass_good++; else s_read_pass_bad++;
        s_read_passes++; s_passes++;
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
    uint8_t exp=s_ram[off-0x6000u];
    s_sample_target++;
    s_sample_last_off=off; s_sample_last_got=data; s_sample_last_exp=exp;
    if(data==exp) s_sample_match++; else s_sample_bad++;
}

void cx4bus_service(void){ if(s_reset_requested){ s_reset_requested=false; clear_state(); } }
void cx4bus_print_trace(void){ printf("BUSTRACE last=%04X:%02X phase=%02X magic=%u mask=%X hits=%llu bad=%llu\n",s_last_off,s_last_data,s_phase,s_magic?1u:0u,s_magic_mask,(unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad); }
void cx4bus_print_runs(void){ printf("BUSREG seq=%u index=%lu bad=%lu expected=7F49:00 7F4A:80 7F4B:02 7F4D:0E 7F4E:00 7F4F:5C\n",s_seq,(unsigned long)s_reg_index,(unsigned long)s_reg_bad); }
void cx4bus_selfcheck(void){ printf("BUSTEST ROM expected: test_rom/CX4_BUS_TEST.sfc (no injected game ROM required)\n"); }
void cx4bus_print_status(void){
    printf("BUSSTAT mode=BUS_VALIDATOR_V0.8.1_HOLD arm=%u gate=%u drive=%s magic=%u seq=%u phase=%02X passes=%llu "
           "write_passes=%llu write_seen=%lu bad_data=%lu bad_addr=%lu count_bad=%lu bad_data_total=%llu bad_addr_total=%llu missing_total=%llu extra_total=%llu "
           "reg_passes=%llu reg_index=%lu reg_bad=%lu read_passes=%llu cur_err=%u last_err=%u read_good=%llu read_bad=%llu "
           "magic_mask=%X magic_hits=%llu magic_bad=%llu pio_w=%llu phi=%llu phi_target=%llu phi_rd=%llu phi_target_rd=%llu phi_no_rd=%llu phi_late=%llu "
           "rd_edges=%llu rd_safe=%llu rd_bank_miss=%llu rd_range_miss=%llu cpu_r=%llu driven=%llu hold_wait=%llu hold_to=%llu "
           "sample_target=%llu sample_match=%llu sample_bad=%llu sample_last=%04X:%02X/%02X "
           "first_bad=%04lX/%04lX:%02X/%02X last=%04X:%02X\n",
           s_arm_request?1u:0u,(s_arm_request&&s_magic)?1u:0u,s_driving?"ON":"OFF",s_magic?1u:0u,s_seq,s_phase,
           (unsigned long long)s_passes,(unsigned long long)s_write_passes,(unsigned long)s_write_seen,
           (unsigned long)s_write_bad_data,(unsigned long)s_write_bad_addr,(unsigned long)s_write_count_bad,
           (unsigned long long)s_write_bad_data_total,(unsigned long long)s_write_bad_addr_total,
           (unsigned long long)s_write_missing_total,(unsigned long long)s_write_extra_total,
           (unsigned long long)s_reg_passes,(unsigned long)s_reg_index,(unsigned long)s_reg_bad,
           (unsigned long long)s_read_passes,(unsigned)s_read_errors,(unsigned)s_last_read_errors,
           (unsigned long long)s_read_pass_good,(unsigned long long)s_read_pass_bad,
           s_magic_mask,(unsigned long long)s_magic_hits,(unsigned long long)s_magic_bad,(unsigned long long)s_pio_w,
           (unsigned long long)s_phi_cycles,(unsigned long long)s_phi_target,(unsigned long long)s_phi_rd,
           (unsigned long long)s_phi_target_rd,(unsigned long long)s_phi_no_rd,(unsigned long long)s_phi_late,
           (unsigned long long)s_rd_edges,(unsigned long long)s_rd_safe,(unsigned long long)s_rd_bank_miss,
           (unsigned long long)s_rd_range_miss,(unsigned long long)s_cpu_r,(unsigned long long)s_driven,
           (unsigned long long)s_hold_wait,(unsigned long long)s_hold_timeout,
           (unsigned long long)s_sample_target,(unsigned long long)s_sample_match,(unsigned long long)s_sample_bad,
           s_sample_last_off,s_sample_last_got,s_sample_last_exp,
           (unsigned long)s_first_bad_exp,(unsigned long)s_first_bad_got,s_first_bad_data,s_first_bad_expected_data,
           s_last_off,s_last_data);
}
