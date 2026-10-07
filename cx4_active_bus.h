#ifndef CX4_ACTIVE_BUS_H
#define CX4_ACTIVE_BUS_H
#include <stdbool.h>
#include <stdint.h>
void cx4bus_init(void);
void cx4bus_launch_core1(void);
void cx4bus_service(void);
void cx4bus_arm(bool enabled);
bool cx4bus_is_armed(void);
void cx4bus_reset_state(void);
void cx4bus_print_status(void);
void cx4bus_print_trace(void);
void cx4bus_print_runs(void);
void cx4bus_print_queue(void);
void cx4bus_selfcheck(void);
void cx4bus_status42(bool enabled);
void cx4bus_status42_clear(void);
void cx4bus_print_status42(void);
#endif
