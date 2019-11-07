#ifndef CRUISE_CONTROL_STATE
#define CRUISE_CONTROL_STATE

#include "../common/common.h"

typedef struct {
    bool has_previous_desired_speed;
    vehicleSpeed previous_desired_speed;
} scs_state;

scs_state get_scs_state(void);

void set_prev_desired_speed(vehicleSpeed prev);

void reset_prev_desired_speed();

#endif
