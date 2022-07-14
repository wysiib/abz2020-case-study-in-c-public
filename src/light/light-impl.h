#ifndef LIGHT_LIGHT_IMPL_H_INCLUDED
#define LIGHT_LIGHT_IMPL_H_INCLUDED

#include "../system.h"

typedef enum {blink_left, blink_right, hazard, none} blinkingDirection;
typedef enum {cl_blink_left_rev, cl_blink_right_rev, cl_steer_left_rev, cl_steer_right_rev,
              cl_blink_left, cl_blink_right, cl_steer_left, cl_steer_right, cl_none} corneringLightReason;


void reset(void **state);
//void reset(void);

void light_do_step(driverPosition pos, bool armoured, marketCode code, bool ambi_light, bool daytime_light);

#endif
