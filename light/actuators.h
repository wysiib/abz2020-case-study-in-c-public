#ifndef __LIGHT_ACTUATORS_H_INCLUDED
#define __LIGHT_ACTUATORS_H_INCLUDED

#include <stdbool.h>

#include "../common/common.h"

typedef enum {
    hbr_low = 0,
    hbr_high = 300
} highBeamRange;

typedef enum {
    hbm_low = 0, // 65 m
    hbm_one = 1, // 100 m
    // 20 m step size
    hbm_high = 14
} highBeamMotor;

void set_blink_left(percentage p);
void set_blink_right(percentage p);

void set_low_beam_left(percentage p);
void set_low_beam_right(percentage p);

void set_high_beam(bool on);
void set_high_beam_range(highBeamRange range);
void set_high_beam_motor(highBeamMotor motor);

void set_cornering_light_left(percentage p);
void set_cornering_light_right(percentage p);

void set_brake_light(percentage p);

// USA + Canada: tail lamps take task of rear indicator lamps
void set_tail_lamp_left(percentage p);
void set_tail_lamp_right(percentage p);

void set_reverse_light(percentage p);

#endif
