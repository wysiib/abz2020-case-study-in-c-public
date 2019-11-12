#ifndef LIGHT_ACTUATORS_H_INCLUDED
#define LIGHT_ACTUATORS_H_INCLUDED

#include <stdbool.h>

#include "../common/common.h"

typedef enum {
    hbm_min = 0, // 65 m
    hbm_one = 1, // 100 m
    // 20 m step size
    hbm_max = 14
} high_beam_motor;

typedef enum {
    hbr_low = 0,
    hbr_high = 300
} high_beam_range;

void set_blink_left(percentage p);
void set_blink_right(percentage p);

void set_low_beam_left(percentage p);
void set_low_beam_right(percentage p);

void set_high_beam(bool on);
void set_high_beam_range(high_beam_range range);
void set_high_beam_motor(high_beam_motor motor);

void set_cornering_light_left(percentage p);
void set_cornering_light_right(percentage p);

void set_brake_light(percentage p);

// USA + Canada: tail lamps take task of rear indicator lamps
void set_tail_lamp_left(percentage p);
void set_tail_lamp_right(percentage p);

void set_reverse_light(percentage p);

#endif
