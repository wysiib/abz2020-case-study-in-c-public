#ifndef LIGHT_USER_INTERFACE_INCLUDED
#define LIGHT_USER_INTERFACE_INCLUDED

#include <stdbool.h>

typedef enum {lrs_off, lrs_auto, lrs_on} lightRotarySwitch;

typedef enum {pa_ud_Neutral, pa_Downward5, pa_Downward7, pa_Upward5, pa_Upward7} pitmanArmUpDown;
typedef enum {pa_fb_Neutral, pa_Backward, pa_Forward} pitmanArmForthBack;


void reset_user_interface(void);

void set_light_rotary_switch(lightRotarySwitch val);
lightRotarySwitch get_light_rotary_switch(void);

// trigger direction indicator (which one: unspecified so far)
// temporary activation: 5-7 degree
// permanent activation: 7 degree
void pitman_vertical(pitmanArmUpDown deflection);

// change boolean value
void toggle_hazard_warning(void);
bool get_hazard_warning(void);

// change boolean value, armoured vehicle only
void toggle_darkness_mode(void);

pitmanArmUpDown get_pitman_vertical(void);

void pitman_horizontal(pitmanArmForthBack deflection);

pitmanArmForthBack get_pitman_horizontal(void);


#endif
