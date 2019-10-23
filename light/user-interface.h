#ifndef __LIGHT_USER_INTERFACE_INCLUDED
#define __LIGHT_USER_INTERFACE_INCLUDED

typedef enum {lrs_off, lrs_auto, lrs_on} lightRotarySwitch;

typedef enum {pa_ud_Neutral, pa_Downward5, pa_Downward7, pa_Upward5, pa_Upward7} pitmanArmUpDown;
typedef enum {pa_fb_Neutral, pa_Backward, pa_Forward} pitmanArmForthBack;

void set_light_rotary_switch(lightRotarySwitch val);
lightRotarySwitch get_light_rotary_switch(void);

// permanent activation of the adaptive high beam, engages pitman arm
void pitman_backward(void);

// temporary activation of the high beam (non-engaging)
void pitman_forward(void);

void pitman_disengage_horizontal(void);

// trigger direction indicator (which one: unspecified so far)
// temporary activation: 5-7 degree
// permanent activation: 7 degree
void pitman_vertical(pitmanArmUpDown deflection);

// disengange automatically if steering wheel
// turned more than 10 degrees
void pitman_disengage_vertical(void);

// change boolean value
void toggle_hazard_warning(void);

// change boolean value, armoured vehicle only
void toggle_darkness_mode(void);

// change boolean value
void toggle_daytime_running_light(void);

// change boolean value
void toggle_ambient_light(void);

pitmanArmUpDown get_pitman_vertical(void);

#endif
