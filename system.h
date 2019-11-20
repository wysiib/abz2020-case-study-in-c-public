#ifndef SYSTEM_H_INCLUDED
#define SYSTEM_H_INCLUDED

#include <stdbool.h>

typedef enum { 
    leftHand, rightHand
} driverPosition;

typedef enum {
    // USA + Canada: direction indicator is realised
    // by blinking red tail light
    USA = 1,
    Canada = 2,
    // Europe: blinker is extra yellow light
    EU = 3 
    // TODO: more?
} marketCode;

typedef struct init{
    driverPosition pos;
    bool armored_vehicle;
    marketCode market_code;
    bool ambient_light;
    bool daytime_running_light;
}init;

void init_system_v2(init i);

void init_system(driverPosition pos, bool armored_vehicle, marketCode code, bool ambient_light, bool daytime_running_light);

driverPosition get_driver_position(void);

bool is_armoured_vehicle(void);

marketCode get_market_code(void);

bool get_daytime_running_light(void);

bool get_ambient_light(void);

// for cmbc assertions
#define implies(x,y) ((!(x)) || (y))

#endif
