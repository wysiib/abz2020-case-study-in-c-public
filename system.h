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

void init_system(driverPosition pos, bool armored_vehicle, marketCode code);

#endif