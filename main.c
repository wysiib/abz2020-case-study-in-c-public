#include <stdbool.h>

#include "light/user-interface.h"
#include "light/sensors.h"
#include "light/actuators.h"

#include "cruise-control/user-interface.h"

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


int main(int argc, char *argv[]) {
    // TODO: get parameters from arguments
    (void) argc; (void) argv;

    driverPosition driver_position = leftHand;
    bool armored_vehicle = false;
    marketCode market_code = EU;


    return 0;
}
