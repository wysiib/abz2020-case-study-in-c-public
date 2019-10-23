#include <stdbool.h>

#include "light/user-interface.h"
#include "light/sensors.h"
#include "light/actuators.h"
#include "light/light-state.h"

#include "cruise-control/user-interface.h"
#include "cruise-control/sensors.h"
#include "cruise-control/actuators.h"

#include "system.h"


int main(int argc, char *argv[]) {
    // TODO: get parameters from arguments
    (void) argc; (void) argv;

    driverPosition driver_position = leftHand;
    bool armored_vehicle = false;
    marketCode market_code = EU;


    return 0;
}
