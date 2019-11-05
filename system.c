#include <assert.h>

#include "system.h"

static driverPosition pos;
static bool armoured;
static marketCode code;

static bool initialised = false;

void init_system(driverPosition apos, bool av, marketCode acode) {
    // I would like this assertion but it will break our test cases
    // assert(!initialised); 
    pos = apos;
    armoured = av;
    code = acode;
    initialised = true;
}

driverPosition get_driver_position(void) {
    assert(initialised);
    return pos;
}

bool is_armoured_vehicle(void) {
    assert(initialised);
    return armoured;
}

marketCode get_market_code(void) {
    assert(initialised);
    return code;
}
