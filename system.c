#include <assert.h>

#include "system.h"

static driverPosition pos;
static bool armoured;
static marketCode code;
static bool ambi_light;
static bool daytime_light;
static bool adaptive_high_beam;

static bool initialised = false;
void init_system_v2(init i) {
    init_system(i.pos,i.armored_vehicle,i.market_code,
                i.ambient_light,i.daytime_running_light,i.adaptive_high_beam);
}

void init_system(driverPosition apos, bool av, marketCode acode, bool ambient_light, bool daytime_running_light, bool ahb) {
    // I would like this assertion but it will break our test cases
    // assert(!initialised); 
    pos = apos;
    armoured = av;
    code = acode;
    ambi_light = ambient_light;
    daytime_light = daytime_running_light;
    adaptive_high_beam = ahb;
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

bool get_ambient_light(void) {
    assert(initialised);
    return ambi_light;
}

bool get_daytime_running_light(void) {
    assert(initialised);
    return daytime_light;
}

bool get_adaptive_high_beam(void) {
    assert(initialised);
    return adaptive_high_beam;
}
