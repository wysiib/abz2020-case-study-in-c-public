#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <google/cmockery.h>

#include <string.h>

#include "light/user-interface.h"
#include "light/light-state.h"
#include "light/light-impl.h"
#include "light/sensors.h"

#include "cruise-control/user-interface.h"

#include "system.h"

#include "test_common.h"

void scs_do_step(void) {
    // FIXME: The SCS step function is a dependency for mock_and_execute in
    // test_common.c, but linking to scs-impl.c adds a big bunch of further
    // dependencies to the ELS, which currently should not be in here.
    // The two systems are not cleanly divided in the first place, as
    // for instance test_common.c is directly dependend on the light subsystem,
    // which does not really make sense for a common test suit usable by
    // any subsystem.
}

void sequence1(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);

    mock_and_execute(sensor_states);

    // ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);


    // sensor: light outside
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // tunnel: value at border
    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // brightness value below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    // brightness exceeds value but no three seconds time
    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    // below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    // exceeds hystheresis value
    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // tunnel ride terminated
    sensor_states = update_sensors(sensor_states, sensorTime, 12000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to on
    sensor_states = update_sensors(sensor_states, sensorTime, 13000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));


    // turning engine off
    sensor_states = update_sensors(sensor_states, sensorTime, 19000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch auto
    sensor_states = update_sensors(sensor_states, sensorTime, 20000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // switch on light on
    sensor_states = update_sensors(sensor_states, sensorTime, 21000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0, 0}));

    // key removal: light off
    sensor_states = update_sensors(sensor_states, sensorTime, 22000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // pitman arm to downward7
    sensor_states = update_sensors(sensor_states, sensorTime, 23000);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0, 0}));

    // switch to auto
    sensor_states = update_sensors(sensor_states, sensorTime, 24000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence2(void **state) {
    init_system(leftHand, false, EU, true, false, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to auto
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to on
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // insert key
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // engine start
    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    // light switch: auto
    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    // engine off
    sensor_states = update_sensors(sensor_states, sensorTime, 10000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 25000);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 50000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 75000);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 104000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 105000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}


void sequence3(void **state) {
    init_system(leftHand, false, EU, false, true, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // key inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // engine start
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0, 0}));
}

void sequence4(void **state) {
    init_system(leftHand, false, EU, true, true, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO


    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorTime, 24000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 34000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence5(void **state) {
    init_system(leftHand, false, EU, true, false, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 33000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 34000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence6(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    pitman_vertical(pa_Downward5);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5200);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5999);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6999);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7700);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7800);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 8500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 8700);
    pitman_vertical(pa_Upward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 9000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 9499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 9500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 9999);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10800);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence7(void **state) {
    init_system(leftHand, false, USA, false, true, false);

    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    pitman_vertical(pa_Upward7);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 100, 100, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 50, 100, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 100, 100, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5300);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 50, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10200);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 50, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    //progress_time(10201, 10499, ((light_state) {0, 100, 0, 50, 100, 100, 100, 0, 0, 0, 0, 0}));
    //progress_time(10500, 10799, ((light_state) {0, 0, 0, 50, 100, 0, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10800);
    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 100, 0, 100, 0, 0, 0, 0, 0, 0}));

    progress_time(10801, 10998, ((light_state) {0, 0, 0, 50, 100, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10999);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 100, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 11000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    progress_time(11001, 11499, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));
    progress_time(11500, 11999, ((light_state) {0, 0, 0, 50, 50, 0, 0, 0, 0, 0, 0, 0}));
    progress_time(12000, 12499, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));
    progress_time(12500, 12999, ((light_state) {0, 0, 0, 50, 50, 0, 0, 0, 0, 0, 0, 0}));
    progress_time(13000, 13299, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 13300);
    pitman_vertical(pa_Upward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 13400);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    progress_time(13401, 13499, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));
    progress_time(13500, 13999, ((light_state) {0, 0, 0, 50, 50, 0, 0, 0, 0, 0, 0, 0}));
    progress_time(14000, 14299, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 14300);
    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

    progress_time(14301, 14499, ((light_state) {0, 100, 100, 50, 50, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 14500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 0, 0, 0, 0, 0, 0, 0, 0}));

    progress_time(14500, 14999, ((light_state) {0, 0, 0, 50, 50, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 15000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence8(void **state) {
    init_system(leftHand, false, USA, false, false, false);

    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    mock_and_execute(sensor_states);
    // fixed test case: light on manually -> low beam and tail lamps on
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 1, 7, 100, 0, 0, 0}));

    pitman_horizontal(pa_fb_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    mock_and_execute(sensor_states);
    // fixed test case: see above
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    pitman_horizontal(pa_Forward);
    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    // again, see above
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 1, 7, 100, 0, 0, 0}));

    pitman_horizontal(pa_fb_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 9000);
    mock_and_execute(sensor_states);
    // again, see above
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

}

void sequence9(void **state) {
    init_system(leftHand, false, EU, false, false, true);

    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 135);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Ready);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    pitman_horizontal(pa_Backward);
    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 200);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 300);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 10000);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 301);
    mock_and_execute(sensor_states);
    // adapted test: we do immediate rampup
    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 1, 0, 30, 0, 0, 0}));

    // I dont understand the high beam motor from here on
}

int main(int argc, char* argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        unit_test_setup_teardown(sequence1, reset, reset),
        unit_test_setup_teardown(sequence2, reset, reset),
        unit_test_setup_teardown(sequence3, reset, reset),
        unit_test_setup_teardown(sequence4, reset, reset),
        unit_test_setup_teardown(sequence5, reset, reset),
        unit_test_setup_teardown(sequence6, reset, reset),
        unit_test_setup_teardown(sequence7, reset, reset),
        unit_test_setup_teardown(sequence8, reset, reset),
        unit_test_setup_teardown(sequence9, reset, reset),
    };
    run_tests(tests);
    return 0;
}
