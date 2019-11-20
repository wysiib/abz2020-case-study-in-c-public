#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <google/cmockery.h>

#include <string.h>
#include <limits.h>

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
    // any subsystem. 🤷‍
}

void els1_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);

    int i;
    for (i = 2; i < 100; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els1_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Upward7);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 0, blinkRight, 100);

    int i;
    for (i = 2; i < 100; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 0, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els2_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);
    pitman_vertical(pa_ud_Neutral);

    int i;
    for (i = 2; i < 5; i++) { // three cycles
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    progress_time_partial2(5000, 10000, blinkLeft, 0, blinkRight, 0);
}

void els3_a_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Upward7);

    progress_time_partial2(2000, 2499, blinkLeft, 100, blinkRight, 0);
    progress_time_partial2(2500, 2999, blinkLeft, 0, blinkRight, 0);

    int i;

    for (i = 3; i < 6; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 0, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els3_b_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    toggle_hazard_warning();

    progress_time_partial2(2000, 2499, blinkLeft, 100, blinkRight, 0);
    progress_time_partial2(2500, 2999, blinkLeft, 0, blinkRight, 0);

    int i;
    for (i = 3; i < 100; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els4_left_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);

    int i;
    for (i = 2; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time_partial2(10000, 20000, blinkLeft, 0, blinkRight, 0);
}


void els4_left_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);

    int i;
    for (i = 2; i < 3; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time_partial2(3000, 10000, blinkLeft, 0, blinkRight, 0);
}

void els6_left_canada(void **state) {
    init_system(leftHand, false, Canada, false, true, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);


    int i;
    for (i = 2; i < 10; i++) {
        progress_time(i*1000, i * 1000 + 499, ((light_state) {0, 100, 0, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 50, 100, 0, 0, 0, 0, 0, 0, 0}));
    }
    pitman_vertical(pa_ud_Neutral);

    progress_time(10000, 15000, ((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0}));
}

void els6_left_usa(void **state) {
    init_system(leftHand, false, USA, false, true, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);


    int i;
    for (i = 2; i < 10; i++) {
        progress_time(i * 1000, i * 1000 + 499, ((light_state) {0, 100, 0, 50, 100, 100, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 50, 100, 0, 0, 0, 0, 0, 0, 0}));
    }
    pitman_vertical(pa_ud_Neutral);

    progress_time(10000, 15000, ((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0}));
}

void els7_a_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    progress_time_partial2(2001, 2400, blinkLeft, 100, blinkRight, 0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2401); // less than .5 seconds
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    progress_time_partial2(2500, 2599, blinkLeft, 0, blinkRight, 0);
    pitman_vertical(pa_ud_Neutral);
    progress_time_partial2(2600, 2999, blinkLeft, 0, blinkRight, 0);

    int i;

    for (i = 3; i < 5; i++) { // queued command: tip blinking
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
    progress_time_partial2(5000, 11000, blinkLeft, 0, blinkRight, 0);
}

void els7_b_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_partial_state2(blinkLeft, 100, blinkRight, 0);
    pitman_vertical(pa_Downward7);
    sensor_states = update_sensors(sensor_states, sensorTime, 2001);
    mock_and_execute(sensor_states);

    progress_time_partial2(2001, 2499, blinkLeft, 100, blinkRight, 0);
    progress_time_partial2(2500, 2999, blinkLeft, 0, blinkRight, 0);

    int i;

    for (i = 3; i <= 11; i++) { // queued command: direction blinking
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els8_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);

    toggle_hazard_warning();

    int i;
    for (i = 2; i < 100; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els8_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key NOT inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);

    mock_and_execute(sensor_states);

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 5; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 332, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 333, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els9(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);

    mock_and_execute(sensor_states);

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    /* reacts immediatelly for now
    sensor_states = update_sensors(sensor_states, sensorTime, 10200);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 100, blinkRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 10800);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 0, blinkRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 11200);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 100, blinkRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 11800);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 0, blinkRight, 0);
    */

    // dark cycle is prolonged due to key removal
    progress_time_partial2(9999,10166, blinkLeft, 0, blinkRight, 0);


    for (i = 10; i < 20; i++) {
        progress_time_partial2(i * 1000 + 167,       i * 1000 + 167 + 332, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 167 + 333, i * 1000 + 167 + 999, blinkLeft, 0, blinkRight, 0);
    }

    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    /* again, immediatelly for now
    sensor_states = update_sensors(sensor_states, sensorTime, 20200);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 100, blinkRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 20800);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 0, blinkRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 21200);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 100, blinkRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 21800);
    mock_and_execute(sensor_states);
    assert_partial_state2(blinkLeft, 0, blinkRight, 0);
    */

    for (i = 20; i < 21; i++) {
        progress_time_partial2(i * 1000 + 167,       i * 1000 + 167 + 498, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 167 + 499, i * 1000 + 167 + 998, blinkLeft, 0, blinkRight, 0); // TODO: what happens to 999?
    }
}

void els12_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);
    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    toggle_hazard_warning();

    for (i = 10; i < 20; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els12_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);

    // different ordering
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    toggle_hazard_warning();

    for (i = 10; i < 20; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els13_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);
    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time_partial2(1000, 1499, blinkLeft, 100, blinkRight, 100);
    progress_time_partial2(1500, 1999, blinkLeft, 0, blinkRight, 0);
    toggle_hazard_warning();
    progress_time_partial2(2000, 5000, blinkLeft, 0, blinkRight, 0);
}

void els13_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time_partial2(1000, 1499, blinkLeft, 100, blinkRight, 100);
    progress_time_partial2(1500, 1999, blinkLeft, 0, blinkRight, 0);
    toggle_hazard_warning();
    progress_time_partial2(2000, 5000, blinkLeft, 0, blinkRight, 0);
}

void els14(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    progress_time_partial2(1000, 3000, lowBeamLeft, 100, lowBeamRight, 100);
}

void els15(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    progress_time_partial2(1000, 3000, lowBeamLeft, 50, lowBeamRight, 50);
}

void els16a(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);

    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els16b(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);

    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els16conflict17a(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=true,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);

    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els16conflict17b(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=true,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);

    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els17(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=0,
                          .daytime_running_light=true,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els18a(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 1100);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1200);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1300);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 250);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1400);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1500);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 201);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1600);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1700);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1800);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 1900);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 10000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 201);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 14000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);
}
void els18b(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249); // 250 already included, see scenarios
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, INT_MAX); //time could actually be higher, but update-sensors only accepts ints
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 250);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els18c(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,.ambient_light=false,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, INT_MAX); //time could actually be higher, but update-sensors only accepts ints
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 249);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);
}

void els19a(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight prolongs low beams 30 seconds on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 32000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els19b(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight does not activate light when key state change to insert
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_off);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0); // ambilight will NOT *turn on* light

    sensor_states = update_sensors(sensor_states, sensorTime, 32001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els19c(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight activates light with reset of 30 seconds
    //reset via door open & close
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 12000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 41000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, false); //open door
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 71000); //still on?
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 71000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, true); //close door
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 101000); //still on?
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 101001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);//now off
}
void els19d(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight activates light with reset of 30 seconds
    //reset via key insert / removal
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    //still on?
    sensor_states = update_sensors(sensor_states, sensorTime, 32000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 32000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);//insert key
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    //still on?
    sensor_states = update_sensors(sensor_states, sensorTime, 62000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 62001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els19f(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test delayed ambilight activates light
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);

    mock_and_execute(sensor_states);

    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, false);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 33000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, false);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 33000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 63000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 999);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 65001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}
void els19conflict28a(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight activates light for no longer than 30 seconds without parking light
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 32000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 32001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,10,lowBeamRight,10);
}
void els19conflict28b(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    //test ambilight activates light for no longer than 30 seconds without parking light
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    pitman_vertical(pa_Upward7);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 32000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,percentage_high,lowBeamRight,percentage_high);

    sensor_states = update_sensors(sensor_states, sensorTime, 32001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);
    assert_partial_state2(lowBeamLeft,10,lowBeamRight,10);
}

void els21(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=true,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false,
                          .adaptive_high_beam=false});
    sensors_and_time sensor_states = {0};
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, false);
    mock_and_execute(sensor_states);

    // assuming els19a works
    toggle_darkness_mode();

    sensor_states = update_sensors(sensor_states, sensorTime, 2001);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}

void els23_left_a(void **state) {
    // precedence direction blinking over tail lights
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated
    assert_partial_state2(tailLampLeft, 100, tailLampRight, 100); // tail lamps activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, tailLampLeft, 100, tailLampRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, tailLampLeft, 0, tailLampRight, 100);
    }
}

void els23_right_a(void **state) {
    // precedence direction blinking over tail lights
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated
    assert_partial_state2(tailLampLeft, 100, tailLampRight, 100); // tail lamps activated

    pitman_vertical(pa_Upward7); // direction blinking requested
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, tailLampLeft, 100, tailLampRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, tailLampLeft, 100, tailLampRight, 0);
    }
}

void els23_b(void **state) {
    // precedence hazard lights over tail lights
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated
    assert_partial_state2(tailLampLeft, 100, tailLampRight, 100); // tail lamps activated

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    int i;
    for (i = 1; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, tailLampLeft, 100, tailLampRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, tailLampLeft, 0, tailLampRight, 0);
    }
}


void els24_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 100, corneringLightRight, 0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_ud_Neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 100, corneringLightRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 49, corneringLightRight, 0); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els24_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Upward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 100);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_ud_Neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 0, corneringLightRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 49); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els25_left(void **state) {
    init_system(leftHand, true, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    toggle_darkness_mode();
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els25_right(void **state) {
    init_system(leftHand, true, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    toggle_darkness_mode();
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els26_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_hard_left_min); // 11 degree deflection
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 100, corneringLightRight, 0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 100, corneringLightRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 49, corneringLightRight, 0); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els26_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_hard_right_min); // 11 degree deflection
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 100);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 0, corneringLightRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 49); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els27_left_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorReverseGear, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 100);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_ud_Neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 0, corneringLightRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 49); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els27_right_a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorReverseGear, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Upward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 100, corneringLightRight, 0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    pitman_vertical(pa_ud_Neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 100, corneringLightRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 49, corneringLightRight, 0); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els27_left_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorReverseGear, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_hard_left_min); // 11 degree deflection
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 100);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 0, corneringLightRight, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 49); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els27_right_b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorReverseGear, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h
    //mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_hard_right_min); // 11 degree deflection
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 100, corneringLightRight, 0);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorSteeringAngle, st_neutral); // back
    mock_and_execute(sensor_states);

    progress_time_partial2(2000, 6999, corneringLightLeft, 100, corneringLightRight, 0);
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 7501); // gentle fade-out
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 49, corneringLightRight, 0); // FIXME: or 50?

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    mock_and_execute(sensor_states);
    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els28_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);

    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    progress_time_partial4(1000, 5000, lowBeamLeft, 10, lowBeamRight, 0, tailLampLeft, 10, tailLampRight, 0);
}

void els28_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);

    pitman_vertical(pa_Upward7);
    mock_and_execute(sensor_states);

    progress_time_partial4(1000, 5000, lowBeamLeft, 0, lowBeamRight, 10, tailLampLeft, 0, tailLampRight, 10);
}

void els30(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);
    mock_and_execute(sensor_states);

    pitman_horizontal(pa_Forward);
    mock_and_execute(sensor_states);

    progress_time_partial1(1000, 3000, highBeamOn, true);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els31(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    progress_time_partial3(1000, 3000, highBeamOn, true, highBeamMotor, 7, highBeamRange, 100);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els33_34_35(void **state) {
    init_system(leftHand, false, EU, false, false, true);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    /* ELS-33 */
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, true);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Ready);
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1800);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);


    progress_time_partial3(3000, 6000, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11);

    /* ELS-34 */
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, true);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 6500);
    progress_time_partial3(6500, 10000, highBeamOn, true, highBeamRange, 30, highBeamMotor, 0);

    /* ELS-35 */
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    mock_and_execute(sensor_states);
    sensor_states = update_sensors(sensor_states, sensorTime, 12000);

    progress_time_partial3(12000, 1500, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11);
}

void els34_35(void **state) {
    init_system(leftHand, false, EU, false, false, true);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, true);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Ready);
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1800);

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);


    progress_time_partial3(3000, 6000, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11 /* ? */);
}

void els36(void **state) {
    init_system(leftHand, false, EU, false, false, true);
    sensors_and_time sensor_states = {0};
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, true);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Ready);
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 315);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 31, highBeamMotor, 0);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 316);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 31, highBeamMotor, 1);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 657);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 57, highBeamMotor, 1);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 658);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 57, highBeamMotor, 2);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 859);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 73, highBeamMotor, 2);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 860);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 73, highBeamMotor, 3);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1015);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 85, highBeamMotor, 3);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1016);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 85, highBeamMotor, 4);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1147);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 95, highBeamMotor, 4);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1148);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 95, highBeamMotor, 5);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1263);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 5);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1264);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 6);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1367);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 6);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1368);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 7);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1462);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 7);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1463);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 8);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1551);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 8);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1552);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 9);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1634);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 9);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1635);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 10);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1712);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 10);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1713);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 11);

    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 2800);
    mock_and_execute(sensor_states);
    assert_partial_state3(highBeamOn, true, highBeamRange, 100, highBeamMotor, 11);
}

void els38(void **state) {
    init_system(leftHand, false, EU, false, false, true);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, true);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Ready);
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 1800);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, 90);

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);


    progress_time_partial3(3000, 6000, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11);

    pitman_horizontal(pa_fb_Neutral);
    mock_and_execute(sensor_states);
    progress_time_partial3(6000, 10000, highBeamOn, false, lowBeamLeft, 100, lowBeamRight, 100);
}

void els41(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, true);
    sensor_states = update_sensors(sensor_states, sensorReverseGear, true);

    mock_and_execute(sensor_states);

    progress_time_partial1(1000, 3000, reverseLight, 100);

    sensor_states = update_sensors(sensor_states, sensorReverseGear, false);
    mock_and_execute(sensor_states);

    progress_time_partial1(3000, 6000, reverseLight, 0);
}

static voltage volt(int num) {
    return num * 10;
}

void els42(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Forward);
    mock_and_execute(sensor_states);

    progress_time_partial1(1000, 3000, highBeamOn, false);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els43(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    progress_time_partial3(1000, 3000, highBeamOn, true, highBeamMotor, 7, highBeamRange, 100);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els44(void **state) {
    init_system_v2((init){.pos=leftHand,.armored_vehicle=false,
                          .market_code=EU,
                          .ambient_light=true,
                          .daytime_running_light=false});
    sensors_and_time sensor_states = {0};
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft,0,lowBeamRight,0);
}

void els45_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els45_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, 50); // less than 10 km/h

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_partial_state2(lowBeamLeft, 100, lowBeamRight, 100); // low beam activated

    pitman_vertical(pa_Downward7); // direction blinking requested
    mock_and_execute(sensor_states);

    assert_partial_state2(corneringLightLeft, 0, corneringLightRight, 0);
}

void els46_left(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);

    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    progress_time_partial4(1000, 5000, lowBeamLeft, 0, lowBeamRight, 0, tailLampLeft, 0, tailLampRight, 0);
}

void els46_right(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorVoltageBattery, volt(8));
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);

    pitman_vertical(pa_Upward7);
    mock_and_execute(sensor_states);

    progress_time_partial4(1000, 5000, lowBeamLeft, 0, lowBeamRight, 0, tailLampLeft, 0, tailLampRight, 0);
}

void els49a(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorCameraState, Dirty);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    progress_time_partial3(1000, 3000, highBeamOn, true, highBeamMotor, 7, highBeamRange, 100);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els49b(void **state) {
    init_system(leftHand, false, EU, false, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorCameraState, NotReady);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    progress_time_partial3(1000, 3000, highBeamOn, true, highBeamMotor, 7, highBeamRange, 100);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}


int main(int argc, char* argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        unit_test_setup_teardown(els1_left, reset, reset),
        unit_test_setup_teardown(els1_right, reset, reset),
        unit_test_setup_teardown(els2_left, reset, reset),
        unit_test_setup_teardown(els3_a_left, reset, reset),
        unit_test_setup_teardown(els3_b_left, reset, reset),
        unit_test_setup_teardown(els4_left_a, reset, reset),
        unit_test_setup_teardown(els4_left_b, reset, reset),
        // TODO: most test cases for right side are missing (ELS-5)
        unit_test_setup_teardown(els6_left_canada, reset, reset),
        unit_test_setup_teardown(els6_left_usa, reset, reset),
        unit_test_setup_teardown(els7_a_left, reset, reset),
        unit_test_setup_teardown(els7_b_left, reset, reset),
        unit_test_setup_teardown(els8_a, reset, reset),
        unit_test_setup_teardown(els8_b, reset, reset),
        unit_test_setup_teardown(els9, reset, reset),
        // TODO: most tests for American cars
        // NOTE: ELS-10 implicitly in previous tests
        // NOTE: ELS-11 implicitly in previous tests
        unit_test_setup_teardown(els12_a, reset, reset),
        unit_test_setup_teardown(els12_b, reset, reset),
        unit_test_setup_teardown(els13_a, reset, reset),
        unit_test_setup_teardown(els13_b, reset, reset),
        unit_test_setup_teardown(els14, reset, reset),
        unit_test_setup_teardown(els15, reset, reset),
        unit_test_setup_teardown(els16a, reset, reset),
        unit_test_setup_teardown(els16b, reset, reset),
        unit_test_setup_teardown(els16conflict17a, reset, reset),
        unit_test_setup_teardown(els16conflict17b, reset, reset),
        unit_test_setup_teardown(els17, reset, reset),
        unit_test_setup_teardown(els18a, reset, reset),
        unit_test_setup_teardown(els18b, reset, reset),
        unit_test_setup_teardown(els18c, reset, reset),
        // NOTE: Probably still not enough els19 tests
        unit_test_setup_teardown(els19a, reset, reset),
        unit_test_setup_teardown(els19b, reset, reset),
        unit_test_setup_teardown(els19c, reset, reset),
        unit_test_setup_teardown(els19d, reset, reset),
        unit_test_setup_teardown(els19f, reset, reset),
        // NOTE: IMPLEMENT PITMAN change!
        unit_test_setup_teardown(els19conflict28a, reset, reset),
        unit_test_setup_teardown(els19conflict28b, reset, reset),

        // NOTE: ELS-20 is deleted
        unit_test_setup_teardown(els21, reset, reset),
        // TODO: ELS-22
        unit_test_setup_teardown(els23_left_a, reset, reset),
        unit_test_setup_teardown(els23_right_a, reset, reset),
        unit_test_setup_teardown(els23_b, reset, reset),
        /* TODO: ELS-23: does tip blinking have precedence over tail lights? */
        unit_test_setup_teardown(els24_left, reset, reset),
        unit_test_setup_teardown(els24_right, reset, reset),
        unit_test_setup_teardown(els25_left, reset, reset),
        unit_test_setup_teardown(els25_right, reset, reset),
        unit_test_setup_teardown(els26_left, reset, reset),
        unit_test_setup_teardown(els26_right, reset, reset),
        unit_test_setup_teardown(els27_left_a, reset, reset),
        unit_test_setup_teardown(els27_right_a, reset, reset),
        unit_test_setup_teardown(els27_left_b, reset, reset),
        unit_test_setup_teardown(els27_right_b, reset, reset),
        unit_test_setup_teardown(els28_left, reset, reset),
        unit_test_setup_teardown(els28_right, reset, reset),
        // NOTE: ESL-29: no test
        unit_test_setup_teardown(els30, reset, reset),
        unit_test_setup_teardown(els31, reset, reset),
        // NOTE: ELS-32: no test
        unit_test_setup_teardown(els33_34_35, reset, reset),
        unit_test_setup_teardown(els36, reset, reset),
        // TODO: ELS-37: make sense of that mess
        unit_test_setup_teardown(els38, reset, reset),
        // TODO: ELS-39: requires clean-up of common test file;
        // TODO: ELS 40| brake pedal is sensor in light subsystem,
        //               yet is part of the UI of the cruise
        unit_test_setup_teardown(els41, reset, reset),
        unit_test_setup_teardown(els42, reset, reset),
        unit_test_setup_teardown(els43, reset, reset),
        unit_test_setup_teardown(els44, reset, reset),
        unit_test_setup_teardown(els45_left, reset, reset),
        unit_test_setup_teardown(els45_right, reset, reset),
        unit_test_setup_teardown(els46_left, reset, reset),
        unit_test_setup_teardown(els46_right, reset, reset),
        // TODO: ELS-47
        // NOTE: ELS-48: no test
        unit_test_setup_teardown(els49a, reset, reset),
        unit_test_setup_teardown(els49b, reset, reset),

    };
    run_tests(tests);
    return 0;
}
