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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, Canada, false, true);
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
    init_system(leftHand, false, USA, false, true);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    for (i = 3; i < 100; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 333, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 334, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els9(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);

    mock_and_execute(sensor_states);

    toggle_hazard_warning();
    mock_and_execute(sensor_states);

    int i;
    for (i = 2; i < 10; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

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


    for (i = 12; i < 20; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 333, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 334, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }

    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

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

    for (i = 22; i < 30; i++) {
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 100);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
}

void els12_a(void **state) {
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=false});
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=false});
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=true});
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=true});
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
                          .marketCode=EU,.ambient_light=0,
                          .daytime_running_light=true});
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
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
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 250);
    mock_and_execute(sensor_states);
    assert_true(get_light_state().lowBeamLeft>0);
    assert_true(get_light_state().lowBeamRight>0);

    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 250);
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
                          .marketCode=EU,.ambient_light=false,
                          .daytime_running_light=false});
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
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
void els30(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    mock_and_execute(sensor_states);

    pitman_horizontal(pa_Forward);
    mock_and_execute(sensor_states);

    progress_time_partial1(1000, 3000, highBeamOn, true);

    pitman_horizontal(pa_fb_Neutral);
    progress_time_partial1(3000, 6000, highBeamOn, false);
}

void els31(void **state) {
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);


    progress_time_partial3(3000, 6000, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11 /* ? */);

    /* ELS-34 */
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, true);
    mock_and_execute(sensor_states);

    sensor_states = update_sensors(sensor_states, sensorTime, 6500);
    progress_time_partial3(6500, 10000, highBeamOn, true /* ?? */, highBeamRange, 30, highBeamMotor, 0 /* ? */);

    /* ELS-35 */
    sensor_states = update_sensors(sensor_states, sensorOncommingTraffic, false);
    mock_and_execute(sensor_states);
    sensor_states = update_sensors(sensor_states, sensorTime, 12000);

    progress_time_partial3(12000, 1500, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11 /* ? */);
}

void els34_35(void **state) {
    init_system(leftHand, false, EU, false, false);
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

void els38(void **state) {
    init_system(leftHand, false, EU, false, false);
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

    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);


    progress_time_partial3(3000, 6000, highBeamOn, true, highBeamRange, 100, highBeamMotor, 11 /* ? */);

    pitman_horizontal(pa_fb_Neutral);
    mock_and_execute(sensor_states);
    progress_time_partial3(6000, 10000, highBeamOn, false, lowBeamLeft, 100, lowBeamRight, 100);
}

void els41(void **state) {
    init_system(leftHand, false, EU, false, false);
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

voltage volt(int num) {
    return num * 10;
}

void els43(void **state) {
    init_system(leftHand, false, EU, false, false);
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

void els49a(void **state) {
    init_system(leftHand, false, EU, false, false);
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
    init_system(leftHand, false, EU, false, false);
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
        // TODO: ELS-19
        // NOTE: ELS-20 is deleted
        // TODO: ELS-21 to ESL-28
        // NOTE: ESL-29: no test
        unit_test_setup_teardown(els30, reset, reset),
        unit_test_setup_teardown(els31, reset, reset),
        // NOTE: ELS-32: no test
        unit_test_setup_teardown(els33_34_35, reset, reset),
        // TODO: ELS-36: define characteristic curves
        // TODO: ELS-37: make sense of that mess
        unit_test_setup_teardown(els38, reset, reset),
        // TODO: ELS-39: requires clean-up of common test file;
        // TODO: ELS 40| brake pedal is sensor in light subsystem,
        //               yet is part of the UI of the cruise
        unit_test_setup_teardown(els41, reset, reset),
        // TODO: ELS-42
        unit_test_setup_teardown(els43, reset, reset),
        // TODO: ELS-44
        // TODO: ELS-45
        // TODO: ELS-46
        // TODO: ELS-47
        // NOTE: ELS-48: no test
        unit_test_setup_teardown(els49a, reset, reset),
        unit_test_setup_teardown(els49b, reset, reset),

    };
    run_tests(tests);
    return 0;
}
