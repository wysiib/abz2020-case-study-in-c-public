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

void els1_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, Canada);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    toggle_daytime_running_light();

    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, USA);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    toggle_daytime_running_light();

    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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

    for (i = 3; i <= 5; i++) { // queued command: tip blinking
        progress_time_partial2(i * 1000,       i * 1000 + 499, blinkLeft, 100, blinkRight, 0);
        progress_time_partial2(i * 1000 + 500, i * 1000 + 999, blinkLeft, 0, blinkRight, 0);
    }
    progress_time_partial2(6000, 11000, blinkLeft, 0, blinkRight, 0);
}

void els7_b_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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


void els30(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

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
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);
    set_light_rotary_switch(lrs_on);
    pitman_horizontal(pa_Backward);
    mock_and_execute(sensor_states);

    // TODO: highBeamRange is new...?
    //progress_time_partial3(1000, 3000, highBeamOn, true, highBeamMotor, 7, highBeamRange, 100);

    //pitman_horizontal(pa_fb_Neutral);
    //progress_time_partial1(3000, 6000, highBeamOn, false);
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
        // TODO: ELS-16
        // TODO: ELS-17
        // TODO: ELS-18
        // TODO: ELS-19
        // NOTE: ELS-20 is deleted
        // TODO: ELS-21 to ESL-29
        unit_test_setup_teardown(els30, reset, reset),
        // TODO: ELS-31 is unfinished (new variable)
    };
    run_tests(tests);
    return 0;
}
