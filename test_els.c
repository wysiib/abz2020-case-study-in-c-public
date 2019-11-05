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
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    int i;
    for (i = 2; i < 100; i++) {
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
}

void els1_right(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    int i;
    for (i = 2; i < 100; i++) {
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
}

void els2_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);

    int i;
    for (i = 2; i < 5; i++) { // three cycles
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }

    progress_time(5000, 10000, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void els3_a_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2001);
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Upward7);

    int i;

    for (i = 2; i < 5; i++) {
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
}

void els3_b_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    toggle_hazard_warning();

    int i;
    for (i = 2; i < 100; i++) {
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
}

void els4_left_a(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    int i;
    for (i = 2; i < 10; i++) {
        progress_time(i * 1000, i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }

    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time(10000, 20000, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}


void els4_left_b(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    int i;
    for (i = 2; i < 3; i++) {
        progress_time(i*1000, i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }

    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    progress_time(3000, 10000, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void els6_left_canada(void **state) {
    init_system(leftHand, false, Canada);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    progress_time(2001, 2400, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2401); // less than .5 seconds
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    progress_time(2500, 2999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);

    int i;

    for (i = 3; i <= 7; i++) { // queued command: tip blinking
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
    progress_time(8000, 11000, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void els7_b_left(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

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

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_Downward7);
    sensor_states = update_sensors(sensor_states, sensorTime, 2001);
    mock_and_execute(sensor_states);

    progress_time(2001, 2499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_Downward5);
    progress_time(2500, 2999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);

    int i;

    for (i = 3; i <= 11; i++) { // queued command: direction blinking
        progress_time(i * 1000,       i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        progress_time(i * 1000 + 500, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
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
    };
    run_tests(tests);
    return 0;
}
