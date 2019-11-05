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
        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i*1000 + 1, i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000 + 500);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i * 1000 + 501, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
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
        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i*1000 + 1, i * 1000 + 499, ((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000 + 500);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i * 1000 + 501, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
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
        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i*1000 + 1, i * 1000 + 499, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000 + 500);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i * 1000 + 501, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }

    progress_time(5000, 10000, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void els3_a(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted + ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);


    sensor_states = update_sensors(sensor_states, sensorTime, 1998);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    pitman_vertical(pa_ud_Neutral);
    sensor_states = update_sensors(sensor_states, sensorTime, 1999);
    mock_and_execute(sensor_states);

    pitman_vertical(pa_Upward7);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    int i;

    for (i = 2; i < 100; i++) {
        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i*1000 + 1, i * 1000 + 499, ((light_state) {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

        sensor_states = update_sensors(sensor_states, sensorTime, i * 1000 + 500);
        mock_and_execute(sensor_states);
        assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        progress_time(i * 1000 + 501, i * 1000 + 999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
}

int main(int argc, char* argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        unit_test_setup_teardown(els1_left, reset, reset),
        unit_test_setup_teardown(els1_right, reset, reset),
        unit_test_setup_teardown(els2_left, reset, reset),
        unit_test_setup_teardown(els3_a, reset, reset),
    };
    return run_tests(tests);
}
