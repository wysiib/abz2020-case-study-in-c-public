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

void els1(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;
    size_t time;

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

    sensor_states = update_sensors(sensor_states, sensorTime, 2400);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2500);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    progress_time(2501, 2999, ((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    mock_and_execute(sensor_states);
    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    progress_time(3001, 3498, ((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

int main(int argc, char* argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        unit_test_setup_teardown(els1, reset, reset),
    };
    return run_tests(tests);
}
