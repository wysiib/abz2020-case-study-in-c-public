#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <google/cmockery.h>

#include <assert.h>
#include <string.h>

#include "light/user-interface.h"
#include "light/light-state.h"
#include "light/light-impl.h"
#include "light/sensors.h"

#include "cruise-control/user-interface.h"

#include "system.h"

brightness get_brightness(void) {
    return (brightness) mock();
}

size_t get_time(void) {
    return (size_t) mock();
}


typedef enum sensors_and_time_key {
    sensorKeyState,
    sensorEngineOn,
    sensorAllDoorsClosed,
    sensorBrightnessSensor,
    sensorReverseGear,
    sensorVoltageBattery,
    sensorSteeringAngle,
    sensorOncommingTraffic,
    sensorTime
} sensors_and_time_key;

typedef struct sensors_and_time {
    keyState key_state;
    bool engine_on;
    bool all_doors_closed;
    brightness brightness_sensor;
    bool reverse_gear;
    voltage voltage_battery;
    steeringAngle steering_angle;
    bool oncomming_trafic;
    size_t time;
} sensors_and_time;

sensors_and_time update_sensors(sensors_and_time data, sensors_and_time_key key, int value) {
    switch (key) {
        case sensorKeyState:
            assert(value >= 0 && value <= 2);
            data.key_state = (keyState) value;
            break;
        case sensorEngineOn:
            assert(value >= 0 && value <= 1);
            data.engine_on = (bool) value;
            break;
        case sensorAllDoorsClosed:
            assert(value >= 0 && value <= 1);
            data.all_doors_closed = (bool) value;
            break;
        case sensorBrightnessSensor:
            assert(value >= brightness_min && value <= brightness_max);
            data.brightness_sensor = (brightness) value;
            break;
        case sensorReverseGear:
            assert(value >= 0 && value <= 1);
            data.reverse_gear = (bool) value;
            break;
        case sensorVoltageBattery:
            assert(value >= voltage_min && value <= voltage_max);
            data.voltage_battery = (voltage) value;
            break;
        case sensorSteeringAngle:
            assert(value >= st_calibrating && value <= st_hard_right_max);
            data.steering_angle = (steeringAngle) value;
            break;
        case sensorOncommingTraffic:
            assert(value >= 0 && value <= 1);
            data.oncomming_trafic = (bool) value;
            break;
        case sensorTime:
            assert(value >= 0);
            data.time = (size_t) value;
            break;
        default: assert(0);
    }
    return data;
}

void mock_all_sensors(sensors_and_time data) {
    will_return(get_brightness, data.brightness_sensor);
    will_return(get_time, data.time);
}

void mock_and_execute(sensors_and_time data) {
    mock_all_sensors(data);
    light_do_step();
}

void assert_light_state(light_state ref) {
    light_state ls = get_light_state();
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));
}


// A test case that does nothing and succeeds.
void sequence1(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO

    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    // TODO: mock?

    // ignition: key inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);

    mock_and_execute(sensor_states);

    // ignition on
    sensor_states = update_sensors(sensor_states, sensorTime, 2);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    set_light_rotary_switch(lrs_auto);

    // sensor: light outside
    sensor_states = update_sensors(sensor_states, sensorTime, 3);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    // tunnel: value at border
    sensor_states = update_sensors(sensor_states, sensorTime, 4);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);
    
    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    // brightness value below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 5);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0});

    // brightness exceeds value but no three seconds time
    sensor_states = update_sensors(sensor_states, sensorTime, 6);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0});

    // below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 7);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0});

    // exceeds hystheresis value
    sensor_states = update_sensors(sensor_states, sensorTime, 8);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    // tunnel ride terminated
    sensor_states = update_sensors(sensor_states, sensorTime, 12);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    // light switch to on
    sensor_states = update_sensors(sensor_states, sensorTime, 13);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0});

    
    // turning engine off
    sensor_states = update_sensors(sensor_states, sensorTime, 19);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
}

int main(int argc, char* argv[]) {
    const UnitTest tests[] = {
        unit_test(sequence1),
    };
    return run_tests(tests);
}
