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

keyState get_key_status(void) {
    return (keyState) mock();
}

bool get_engine_status(void) {
    return (bool) mock();
}

bool get_all_doors_closed(void) {
    return (bool) mock();
}

bool get_reverse_gear(void) {
    return (bool) mock();
}

voltage get_voltage_battery(void) {
    return (voltage) mock();
}

steeringAngle get_steering_angle(void) {
    return (steeringAngle) mock();
}

bool get_oncoming_traffic(void) {
    return (bool) mock();
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
    will_return(get_key_status, data.key_state);
    will_return(get_engine_status, data.engine_on);
    will_return(get_all_doors_closed, data.all_doors_closed);
    will_return(get_reverse_gear, data.reverse_gear);
    will_return(get_voltage_battery, data.voltage_battery);
    will_return(get_steering_angle, data.steering_angle);
    will_return(get_oncoming_traffic, data.oncomming_trafic);
}

void mock_and_execute(sensors_and_time data) {
    mock_all_sensors(data);
    light_do_step();
}

#define assert_light_state(x) ls = get_light_state(); ref = (light_state) x; assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

void sequence1(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // ignition: key inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);

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

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // tunnel: value at border
    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 200);
    mock_and_execute(sensor_states);
    
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // brightness value below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    // brightness exceeds value but no three seconds time
    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    // below threshold
    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 199);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    // exceeds hystheresis value
    sensor_states = update_sensors(sensor_states, sensorTime, 8000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 251);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // tunnel ride terminated
    sensor_states = update_sensors(sensor_states, sensorTime, 12000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to on
    sensor_states = update_sensors(sensor_states, sensorTime, 13000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    
    // turning engine off
    sensor_states = update_sensors(sensor_states, sensorTime, 19000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch auto
    sensor_states = update_sensors(sensor_states, sensorTime, 20000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // switch on light on
    sensor_states = update_sensors(sensor_states, sensorTime, 21000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0}));

    // key removal: light off
    sensor_states = update_sensors(sensor_states, sensorTime, 22000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // pitman arm to downward7
    sensor_states = update_sensors(sensor_states, sensorTime, 23000);
    pitman_vertical(pa_Downward7);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 10, 0, 10, 0, 0, 0, 0, 0, 0}));

    // switch to auto
    sensor_states = update_sensors(sensor_states, sensorTime, 24000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence2(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    toggle_ambient_light();
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to auto
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // light switch to on
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // insert key
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // engine start
    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    // light switch: auto
    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    // engine off
    sensor_states = update_sensors(sensor_states, sensorTime, 10000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 25000);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 50000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 75000);
    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 104000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 105000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}


void sequence3(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    toggle_daytime_running_light();
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // key inserted
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    // engine start
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 6000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 7000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0}));
}

void sequence4(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    toggle_daytime_running_light();
    toggle_ambient_light();
    // why doesnt this work? assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    sensor_states = update_sensors(sensor_states, sensorTime, 24000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 34000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence5(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    toggle_ambient_light();
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    set_light_rotary_switch(lrs_auto);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 4000);
    set_light_rotary_switch(lrs_on);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 33000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 50, 50, 50, 50, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 34000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, NoKeyInserted);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

void sequence6(void **state) {
    sensors_and_time sensor_states = {0}; // TODO: maybe not a TODO
    light_state ls, ref;

    sensor_states = update_sensors(sensor_states, sensorAllDoorsClosed, 1);
    sensor_states = update_sensors(sensor_states, sensorBrightnessSensor, 500);
    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    pitman_vertical(pa_Downward5);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5000);
    pitman_vertical(pa_Downward5);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5200);
    pitman_vertical(pa_ud_Neutral);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5499);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

    sensor_states = update_sensors(sensor_states, sensorTime, 5500);
    mock_and_execute(sensor_states);

    assert_light_state(((light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
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
    };
    return run_tests(tests);
}
