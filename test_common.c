#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <google/cmockery.h>

#include <assert.h>

#include "light/user-interface.h"
#include "light/light-state.h"
#include "light/light-impl.h"
#include "light/sensors.h"

#include "cruise-control/scs-impl.h"

#include "cruise-control/user-interface.h"
#include "cruise-control/sensors.h"

#include "test_common.h"

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
    scs_do_step();
}
