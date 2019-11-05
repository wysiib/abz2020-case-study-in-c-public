#ifndef TEST_COMMON_H_INCLUDED
#define TEST_COMMON_H_INCLUDED

#include <stdlib.h>

#include "light/light-state.h"

brightness get_brightness(void);

keyState get_key_status(void);

bool get_engine_status(void);

bool get_all_doors_closed(void);

bool get_reverse_gear(void);

voltage get_voltage_battery(void);

steeringAngle get_steering_angle(void);

bool get_oncoming_traffic(void);

size_t get_time(void);

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

sensors_and_time update_sensors(sensors_and_time data, sensors_and_time_key key, int value);

void mock_all_sensors(sensors_and_time data);

void mock_and_execute(sensors_and_time data);


#define assert_light_state(x) {light_state ref = (light_state) x; light_state ls = get_light_state(); assert_true(ls.brakeLight == ref.brakeLight); assert_true(ls.blinkLeft == ref.blinkLeft); assert_true(ls.blinkRight == ref.blinkRight); assert_true(ls.lowBeamLeft == ref.lowBeamLeft); assert_true(ls.lowBeamRight == ref.lowBeamRight); assert_true(ls.tailLampLeft == ref.tailLampLeft); assert_true(ls.tailLampRight == ref.tailLampRight); assert_true(ls.highBeamOn == ref.highBeamOn); assert_true(ls.highBeamMotor == ref.highBeamMotor); assert_true(ls.corneringLightLeft == ref.corneringLightLeft); assert_true(ls.corneringLightRight == ref.corneringLightRight); assert_true(ls.reverseLight == ref.reverseLight); }
#define progress_time(start, end, state) {size_t time; for (time = start; time <= end; time++) { sensor_states = update_sensors(sensor_states, sensorTime, time); mock_and_execute(sensor_states); assert_light_state(((light_state) state)); }}

#endif
