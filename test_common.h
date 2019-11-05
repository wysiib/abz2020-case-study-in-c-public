#ifndef TEST_COMMON_H_INCLUDED
#define TEST_COMMON_H_INCLUDED


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

#define assert_light_state(x) ls = get_light_state(); ref = (light_state) x; assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));
#define progress_time(start, end, state) {for (time = start; time <= end; time++) { sensor_states = update_sensors(sensor_states, sensorTime, time); mock_and_execute(sensor_states); assert_light_state(((light_state) state)); }}

#endif
