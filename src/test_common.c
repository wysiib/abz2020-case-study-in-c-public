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
#define vis 1
#if vis
    #include "stdio.h"
    #include "vis.h"
    static int mock_count = 0;
    static statenode head = {0};
    static statenode* latest = &head;

    static sensors_and_time init_sensor_state;
#endif

brightness nondet_get_brightness(void) {
    return (brightness) mock();
}

keyState nondet_get_key_status(void) {
    return (keyState) mock();
}

bool nondet_get_engine_status(void) {
    return (bool) mock();
}

bool nondet_get_all_doors_closed(void) {
    return (bool) mock();
}

bool nondet_get_reverse_gear(void) {
    return (bool) mock();
}

voltage nondet_get_voltage_battery(void) {
    return (voltage) mock();
}

steeringAngle nondet_get_steering_angle(void) {
    return (steeringAngle) mock();
}

bool nondet_get_oncoming_traffic(void) {
    return (bool) mock();
}

size_t nondet_get_time(void) {
    return (size_t) mock();
}

sensorState nondet_get_camera_state(void) {
    return (sensorState) mock();
}

vehicleSpeed nondet_get_current_speed(void) {
    return (vehicleSpeed) mock();
}

sensorState nondet_get_range_radar_state(void) {
    return (sensorState) mock();
}

rangeRadar nondet_read_range_radar_sensor(void) {
    return (rangeRadar) mock();
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
        case sensorCameraState:
            assert(value == Dirty || value == Ready || value == NotReady);
            data.camera_state = (sensorState) value;
            break;
        case sensorCurrentSpeed:
            assert(value >= speed_min && value <= speed_max);
            data.current_speed = (vehicleSpeed) value;
            break;
        case sensorRangedRadar:
            assert((value >= nothing_detected && value <= distance_max) ||
                   (value == faulty));
            data.range_radar_distance = (rangeRadar) value;
            break;
        case sensorRangedRadarState:
            assert((value == Ready) || (value == NotReady) || (value == Dirty));
            data.range_radar_state = (sensorState) value;
            break;
        default: assert(0);
    }
    return data;
}

bool sensors_and_time_equal(sensors_and_time s1, sensors_and_time s2){
    return
    s1.time == s2.time &&
    s1.engine_on == s2.engine_on &&
    s1.key_state == s2.key_state &&
    s1.camera_state == s2.camera_state &&
    s1.reverse_gear == s2.reverse_gear &&
    s1.current_speed == s2.current_speed &&
    s1.steering_angle == s2.steering_angle &&
    s1.voltage_battery == s2.voltage_battery &&
    s1.all_doors_closed == s2.all_doors_closed &&
    s1.oncomming_trafic == s2.oncomming_trafic &&
    s1.brightness_sensor == s2.brightness_sensor &&
    s1.range_radar_state == s2.range_radar_state &&
    s1.range_radar_distance == s2.range_radar_distance;
}

void mock_all_sensors(sensors_and_time data) {
    will_return(nondet_get_brightness, data.brightness_sensor);
    will_return(nondet_get_time, data.time);
    will_return(nondet_get_key_status, data.key_state);
    will_return(nondet_get_engine_status, data.engine_on);
    will_return(nondet_get_all_doors_closed, data.all_doors_closed);
    will_return(nondet_get_reverse_gear, data.reverse_gear);
    will_return(nondet_get_voltage_battery, data.voltage_battery);
    will_return(nondet_get_steering_angle, data.steering_angle);
    will_return(nondet_get_oncoming_traffic, data.oncomming_trafic);
    will_return(nondet_get_camera_state, data.camera_state);
    will_return(nondet_get_current_speed, data.current_speed);
    will_return(nondet_get_range_radar_state, data.range_radar_state);
    will_return(nondet_read_range_radar_sensor, data.range_radar_distance);
}

#if vis
scs_state get_scs_state(){
    scs_state s = {0};
    return s;
}
#endif

void mock_and_execute(sensors_and_time data) {
    mock_all_sensors(data);
    light_do_step(nondet_sys_get_driver_position(), nondet_sys_is_armoured_vehicle(),
            nondet_sys_get_market_code(), nondet_sys_get_ambient_light(), nondet_sys_get_daytime_running_light());
    scs_do_step();
    #if vis
        if(mock_count == 0){
            head.light_state = get_light_state();
            head.scs_state = get_scs_state();
            char* namebuf = malloc(128);
            sprintf(namebuf,"State%d",mock_count);
            head.name = namebuf;
            mock_count++;

            init_sensor_state = data;
            head.prev.sensors_and_time = data;
            head.prev.name = "InitialSensorState";

            return;
        }
        light_state light_state = get_light_state();
        scs_state scs_state = get_scs_state();

            sensors_and_time in_sensors_without_time = data;
            in_sensors_without_time.time = 0;
            sensors_and_time old_sensors_without_time = latest->prev.sensors_and_time;
            old_sensors_without_time.time = 0;

        if(sensors_and_time_equal(in_sensors_without_time,old_sensors_without_time) &&
           light_state_equal(light_state,latest->light_state) /* &&
           scs_state_equals(scs_state,latest->scs_state) */ ){
            return;
        }
        statenode* new_statenode = malloc(sizeof (statenode));
        new_statenode->light_state = light_state;
        new_statenode->scs_state = scs_state;
        char* namebuf = malloc(128);
        sprintf(namebuf,"State%d",mock_count);
        new_statenode->name = namebuf;

        char* transition_name_buf = malloc(128);
        sprintf(transition_name_buf,"%dTo%d",mock_count-1,mock_count);

        new_statenode->prev = (transition){transition_name_buf,data,latest,new_statenode};
        latest->succ = new_statenode->prev;

        mock_count++;

        latest = new_statenode;

    #endif
}

#if vis

    void print_els_visualization(){
        statenode* node = &head;
        char* init_sensors_str = emit_sensor_state_str(head.prev);
        printf("@startuml\n");
        printf("%s\n",init_sensors_str);
        printf("%s --> %s : read init sensor state\n",head.prev.name,head.name);
        while(node != NULL){
            char* light = emit_light_state_str(*node);
            printf("%s\n\n",light);
            if(node->succ.to != NULL){
                char* sensors_str = emit_sensor_state_str(node->succ);
                printf("%s --> %s : sensor change\n",node->succ.from->name,node->succ.name);

                printf("\n%s\n",sensors_str);

                printf("%s --> %s : sensor read\n",node->succ.name,node->succ.to->name);
                printf("%s --> %s : state change\n",node->succ.from->name,node->succ.to->name);
            }
            node = node->succ.to;
        }
        printf("\n@enduml\n");
    }
    void print_els_visualization_v2(){
        statenode* node = &head;


        printf("@startuml\n");
        printf("[*] --> %s : init\n",node->name);
        while(node != NULL){
            printf("state %s{\n",node->name);

            if(node->prev.name == NULL){
                printf("previous name not set. skipping...\n");
                return;
            }

            char* old_prev_name = node->prev.name;
            char* new_prev_name = calloc(128,sizeof(char));
            strcat(new_prev_name,"SensorS");
            strcat(new_prev_name,node->name+1);
            node->prev.name = new_prev_name;
            char* sensors_str = emit_sensor_state_str(node->prev);
            printf("state %s{\n%s\n}\n",node->prev.name,sensors_str);



            char* old_node_name = node->name;
            char* new_node_name = calloc(128,sizeof(char));
            strcat(new_node_name,"LightS");
            strcat(new_node_name,node->name+1);
            node->name=new_node_name;
            char* light = emit_light_state_str(*node);
            printf("state %s{\n%s\n}\n",node->name,light);


            printf("%s -right-> %s : causes\n",node->prev.name,node->name);

            printf("\n}\n");

            node->name = old_node_name;
            free(new_node_name);

            node->prev.name = old_prev_name;
            free(new_prev_name);


            if(node->succ.to != NULL){
                printf("%s --> %s : state change diff\n",node->succ.from->name,node->succ.to->name);
            }

            /* This arrow makes plantuml layout the whole diagram in questionable fashion.
            if(node->prev.from != NULL){
                printf("Light%s --> Light%s \n",node->prev.from->name,node->name);
            }
            */
            node = node->succ.to;
        }
        printf("\n@enduml\n");
    }

    void print_and_reset(){
        print_els_visualization_v2();
        //just leak memory. The output string buffers shouldn't be to big to be an issue.
        statenode empty = {0};
        head = empty;
    }
#endif
