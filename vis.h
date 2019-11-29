#ifndef VIS_H
#define VIS_H

#include "light/light-state.h"
#include "cruise-control/scs-state.h"
#include "stdio.h"
#include "string.h"

#include "test_common.h"

struct statenode;

typedef struct transition {
    char* name;
    sensors_and_time sensors_and_time;
    struct statenode* from;
    struct statenode* to;
} transition;

typedef struct statenode {
    light_state light_state;
    scs_state scs_state;

    char* name;
    transition prev;
    transition succ;
} statenode;


#define emit_var_str_sensors(x) { \
    if(t.from == NULL || t.from->prev.sensors_and_time.x != t.sensors_and_time.x){ \
        sprintf(printbuf,"%s : %s: %ld\n",t.name,#x,(long)t.sensors_and_time.x); \
        strcat(result,printbuf); }}

#define emit_var_str_light(x) { \
    if(n.prev.from == NULL || n.prev.from->light_state.x != s.x){ \
        sprintf(printbuf,"%s : %s: %ld\n",n.name,#x,(long)s.x); \
        strcat(result,printbuf); }}

#define emit_var_str_scs(x) { \
    if(n.prev.from == NULL || n.prev.from->scs_state.x != s.x){ \
        sprintf(printbuf,"%s : %s: %ld\n",n.name,#x,(long)s.x); \
        strcat(result,printbuf); }}

#define emit_var_str_scs_acoustic(x) { \
    if(n.prev.from == NULL || n.prev.from->scs_state.acoustic_warning.x != s.acoustic_warning.x){\
        sprintf(printbuf,"%s : acustic_warning.%s: %ld\n",n.name,#x,(long)s.acoustic_warning.x);\
        strcat(result,printbuf); }}

static const size_t PRINTBUF_SIZE = 1024;

char* emit_sensor_state_str(transition t){
    char* result = malloc(PRINTBUF_SIZE*16); //just something big
    char* printbuf = malloc(PRINTBUF_SIZE);

    emit_var_str_sensors(time)
    emit_var_str_sensors(engine_on)
    emit_var_str_sensors(key_state)
    emit_var_str_sensors(camera_state)
    emit_var_str_sensors(reverse_gear)
    emit_var_str_sensors(current_speed)
    emit_var_str_sensors(steering_angle)
    emit_var_str_sensors(voltage_battery)
    emit_var_str_sensors(all_doors_closed)
    emit_var_str_sensors(oncomming_trafic)
    emit_var_str_sensors(brightness_sensor)
    emit_var_str_sensors(range_radar_state)
    emit_var_str_sensors(range_radar_distance)

    free(printbuf);

    //printf("%s",result);
    return result;
}

char* emit_light_state_str(statenode n){
    light_state s = n.light_state;
    char* result = malloc(PRINTBUF_SIZE*16); //just something big
    char* printbuf = malloc(PRINTBUF_SIZE);

    emit_var_str_light(brakeLight)
    emit_var_str_light(blinkLeft)
    emit_var_str_light(blinkRight)
    emit_var_str_light(lowBeamLeft)
    emit_var_str_light(lowBeamRight)
    emit_var_str_light(tailLampLeft)
    emit_var_str_light(tailLampRight)
    emit_var_str_light(highBeamOn)
    emit_var_str_light(highBeamMotor)
    emit_var_str_light(highBeamRange)
    emit_var_str_light(corneringLightLeft)
    emit_var_str_light(corneringLightRight)
    emit_var_str_light(reverseLight)

    free(printbuf);

    return result;
}


char* emit_cruise_control_str(statenode n){
    scs_state s = n.scs_state;

    char* result = malloc(PRINTBUF_SIZE*16); //just something big
    char* printbuf = malloc(PRINTBUF_SIZE);

    emit_var_str_scs(cruise_control_active)
    emit_var_str_scs(desired_speed)
    emit_var_str_scs(has_previous_desired_speed)
    emit_var_str_scs(previous_desired_speed)
    emit_var_str_scs(target_speed)
    emit_var_str_scs(current_speed)
    emit_var_str_scs(acceleration)
    emit_var_str_scs(lever_pos)
    emit_var_str_scs(lever_prev_pos)
    emit_var_str_scs(lever_last_tic)
    emit_var_str_scs(lever_release_processed)
    emit_var_str_scs(lever_continuous)
    emit_var_str_scs(vehicle_speed_infront)
    emit_var_str_scs(vehicle_acceleration_infront)
    emit_var_str_scs(safety_dist)
    emit_var_str_scs(safety_dist_time)
    emit_var_str_scs(visual_warning_on)

    emit_var_str_scs_acoustic(is_on)
    emit_var_str_scs_acoustic(start_time)
    emit_var_str_scs_acoustic(playing_sound)
    emit_var_str_scs_acoustic(started_playing)

    emit_var_str_scs(brake_assistant_available)
    emit_var_str_scs(brake_pressure)
    emit_var_str_scs(brake_warning_playing)

    free(printbuf);

    return result;

}

#endif // VIS_H
