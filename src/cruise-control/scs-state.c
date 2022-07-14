#include "scs-state.h"
#include "actuators.h"
#include <assert.h>

#include "user-interface.h"

static scs_state scs;

scs_state get_scs_state(void) {
    return scs;
}

//GETTERS

cruiseControlMode get_mode(void){
    return scs.mode;
}
bool get_cruise_control_active(void){
    return scs.cruise_control_active;
}
vehicleSpeed get_desired_speed(void){
    return scs.desired_speed;
}
bool get_has_previous_desired_speed(void){
    return scs.has_previous_desired_speed;
}
vehicleSpeed get_previous_desired_speed(void){
    return scs.previous_desired_speed;
}
vehicleSpeed get_target_speed(void){
    return scs.target_speed;
}
vehicleSpeed scs_internal_get_current_speed(void){
    return scs.current_speed;
}
vehicleAcceleration get_acceleration(void){
    return scs.acceleration;
}
SCSLever get_lever_pos(void){
    return scs.lever_pos;
}
SCSLever get_lever_prev_pos(void){
    return scs.lever_prev_pos;
}
size_t get_lever_last_tic(void){
    return scs.lever_last_tic;
}
bool get_lever_release_processed(void){
    return scs.lever_release_processed;
}
bool get_lever_continuous(void){
    return scs.lever_continuous;
}
vehicleSpeed get_vehicle_speed_infront(void){
    return scs.vehicle_speed_infront;
}
vehicleAcceleration get_vehicle_acceleration_infront(void){
    return scs.vehicle_acceleration_infront;
}
size_t get_safety_dist(void){
    return scs.safety_dist;
}
safetyDistance get_safety_dist_time(void){
    return scs.safety_dist_time;
}
bool get_visual_warning_on(void){
    return scs.visual_warning_on;
}
acousticSignal get_acoustic_warning(void){
    return scs.acoustic_warning;
}
bool get_brake_assistant_available(void){
    return scs.brake_assistant_available;
}
percentage get_brake_pressure(void){
    return scs.brake_pressure;
}
bool get_brake_warning_playing(void){
    return scs.brake_warning_playing;
}

//GETTERS


void reset_scs_state(void **state) {
    scs = (scs_state){0};
}

void set_scs_mode(cruiseControlMode mode) {
    scs.mode = mode;
}

void set_cruise_control(bool active) {
    if (!active && scs.cruise_control_active) {
        // When turning cruise control off, we save the previous speed.
        // TODO: Check if this is really supposed to happen this way.
        set_prev_desired_speed(scs.desired_speed);
    }
    scs.cruise_control_active = active;
}

void set_desired_speed(vehicleSpeed target) {
    assert((target >= speed_min) && (target <= speed_max));
    scs.desired_speed = target;
}

void set_prev_desired_speed(vehicleSpeed prev) {
    assert((prev >= speed_min) && (prev <= speed_max));
    scs.has_previous_desired_speed = true;
    scs.previous_desired_speed = prev;
}

void reset_prev_desired_speed(void) {
    scs.has_previous_desired_speed = false;
    scs.previous_desired_speed = 0;
}

void set_target_speed(vehicleSpeed speed) {
    assert((speed >= speed_min) && (speed <= speed_max));
    scs.target_speed = speed;
}

void set_current_speed(vehicleSpeed current) {
    assert((current >= speed_min) && (current <= speed_max));
    scs.current_speed = current;
}

//
// Acceleration
//

void set_acceleration(vehicleAcceleration acc) {
    assert((acc >= VEHICLE_MAX_DECELERATION) && (acc <= VEHICLE_MAX_ACCELERATION));
    scs.acceleration = acc;
}

//
// Lever handling.
//

void set_lever(SCSLever pos) {
    assert(pos != scs.lever_pos); // TODO: Do we want this assertion?
    scs.lever_prev_pos = scs.lever_pos;
    scs.lever_pos = pos;
    if (pos == scs_Neutral) {
        scs.lever_release_processed = false;
    }
}

void set_lever_last_tic(size_t time) {
    scs.lever_last_tic = time;
}

void set_lever_continuous(bool continuous) {
    scs.lever_continuous = continuous;
}

void set_lever_release(bool processed) {
    assert(processed); // Should actually never be called to set it to false, should it?
    scs.lever_release_processed = processed;
}

void lever_up5_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.desired_speed;
        if (desired < speed_max) {
            set_desired_speed(desired + (vehicleSpeed)10);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_desired_speed(curr);
    }
}

static inline vehicleSpeed getTensPlace(vehicleSpeed s) {
    // Remove the last two digits (0-9kmh).
    return (s / (vehicleSpeed)100) * (vehicleSpeed)100;
}

void lever_up7_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.desired_speed;
        if (desired < speed_max) {
            vehicleSpeed next = getTensPlace(desired) + (vehicleSpeed)100;
            set_desired_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_desired_speed(curr);
    }
}

void lever_down5_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.desired_speed;
        if (desired > speed_min) {
            set_desired_speed(desired - (vehicleSpeed)10);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_desired_speed(curr);
    }
}

void lever_down7_step(void) {
    if (scs.cruise_control_active) {
        vehicleSpeed desired = scs.desired_speed;
        if (desired > speed_min) {
            vehicleSpeed next = getTensPlace(desired);
            if (desired == next) {
                next -= (vehicleSpeed)100;
            }
            set_desired_speed(next);
        }
    } else {
        vehicleSpeed curr = scs.current_speed;
        set_desired_speed(curr);
    }
}

//
// Safety distance.
//
void set_vehicle_speed_infront(vehicleSpeed speed) {
    assert((speed >= speed_min) && (speed <= speed_max));
    scs.vehicle_speed_infront = speed;
}

void set_vehicle_acceleration_infront(vehicleAcceleration acceleration) {
    scs.vehicle_acceleration_infront = acceleration;
}

void set_safety_distance(size_t meters) {
    assert(meters >= (size_t)0);

    scs.safety_dist = meters;
}

void set_safety_distance_time(safetyDistance distance) {
    scs.safety_dist_time = distance;
}

void start_acoustic_signal(size_t start_time) {
    scs.acoustic_warning.is_on = true;
    scs.acoustic_warning.started_playing = true;
    scs.acoustic_warning.playing_sound = true;
    scs.acoustic_warning.start_time = start_time;
}

void reset_acoustic_signal(void) {
    scs.acoustic_warning.is_on = false;
    scs.acoustic_warning.started_playing = false;
    scs.acoustic_warning.playing_sound = false;
    scs.acoustic_warning.start_time = 0;
}

//
// Brake assistant.
//

void brake_assistant_available(bool available) {
    scs.brake_assistant_available = available;
}

void start_acoustic_brake_warning(size_t start_time) {
    start_acoustic_signal(start_time);
    scs.brake_warning_playing = true;
}

void reset_acoustic_brake_warning() {
    scs.acoustic_warning.is_on = false;
    scs.acoustic_warning.started_playing = false;
    scs.acoustic_warning.playing_sound = false;
    scs.acoustic_warning.start_time = 0;

    scs.brake_warning_playing = false;
}

//
// Actuators
//

void set_vehicle_speed(vehicleSpeed target) {
    assert((target >= speed_min) && (target <= speed_max));

    // SCS-12: setVehicleSpeed = 0 => no speed to maintain.
    if (target == speed_min) {
        set_cruise_control(false);
    }
}

void visual_warning(bool on) {
    scs.visual_warning_on = on;
}

void acoustic_warning(bool on) {
    acousticSignal *warning = &scs.acoustic_warning;
    assert(warning->is_on && warning->started_playing);

    warning->playing_sound = on;
}

void brake_pressure(percentage p) {
    scs.brake_pressure = p;
}

bool scs_state_equals(scs_state s1, scs_state s2){
    return
    s1.cruise_control_active == s2.cruise_control_active &&
    s1.desired_speed == s2.desired_speed &&
    s1.has_previous_desired_speed == s2.has_previous_desired_speed &&
    s1.previous_desired_speed == s2.previous_desired_speed &&
    s1.target_speed == s2.target_speed &&
    s1.current_speed == s2.current_speed &&
    s1.acceleration == s2.acceleration &&
    s1.lever_pos == s2.lever_pos &&
    s1.lever_prev_pos == s2.lever_prev_pos &&
    s1.lever_last_tic == s2.lever_last_tic &&
    s1.lever_release_processed == s2.lever_release_processed &&
    s1.lever_continuous == s2.lever_continuous &&
    s1.vehicle_speed_infront == s2.vehicle_speed_infront &&
    s1.vehicle_acceleration_infront == s2.vehicle_acceleration_infront &&
    s1.safety_dist == s2.safety_dist &&
    s1.safety_dist_time == s2.safety_dist_time &&
    s1.visual_warning_on == s2.visual_warning_on &&

    s1.acoustic_warning.is_on == s2.acoustic_warning.is_on &&
    s1.acoustic_warning.start_time == s2.acoustic_warning.start_time &&
    s1.acoustic_warning.playing_sound == s2.acoustic_warning.playing_sound &&
    s1.acoustic_warning.started_playing == s2.acoustic_warning.started_playing &&

    s1.brake_assistant_available == s2.brake_assistant_available &&
    s1.brake_pressure == s2.brake_pressure &&
    s1.brake_warning_playing == s2.brake_warning_playing;
}