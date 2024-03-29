#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
// Must come after setjmp.h, stdarg.h, stddef.h
#include <google/cmockery.h>

#include "cruise-control/actuators.h"
#include "cruise-control/scs-impl.h"
#include "cruise-control/scs-state.h"
#include "cruise-control/user-interface.h"

#include "system.h"
#include "test_common.h"

void light_do_step(void) {
    // FIXME: The light step function is a dependency for mock_and_execute in
    // test_common.c, but linking to light-impl.c adds a big bunch of further
    // light-dependencies to the SCS, which currently should not be in here.
}

/**
 * Sets the given time as system time to the sensors,
 * then mocks and executes the sensors.
 */
void begin_sensor_time_and_exec(sensors_and_time *sensors, size_t time) {
    *sensors = update_sensors(*sensors, sensorTime, time);
    mock_and_execute(*sensors);
}

/**
 * Advances the time the given amount of milliseconds.
 * The step_size serves as time granularity.
 * Each time step, the sensors will be mocked and executed.
 *
 * TODO: Move to text_common.c
 */
void sensors_advance_time_and_exec(sensors_and_time *state,
                                   size_t duration_ms, size_t step_size) {
    size_t target_time = state->time + duration_ms;
    assert_true(target_time > state->time); // Guard against overflow.

    for (size_t time = state->time;
         time < target_time;
         time += step_size) {
        *state = update_sensors(*state, sensorTime, time);
        mock_and_execute(*state);
    }
    // Run last time to ensure target time is reached.
    *state = update_sensors(*state, sensorTime, target_time);
    mock_and_execute(*state);
}

sensors_and_time start_engine(sensors_and_time sensor_states) {
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    return sensor_states;
}

sensors_and_time stop_engine(sensors_and_time sensor_states) {
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);

    return sensor_states;
}

sensors_and_time start_engine_and_drive(sensors_and_time sensor_states,
                                        vehicleSpeed speed) {
    sensor_states = start_engine(sensor_states);
    return update_sensors(sensor_states, sensorCurrentSpeed, speed);
}

/* SCS-1: "After [engine] start, there is no previous desired speed." */
void scs1(void **state) {
    init_system(leftHand, false, EU, false, false); // TODO: Other settings?
    sensors_and_time sensor_states = {0};

    assert_true(!get_scs_state().has_previous_desired_speed); // No pds after init.

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine(sensor_states);
    mock_and_execute(sensor_states);

    assert_true(!get_scs_state().has_previous_desired_speed);
}

void scs1_engine_restart(void **state) {
    init_system(leftHand, false, EU, false, false); // TODO: Other settings?
    sensors_and_time sensor_states = {0};

    assert_true(!get_scs_state().has_previous_desired_speed); // No pds after init.

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine(sensor_states);
    mock_and_execute(sensor_states);

    set_prev_desired_speed(1234); // NOTE: Ensure a change in pds.
    assert_true(get_scs_state().has_previous_desired_speed);

    // Stop engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = stop_engine(sensor_states);
    mock_and_execute(sensor_states);

    // Start engine again.
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = start_engine(sensor_states);
    mock_and_execute(sensor_states);
    assert_true(!get_scs_state().has_previous_desired_speed);
}

void scs1_engine_shutdown(void **state) {
    init_system(leftHand, false, EU, false, false); // TODO: Other settings?
    sensors_and_time sensor_states = {0};

    assert_true(!get_scs_state().has_previous_desired_speed); // No pds after init.

    // Start engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine(sensor_states);
    mock_and_execute(sensor_states);
    set_prev_desired_speed(1234); // NOTE: Ensure a change in pds.
    set_cruise_control(true); // Activate
    assert_true(get_scs_state().has_previous_desired_speed);
    assert_true(get_scs_state().cruise_control_active);

    // Stop engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = stop_engine(sensor_states);
    mock_and_execute(sensor_states);

    assert_true(!get_scs_state().cruise_control_active); // No pds after init.
}

/*
    SCS-2:
    When pulling the cruise control lever to 1, the desired speed is either
    the current vehicle speed (if there is no previous desired speed) or
    the previous desired speed (if already set).
*/

void scs2_no_prev_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    mock_and_execute(sensor_states);

    assert_true(!get_scs_state().has_previous_desired_speed);

    const vehicleSpeed spe = 800;
    sensor_states = update_sensors(sensor_states, sensorTime, 1001);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, spe);
    mock_and_execute(sensor_states);
    lever_forward();

    scs_state scs = get_scs_state();
    assert_true(get_scs_state().cruise_control_active);
    assert_int_equal(scs.desired_speed, spe);
}

void scs2_with_prev_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed pre = 500;
    set_prev_desired_speed(pre);

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine(sensor_states);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().previous_desired_speed, pre);

    const vehicleSpeed spe = 800;
    sensor_states = update_sensors(sensor_states, sensorTime, 1001);
    sensor_states = update_sensors(sensor_states, sensorCurrentSpeed, spe);
    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(get_scs_state().cruise_control_active);
    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().desired_speed, pre);
}

/*
    SCS-3: If the current vehicle speed is below 20km/h and there is no previous
    desired speed, then pulling the cruise control lever to 1 does not
    activate the (adaptive) cruise control.
*/

void scs3_at_20kmh(void **state) { // TODO: Is this test necessary?
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 200;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, spe);
    mock_and_execute(sensor_states);

    lever_forward();

    assert_true(get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed, spe);
}

void scs3_below_20kmh(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 190;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, spe);
    mock_and_execute(sensor_states);

    lever_forward();

    assert_true(!get_scs_state().cruise_control_active);
    assert_true(!get_scs_state().has_previous_desired_speed);
}

void scs3_below_20kmh_with_prev_desired_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 190;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, spe);

    const vehicleSpeed pre = 300;
    set_prev_desired_speed(pre);
    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_true(get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed, pre);
}

/**
    SCS-4: If the driver pushes the cruise control lever to 2 up to the first
    resistance level (5°) and the (adaptive) cruise control is activated,
    the desired speed is increased by 1 km/h.
 */

void scs4_active_cc(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_desired_speed(desired);
    set_cruise_control(true);
    mock_and_execute(sensor_states);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, desired + 10);
}

void scs4_active_cc_twice(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_desired_speed(desired);
    set_cruise_control(true);
    mock_and_execute(sensor_states);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, desired + 20);
}

void scs4_active_cc_max_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = speed_max;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_desired_speed(desired);
    set_cruise_control(true);
    mock_and_execute(sensor_states);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_max);
}

/*
    SCS-5: If the driver pushes the cruise control lever to 2 above the first
    resistance level (7°, beyond the pressure point) and the (adaptive)
    cruise control is activated, the desired speed is increased to the next
    ten’s place.
 */

void scs5_active_cc(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 310;
    vehicleSpeed nextTensValue = 400;

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs5_active_cc_twice(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 310;
    vehicleSpeed nextTensValue = 500;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs5_active_cc_already_multiple_of_ten(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    vehicleSpeed nextTensValue = 400;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs5_active_cc_almost_max_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = speed_max - 2;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_max);
}

void scs5_active_cc_max_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(speed_max);
    set_cruise_control(true);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_max);
}

/*
    SCS-6: Pushing the cruise control lever to 3 reduces the desired speed
    accordingly to Req. SCS-4 and Req. SCS-5.
 */

void scs6_down5_active_cc(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, desired - 10);
}

void scs6_down5_active_cc_twice(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, desired - 20);
}

void scs6_down5_active_cc_min_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = speed_min;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_min);
}

void scs6_down7_active_cc(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 310;
    vehicleSpeed nextTensValue = 300;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs6_down7_active_cc_twice(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 310;
    vehicleSpeed nextTensValue = 200;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs6_down7_active_cc_already_multiple_of_ten(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = 300;
    vehicleSpeed nextTensValue = 200;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, nextTensValue);
}

void scs6_down7_active_cc_almost_min_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    vehicleSpeed desired = speed_min + 2;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(desired);
    set_cruise_control(true);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_min);
}

void scs6_down7_active_cc_min_speed(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 330);
    set_desired_speed(speed_min);
    set_cruise_control(true);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_int_equal(get_scs_state().desired_speed, speed_min);
}

/*
    SCS-7: If the driver pushes the cruise control lever to 2 with activated
    cruise control within the first resistance level (5°, not beyond the
    pressure point) and holds it there for 2 seconds, the target speed of
    the cruise control is increased every second by 1 km/h until the lever
    is in neutral position again.
    Example: Current speed is 57 km/h -> after holding 2 seconds,
    desired speed is set to 58 km/h, after holding 3 seconds, desired
    speed is set to 59 km/h, after holding 4 seconds, desired speed is set
    to 60 km/h, etc.
 */

void scs7_example(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(570);
    set_cruise_control(true);
    lever_up5();

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 570);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().desired_speed == 570);

    const size_t granularity = 50; // Time step granularity in ms.

    // One second passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 570);

    // Two seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 580);

    // Three seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 590);

    // Four seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 600);

    lever_release();

    // Five seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 600);

    // Six seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 600);
}

/*
    SCS-8: If the driver pushes the cruise control lever to 2 with activated cruise
    control through the first resistance level (7°, beyond the pressure
    point) and holds it there for 2 seconds, the speed set point of the
    cruise control is increased every 2 seconds to the next ten’s place
    until the lever is in neutral position again.
    Example: Current speed is 57 km/h −→ after holding 2 seconds,
    desired speed is set to 60 km/h, after holding 4 seconds, desired
    speed is set to 70 km/h, after holding 6 seconds, desired speed is set
    to 80 km/h, etc.
*/

void scs8_example(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(570);
    set_cruise_control(true);
    lever_up7();

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 570);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().desired_speed == 570);

    const size_t granularity = 50; // Time step granularity in ms.

    // One second passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 570);

    // Two seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 600);

    // Three seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 600);

    // Four seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 700);

    lever_release();

    // Five seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 700);

    // Six seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 700);
}

/*
    SCS-9: If the driver pushes the cruise control lever to 3 with activated cruise
    control within the first resistance level (5°, not beyond the pressure
    point) and holds it there for 2 seconds, the target speed of the cruise
    control is reduced every second by 1 km/h until the lever is in neutral
    position again.
    Example: Current speed is 57 km/h −→ after holding 2 seconds,
    desired speed is set to 56 km/h, after holding 3 seconds, desired
    speed is set to 55 km/h, after holding 4 seconds, desired speed is set
    to 54 km/h, etc.
*/

void scs9_example(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(570);
    set_cruise_control(true);
    lever_down5();

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 570);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().desired_speed == 570);

    const size_t granularity = 50; // Time step granularity in ms.

    // One second passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 570);

    // Two seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 560);

    // Three seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 550);

    // Four seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 540);

    lever_release();

    // Five seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 540);

    // Six seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 540);
}

/*
    SCS-10: If the driver pushes the cruise control lever to 3 with activated
    cruise control through the first resistance level (7°, beyond the pressure
    point) and holds it there for 2 seconds, the speed set point of the
    cruise control is increased every 2 seconds to the next ten’s place
    until the lever is in neutral position again.
 */

void scs10_example(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(570);
    set_cruise_control(true);
    lever_down7();

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 570);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().desired_speed == 570);

    const size_t granularity = 50; // Time step granularity in ms.

    // One second passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 570);

    // Two seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 500);

    // Three seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 500);

    // Four seconds passed.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 400);

    lever_release();

    // Five seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 400);

    // Six seconds passed, but lever was released already.
    sensors_advance_time_and_exec(&sensor_states, 1000, granularity);
    assert_true(get_scs_state().desired_speed == 400);
}

/*
    SCS-11: If the (adaptive) cruise control is deactivated and the cruise
    control lever is moved up or down (either to the first or above the first
    resistance level, the current vehicle speed is used as desired speed.
 */

void scs11_up5(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_cruise_control(false);
    reset_prev_desired_speed();
    mock_and_execute(sensor_states);
    assert_true(!get_scs_state().has_previous_desired_speed);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_true(!get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed,
                     400);
}

void scs11_up7(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_cruise_control(false);
    reset_prev_desired_speed();

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_true(!get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed,
                     400);
}

void scs11_down5(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_cruise_control(false);
    reset_prev_desired_speed();

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_true(!get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed,
                     400);
}

void scs11_down7(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    set_cruise_control(false);
    reset_prev_desired_speed();

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 1, 1);

    assert_true(!get_scs_state().cruise_control_active);
    assert_int_equal(get_scs_state().desired_speed,
                     400);
}

/*
    SCS-12: Pressing the cruise control lever to 4 deactivates the (adaptive)
    cruise control. setVehicleSpeed = 0 indicates to the car that there
    is no speed to maintain.
 */

void scs12_lever(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(400);
    set_cruise_control(true);

    lever_backward();

    assert_true(!get_scs_state().cruise_control_active);
}

void scs12_speed_to_zero(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(400);
    set_cruise_control(true);

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states = start_engine_and_drive(sensor_states, 1000); // 100 km/h
    mock_and_execute(sensor_states);

    set_vehicle_speed(0);

    assert_true(!get_scs_state().cruise_control_active);
}

/*
    SCS-13: The cruise control is activated using the cruise control lever
    according to Reqs. SCS-1 to SCS-12.
*/

void scs13_forward(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_forward(); // TODO: Register in scs_do_step?

    assert_true(get_scs_state().cruise_control_active == true);
}

void scs13_forward_below20(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 190);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_forward(); // TODO: Register in scs_do_step?

    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_up5(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_up7(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_down5(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_down7(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_long_up5(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_up5();
    sensors_advance_time_and_exec(&sensor_states, 5000, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_long_up7(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_up7();
    sensors_advance_time_and_exec(&sensor_states, 5000, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_long_down5(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_down5();
    sensors_advance_time_and_exec(&sensor_states, 5000, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

void scs13_long_down7(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_down7();
    sensors_advance_time_and_exec(&sensor_states, 5000, 50);
    lever_release();
    sensors_advance_time_and_exec(&sensor_states, 500, 50);

    // This should not activate the cruise control.
    assert_true(get_scs_state().cruise_control_active == false);
}

/*
    SCS-16: By pushing the brake, the cruise control is deactivated until it is
    activated again.
 */
void scs16_brake(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_desired_speed(400);
    set_cruise_control(true);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    mock_and_execute(sensor_states);

    brakePedal(40);

    assert_true(get_scs_state().cruise_control_active == false);
}

/*
    SCS-17: By pushing the control lever backwards, the cruise control is
    deactivated until it is activated again.
 */

void scs17_backward_inactive(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_backward();

    assert_true(get_scs_state().cruise_control_active == false);
}

void scs17_backward_active(void **State) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 400);
    begin_sensor_time_and_exec(&sensor_states, 1000);

    lever_forward();
    sensors_advance_time_and_exec(&sensor_states, 3000, 50);
    lever_backward();

    assert_true(get_scs_state().cruise_control_active == false);
}

/*
    SCS-20: If the distance to the vehicle ahead falls below the specified
    speed-dependent safety distance (see Req. SCS-24), the vehicle brakes
    automatically. The maximum deceleration is 5m/s².
*/

void scs20_collision_ahead(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 30);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    mock_and_execute(sensor_states);

    assert_in_range(get_scs_state().acceleration,
                    -VEHICLE_MAX_DECELERATION,
                    -1);
}

void scs20_collision_ahead_non_adaptive(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(simple);
    set_safety_distance_time(three_secs);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 30);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().acceleration, 0);
}

void scs20_no_collision_ahead(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 120);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 400);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().acceleration, 0);
}

/*
    SCS-21: If the maximum deceleration of 5m/s 2 is insufficient to prevent a
    collision with the vehicle ahead, the vehicle warns the driver by two
    acoustical signals (0.1 seconds long with 0.2 seconds pause between)
    and by this demands to intervene.
*/

void scs21_insufficient_deceleration(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250); // TODO: incorporate this
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 1);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);

    scs_state scs;

    // Play sound for 0.1 sec.
    for (size_t time = 1000;
         time < 1100;
         ++time) {
        sensor_states = update_sensors(sensor_states, sensorTime, time);
        mock_and_execute(sensor_states);
        scs = get_scs_state();
        assert_true(scs.acoustic_warning.is_on);
        assert_true(scs.acoustic_warning.playing_sound);
    }

    // Pause sound for 0.2 sec.
    for (size_t time = 1100;
         time < 1300;
         ++time) {
        sensor_states = update_sensors(sensor_states, sensorTime, time);
        mock_and_execute(sensor_states);
        scs = get_scs_state();
        assert_true(scs.acoustic_warning.is_on);
        assert_true(!(scs.acoustic_warning.playing_sound));
    }

    // Play sound again for 0.1 sec.
    for (size_t time = 1300;
         time < 1400;
         ++time) {
        sensor_states = update_sensors(sensor_states, sensorTime, time);
        mock_and_execute(sensor_states);
        scs = get_scs_state();
        assert_true(scs.acoustic_warning.is_on);
        assert_true(scs.acoustic_warning.playing_sound);
    }

    // No warning anymore after it played.
    sensor_states = update_sensors(sensor_states, sensorTime, 1400);
    mock_and_execute(sensor_states);
    scs = get_scs_state();

    assert_true(!(scs.acoustic_warning.is_on));
    assert_true(!(scs.acoustic_warning.playing_sound));
}

void scs21_sufficient_deceleration(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250); // TODO: incorporate this
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 90);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    scs_state scs = get_scs_state();

    assert_true(!(scs.acoustic_warning.is_on));
    assert_true(!(scs.acoustic_warning.playing_sound));
}

/*
    SCS-22: If the distance to the preceding vehicle increases again above the
    speed-dependent safety distance, the vehicle accelerates with a
    maximum of 2m/s 2 until the set speed is reached.

    Example: Figure 10 shows an exemplary situation with a desired
    speed of 120 km/h. At the beginning, the car drives at this speed
    until another car appears with 80 km/h. The adaptive cruise control
    decelerates to 80 km/h with a maximum deceleration of 5m/s 2 . If
    this is not sufficient, two acoustical signals warn the driver. As soon
    as the vehicle in front accelerates to 100 km/h, the adaptive cruise
    control also accelerates with a maximum of 2m/s 2 . When the vehicle
    in front finally accelerates to a speed of more than 120 km/h the
    adaptive cruise control increases the speed back to 120 km/h.
 */
void scs22_example(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_desired_speed(1200);
    set_safety_distance_time(three_secs); // At 12 km/h corresponds to 100 m.
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 0);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1200);
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 1200);

    // 80 kmh car incoming, not yet within safety distance.
    set_vehicle_speed_infront(800);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 120);
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 1200);

    // 80 kmh car incoming, now within safety distance
    set_vehicle_speed_infront(800);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 90);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 800);
    assert_int_equal(get_scs_state().desired_speed, 1200);

    // 80 kmh car accelerates to 100 km/h
    set_vehicle_speed_infront(1000);
    set_vehicle_acceleration_infront(2); // NOTE: Don't now if this is a fitting value.
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 1000);
    assert_int_equal(get_scs_state().desired_speed, 1200);

    // 100 kmh car accelerates to >=120 km/h
    set_vehicle_speed_infront(1300);
    set_vehicle_acceleration_infront(2); // NOTE: Don't now if this is a fitting value.
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 100);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 1200);
    assert_int_equal(get_scs_state().desired_speed, 1200);

    // 100 kmh car leaves safety distance
    set_vehicle_speed_infront(1300);
    set_vehicle_acceleration_infront(0);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 140);
    sensor_states = update_sensors(sensor_states, sensorTime, 3000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().target_speed, 1200);
    assert_int_equal(get_scs_state().desired_speed, 1200);
}
/*
    SCS-23: If the speed of the preceding vehicle decreases below 20 km/h, the
    distance is set to 2.5s * currentSpeed, down to a standstill. When
    both vehicles are standing the absolute distance is regulated to 2m.
    When the preceding vehicle is accelerating again, the distance is set
    to 3s * currentSpeed. This distance is valid until the vehicle speed
    exceeds 20 km/h, independent of the user’s input via the distance
    level (turning the cruise control lever head).
 */

void scs23_below_20kmh_two_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_secs); // Should not matter

    set_vehicle_speed_infront(190);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_two_point_five_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_point_five_secs); // Should not matter

    set_vehicle_speed_infront(190);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_three_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs); // Should not matter

    set_vehicle_speed_infront(190);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_decelerating_two_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(-2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_decelerating_two_point_five_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_point_five_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(-2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_decelerating_three_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(-2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 13);
}

void scs23_below_20kmh_accelerating_two_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 15);
}

void scs23_below_20kmh_accelerating_two_point_five_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_point_five_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 15);
}

void scs23_below_20kmh_accelerating_three_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs); // Should not matter

    set_vehicle_speed_infront(190);
    set_vehicle_acceleration_infront(2);
    sensor_states = start_engine_and_drive(sensor_states, 300);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 15);
}

/*
    SCS-24: By turning the cruise control lever head, the distance to be
    maintained to the vehicle ahead can be selected. Three levels are available:
    2 seconds, 2.5 seconds and 3 seconds. The desired level only applies
    within the velocity window > 20 km/h. Below this level, the system
    autonomously sets the distance according to Req. SCS-23.
 */

void scs24_20kmh_two_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_secs);
    set_vehicle_speed_infront(200);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 55);
}

void scs24_two_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_secs);
    set_vehicle_speed_infront(250);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 55);
}

void scs24_two_point_five_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(two_point_five_secs);
    set_vehicle_speed_infront(250);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 69);
}

void scs24_three_secs(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance_time(three_secs);
    set_vehicle_speed_infront(250);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().safety_dist, 83);
}

/*
    SCS-25: A visual warning is activated if the actual distance is less than
    (current speed/3.6) · 1.5.
 */

void scs25_distance_is_less(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 41);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().visual_warning_on);
}

void scs25_distance_is_more(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 42);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_true(!(get_scs_state().visual_warning_on));
}

/*
    SCS-26: An acoustic alarm is activated if the actual distance is less than
    (current speed/3.6) * 0.8.
*/

void scs26_distance_is_less(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 22);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().acoustic_warning.is_on);
}

void scs26_distance_is_more(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_scs_mode(adaptive);
    set_safety_distance(100);
    set_vehicle_speed_infront(250);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 23);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    sensor_states = start_engine_and_drive(sensor_states, 1000);
    mock_and_execute(sensor_states);

    assert_true(!(get_scs_state().acoustic_warning.is_on));
}

/*
    SCS-27: The emergency brake assistant must be available in the following
    speed windows: 0 - 60 km/h, for emergency braking to stationary
    obstacles, 0 – 120 km/h on moving obstacles.
 */

void scs27_stationary_below_60kmh(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_vehicle_speed_infront(0); // Stationary

    // 59 km/h
    sensor_states = start_engine_and_drive(sensor_states, 590);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 50);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().brake_assistant_available);

    // 60 km/h
    sensor_states = start_engine_and_drive(sensor_states, 600);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 50);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().brake_assistant_available);
}

void scs27_moving_below_120kmh(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    set_vehicle_speed_infront(800); // 80 km/h

    // 119 km/h
    sensor_states = start_engine_and_drive(sensor_states, 1190);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 90);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().brake_assistant_available);

    // 120 km/h
    sensor_states = start_engine_and_drive(sensor_states, 1200);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 90);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().brake_assistant_available);
}

/*
    SCS-28: The time necessary to perform braking to standstill is determined by
    the value for the maximum deceleration. If an object is ahead of the
    vehicle and the time until an impact is less or equal to the time until a
    standstill plus 3 seconds, three acoustic signals are given (0.1 seconds
    long with 0.05 seconds pause between) is issued and the brakes are
    activated by 20%. If the time until an impact is less or equal to the
    time until a standstill plus 1.5 seconds, the brake is activated by 60%.
    If the time until an impact is less or equal to the time until standstill
    then the brake is activated at 100%.
 */

void scs28_plus_three_seconds_braking(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 360); // 10 m/s.

    // Assuming brake deceleration of 5 m/s²:
    //  way for standstill: 10 m
    //  seconds for standstill: 2 s
    //  seconds for standstill + 3 s: 5 s
    // Time until impact with object that is <50 m away: <5 s.
    set_vehicle_speed_infront(0);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 50);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().brake_pressure, 20);
}

void scs28_plus_one_point_five_seconds_braking(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 360); // 10 m/s.

    // Assuming brake deceleration of 5 m/s²:
    //  way for standstill: 10 m
    //  seconds for standstill: 2 s
    //  seconds for standstill + 1.5 s: 3.5 s
    // Time until impact with object that is <35 m away: <3.5 s.
    set_vehicle_speed_infront(0);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 35);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().brake_pressure, 60);
}

void scs28_plus_zero_seconds_braking(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 360); // 10 m/s.

    // Assuming brake deceleration of 5 m/s²:
    //  way for standstill: 10 m
    //  seconds for standstill: 2 s
    // Time until impact with object that is <20 m away: <2 s.
    set_vehicle_speed_infront(0);
    sensor_states = update_sensors(sensor_states, sensorRangedRadar, 20);
    sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);
    mock_and_execute(sensor_states);

    assert_int_equal(get_scs_state().brake_pressure, 100);
}

void scs28_audio_warning(void **state) {
    init_system(leftHand, false, EU, false, false);
    sensors_and_time sensor_states = {0};

    sensor_states = start_engine_and_drive(sensor_states, 360); // 10 m/s.

    /*
        FIXME: This test ensures that the audio signal described in SCS-28 is
        played accordingly. However, implementing the necessary logic to play
        the sound actually breaks the audio signal tests for SCS-21.
        This is due to both tests working on the notion of "collision while
        decelerating with 5 m/s²".

        In the current implementation, the SCS-28 audio signal is not called.
        To enable it, call `start_acoustic_brake_warning(system_time)` in the
        `handle_brake_assist(scs_state, size_t, size_t)` function in scs-impl.c.
        This will however break SCS-21.

        I am rather unsure how to satisfy both specifications or whether I maybe
        misunderstood when SCS-21 is to be considered, but as of now,
        both cases overlap.
     */

    size_t distances[] = {50, 35, 20};
    for (int i = 0; i < 3; ++i) {
        size_t dist = distances[i];

        set_vehicle_speed_infront(0);
        sensor_states = update_sensors(sensor_states, sensorRangedRadar, dist);
        sensor_states = update_sensors(sensor_states, sensorRangedRadarState, Ready);

        size_t test_granularity = 10;

        // Play sound for 0.1 seconds.
        for (size_t time = 1000;
             time < 1100;
             time += test_granularity) {
            sensor_states = update_sensors(sensor_states, sensorTime, time);
            mock_and_execute(sensor_states);

            assert_true(get_scs_state().acoustic_warning.is_on);
            assert_true(get_scs_state().acoustic_warning.playing_sound);
        }
        // Pause sound for 0.05 seconds.
        for (size_t time = 1100;
             time < 1150;
             time += test_granularity) {
            sensor_states = update_sensors(sensor_states, sensorTime, time);
            mock_and_execute(sensor_states);

            assert_true(get_scs_state().acoustic_warning.is_on);
            assert_true(!(get_scs_state().acoustic_warning.playing_sound));
        }
        // Play sound for 0.1 seconds.
        for (size_t time = 1150;
             time < 1250;
             time += test_granularity) {
            sensor_states = update_sensors(sensor_states, sensorTime, time);
            mock_and_execute(sensor_states);

            assert_true(get_scs_state().acoustic_warning.is_on);
            assert_true(get_scs_state().acoustic_warning.playing_sound);
        }
        // Pause sound for 0.05 seconds.
        for (size_t time = 1250;
             time < 1300;
             time += test_granularity) {
            sensor_states = update_sensors(sensor_states, sensorTime, time);
            mock_and_execute(sensor_states);

            assert_true(get_scs_state().acoustic_warning.is_on);
            assert_true(!(get_scs_state().acoustic_warning.playing_sound));
        }
        // Play sound for 0.1 seconds.
        for (size_t time = 1300;
             time < 1400;
             time += test_granularity) {
            sensor_states = update_sensors(sensor_states, sensorTime, time);
            mock_and_execute(sensor_states);

            assert_true(get_scs_state().acoustic_warning.is_on);
            assert_true(get_scs_state().acoustic_warning.playing_sound);
        }

        assert_true(!(get_scs_state().acoustic_warning.is_on));
        assert_true(!(get_scs_state().acoustic_warning.playing_sound));

    }
}
//
//
//

int main(int argc, char *argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        // Setting and modifying desired speed:
        // SCS-1
        unit_test_setup_teardown(scs1, reset, reset),
        unit_test_setup_teardown(scs1_engine_restart, reset, reset),
        unit_test_setup_teardown(scs1_engine_shutdown, reset, reset),
        // SCS-2
        unit_test_setup_teardown(scs2_no_prev_speed, reset, reset),
        unit_test_setup_teardown(scs2_with_prev_speed, reset, reset),
        // SCS-3
        unit_test_setup_teardown(scs3_at_20kmh, reset, reset),
        unit_test_setup_teardown(scs3_below_20kmh, reset, reset),
        unit_test_setup_teardown(scs3_below_20kmh_with_prev_desired_speed,
                                 reset, reset),
        // SCS-4
        unit_test_setup_teardown(scs4_active_cc, reset, reset),
        unit_test_setup_teardown(scs4_active_cc_twice, reset, reset),
        unit_test_setup_teardown(scs4_active_cc_max_speed, reset, reset),
        // SCS-5
        unit_test_setup_teardown(scs5_active_cc, reset, reset),
        unit_test_setup_teardown(scs5_active_cc_twice, reset, reset),
        unit_test_setup_teardown(scs5_active_cc_already_multiple_of_ten, reset,
                                 reset),
        unit_test_setup_teardown(scs5_active_cc_almost_max_speed, reset, reset),
        unit_test_setup_teardown(scs5_active_cc_max_speed, reset, reset),
        // SCS-6
        unit_test_setup_teardown(scs6_down5_active_cc, reset, reset),
        unit_test_setup_teardown(scs6_down5_active_cc_twice, reset, reset),
        unit_test_setup_teardown(scs6_down5_active_cc_min_speed, reset, reset),
        unit_test_setup_teardown(scs6_down7_active_cc, reset, reset),
        unit_test_setup_teardown(scs6_down7_active_cc_twice, reset, reset),
        unit_test_setup_teardown(scs6_down7_active_cc_already_multiple_of_ten, reset,
                                 reset),
        unit_test_setup_teardown(scs6_down7_active_cc_almost_min_speed, reset, reset),
        unit_test_setup_teardown(scs6_down7_active_cc_min_speed, reset, reset),
        // SCS-7
        unit_test_setup_teardown(scs7_example, reset, reset),
        // SCS-8
        unit_test_setup_teardown(scs8_example, reset, reset),
        // SCS-9
        unit_test_setup_teardown(scs9_example, reset, reset),
        // SCS-10
        unit_test_setup_teardown(scs10_example, reset, reset),
        // SCS-11
        unit_test_setup_teardown(scs11_up5, reset, reset),
        unit_test_setup_teardown(scs11_up7, reset, reset),
        unit_test_setup_teardown(scs11_down5, reset, reset),
        unit_test_setup_teardown(scs11_down7, reset, reset),
        // SCS-12
        unit_test_setup_teardown(scs12_lever, reset, reset),
        unit_test_setup_teardown(scs12_speed_to_zero, reset, reset),

        // Simple cruise control:
        // SCS-13
        unit_test_setup_teardown(scs13_forward, reset, reset),
        unit_test_setup_teardown(scs13_up5, reset, reset),
        unit_test_setup_teardown(scs13_up7, reset, reset),
        unit_test_setup_teardown(scs13_down5, reset, reset),
        unit_test_setup_teardown(scs13_down7, reset, reset),
        unit_test_setup_teardown(scs13_long_up5, reset, reset),
        unit_test_setup_teardown(scs13_long_up7, reset, reset),
        unit_test_setup_teardown(scs13_long_down5, reset, reset),
        unit_test_setup_teardown(scs13_long_down7, reset, reset),
        // TODO: SCS-14 --- Don't really know how to test for it.
        // TODO: SCS-15
        // SCS-16
        unit_test_setup_teardown(scs16_brake, reset, reset),
        // SCS-17
        unit_test_setup_teardown(scs17_backward_inactive, reset, reset),
        unit_test_setup_teardown(scs17_backward_active, reset, reset),

        // Adaptive cruise control:
        // TODO: SCS-18 --- Unsure if we can test for this in a simple way
        //                  besides LTL model checking on the final vehicle?
        // TODO: SCS-19 --- Implementation is invariant to cruise control state,
        //                  I guess we could repeat tests with adaptive mode on
        // SCS-20
        unit_test_setup_teardown(scs20_collision_ahead, reset, reset),
        unit_test_setup_teardown(scs20_collision_ahead_non_adaptive, reset, reset),
        unit_test_setup_teardown(scs20_no_collision_ahead, reset, reset),
        // SCS-21
        unit_test_setup_teardown(scs21_insufficient_deceleration, reset, reset),
        unit_test_setup_teardown(scs21_sufficient_deceleration, reset, reset),
        // TODO: SCS-22
        unit_test_setup_teardown(scs22_example, reset, reset),
        // SCS-23
        unit_test_setup_teardown(scs23_below_20kmh_two_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_two_point_five_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_three_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_decelerating_two_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_decelerating_two_point_five_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_decelerating_three_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_accelerating_two_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_accelerating_two_point_five_secs, reset, reset),
        unit_test_setup_teardown(scs23_below_20kmh_accelerating_three_secs, reset, reset),
        // SCS-24
        unit_test_setup_teardown(scs24_two_secs, reset, reset),
        unit_test_setup_teardown(scs24_20kmh_two_secs, reset, reset),
        unit_test_setup_teardown(scs24_two_point_five_secs, reset, reset),
        unit_test_setup_teardown(scs24_three_secs, reset, reset),

        // Distance warning:
        // SCS-25
        unit_test_setup_teardown(scs25_distance_is_less, reset, reset),
        unit_test_setup_teardown(scs25_distance_is_more, reset, reset),
        // SCS-26
        unit_test_setup_teardown(scs26_distance_is_less, reset, reset),
        unit_test_setup_teardown(scs26_distance_is_more, reset, reset),

        // Emergency Brake Assistant:
        // SCS-27
        unit_test_setup_teardown(scs27_stationary_below_60kmh, reset, reset),
        unit_test_setup_teardown(scs27_moving_below_120kmh, reset, reset),
        // SCS-28
        unit_test_setup_teardown(scs28_plus_three_seconds_braking, reset, reset),
        unit_test_setup_teardown(scs28_plus_one_point_five_seconds_braking, reset, reset),
        unit_test_setup_teardown(scs28_plus_zero_seconds_braking, reset, reset),
        unit_test_setup_teardown(scs28_audio_warning, reset, reset),

        // Speed Limit:
        // TODO: SCS-29
        // TODO: SCS-30
        // TODO: SCS-31
        // TODO: SCS-32
        // TODO: SCS-33
        // TODO: SCS-34
        // TODO: SCS-35
        // TODO: SCS-36

        // Traffic Sign Detection:
        // TODO: SCS-37
        // TODO: SCS-38
        // TODO: SCS-39

        // Fault handling and general properties:
        // TODO: SCS-40
        // TODO: SCS-41
        // TODO: SCS-42
        // TODO: SCS-43
    };
    run_tests(tests);
    return 0;
}
