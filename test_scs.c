#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
// Must come after setjmp.h, stdarg.h, stddef.h
#include <google/cmockery.h>

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

/* SCS-1: "After [engine] start, there is no previous desired speed." */
void scs1(void **state) {
    init_system(leftHand, false, EU); // TODO: Other settings?
    sensors_and_time sensor_states = {0};

    assert_true(
        !get_scs_state().has_previous_desired_speed); // No pds after init.

    // Start engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);

    mock_and_execute(sensor_states);

    assert_true(!get_scs_state().has_previous_desired_speed);
}

void scs1_engine_restart(void **state) {
    init_system(leftHand, false, EU); // TODO: Other settings?
    sensors_and_time sensor_states = {0};

    assert_true(
        !get_scs_state().has_previous_desired_speed); // No pds after init.

    // Start engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);
    // TODO: This is a very artifical tests by direct state change.
    // Once more functionality is implemented regarding the speed control
    // system, a more complex test should be set up as well.
    set_prev_desired_speed(1234); // NOTE: Ensure a change in pds.
    assert_true(
        get_scs_state().has_previous_desired_speed); // No pds after init.

    // Stop engine.
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states = update_sensors(sensor_states, sensorKeyState, KeyInserted);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 0);
    mock_and_execute(sensor_states);

    // Start engine again.
    sensor_states = update_sensors(sensor_states, sensorTime, 2000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);
    assert_true(
        !get_scs_state().has_previous_desired_speed); // No pds after init.
}

/*
    SCS-2:
    When pulling the cruise control lever to 1, the desired speed is either
    the current vehicle speed (if there is no previous desired speed) or
    the previous desired speed (if already set).
*/

void scs2_no_prev_speed(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    mock_and_execute(sensor_states);

    assert_true(!get_scs_state().has_previous_desired_speed);

    const vehicleSpeed spe = 800;
    sensor_states = update_sensors(sensor_states, sensorTime, 1001);
    sensor_states = update_sensors(sensor_states, sensorSpeed, spe);
    mock_and_execute(sensor_states);
    lever_forward();

    scs_state scs = get_scs_state();
    assert_true(scs.has_previous_desired_speed);
    assert_int_equal(scs.previous_desired_speed, spe);
}

void scs2_with_prev_speed(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed pre = 500;
    set_prev_desired_speed(pre);

    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    mock_and_execute(sensor_states);

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().previous_desired_speed, pre);

    const vehicleSpeed spe = 800;
    sensor_states = update_sensors(sensor_states, sensorTime, 1001);
    sensor_states = update_sensors(sensor_states, sensorSpeed, spe);
    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().previous_desired_speed, pre);
}

/*
    SCS-3: If the current vehicle speed is below 20km/h and there is no previous
    desired speed, then pulling the cruise control lever to 1 does not
    activate the (adaptive) cruise control.
*/

void scs3_at_20kmh(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 200;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorSpeed, spe);

    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().previous_desired_speed, spe);
}

void scs3_below_20kmh(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 190;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorSpeed, spe);

    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(!get_scs_state().has_previous_desired_speed);
}

void scs3_below_20kmh_with_prev_desired_speed(void **state) {
    init_system(leftHand, false, EU);
    sensors_and_time sensor_states = {0};

    const vehicleSpeed spe = 190;
    sensor_states = update_sensors(sensor_states, sensorTime, 1000);
    sensor_states =
        update_sensors(sensor_states, sensorKeyState, KeyInIgnitionOnPosition);
    sensor_states = update_sensors(sensor_states, sensorEngineOn, 1);
    sensor_states = update_sensors(sensor_states, sensorSpeed, spe);

    const vehicleSpeed pre = 300;
    set_prev_desired_speed(pre);
    mock_and_execute(sensor_states);
    lever_forward();

    assert_true(get_scs_state().has_previous_desired_speed);
    assert_int_equal(get_scs_state().previous_desired_speed, pre);
}

int main(int argc, char *argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        // SCS-1
        unit_test_setup_teardown(scs1, reset, reset),
        unit_test_setup_teardown(scs1_engine_restart, reset, reset),
        // SCS-2
        unit_test_setup_teardown(scs2_no_prev_speed, reset, reset),
        unit_test_setup_teardown(scs2_with_prev_speed, reset, reset),
        // SCS-3
        unit_test_setup_teardown(scs3_at_20kmh, reset, reset),
        unit_test_setup_teardown(scs3_below_20kmh, reset, reset),
        unit_test_setup_teardown(scs3_below_20kmh_with_prev_desired_speed,
                                 reset, reset),
        // TODO: SCS-4
        // TODO: SCS-5
        // TODO: SCS-6
        // TODO: SCS-7
        // TODO: SCS-8
        // TODO: SCS-9
        // TODO: SCS-10
        // TODO: SCS-11
        // TODO: SCS-12
        // TODO: SCS-13
        // TODO: SCS-14
        // TODO: SCS-15
        // TODO: SCS-16
        // TODO: SCS-17
        // TODO: SCS-18
        // TODO: SCS-19
        // TODO: SCS-20
        // TODO: SCS-21
        // TODO: SCS-22
        // TODO: SCS-23
        // TODO: SCS-24
        // TODO: SCS-25
        // TODO: SCS-26
        // TODO: SCS-27
        // TODO: SCS-28
        // TODO: SCS-29
        // TODO: SCS-30
        // TODO: SCS-31
        // TODO: SCS-32
        // TODO: SCS-33
        // TODO: SCS-34
        // TODO: SCS-35
        // TODO: SCS-36
        // TODO: SCS-37
        // TODO: SCS-38
        // TODO: SCS-39
        // TODO: SCS-40
        // TODO: SCS-41
        // TODO: SCS-42
        // TODO: SCS-43
    };
    run_tests(tests);
    return 0;
}
