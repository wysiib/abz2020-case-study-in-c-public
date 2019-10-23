#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <google/cmockery.h>

#include <string.h>

#include "light/user-interface.h"
#include "light/light-state.h"
#include "light/light-impl.h"
#include "light/sensors.h"

#include "cruise-control/user-interface.h"

brightness get_brightness(void) {
    return (brightness) mock();
}

size_t get_time(void) {
    return (size_t) mock();
}

// A test case that does nothing and succeeds.
void sequence1(void **state) {
    light_state ls, ref;

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // TODO: mock?
    // ignition: key inserted
    will_return(get_brightness, 500);
    will_return(get_time, 1);
    light_do_step();

    // ignition on
    will_return(get_brightness, 500);
    will_return(get_time, 2);
    light_do_step();

    set_light_rotary_switch(lrs_auto);

    // sensor: light outside
    will_return(get_brightness, 500);
    will_return(get_time, 3);
    light_do_step();

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // tunnel: value at border
    will_return(get_brightness, 200);
    will_return(get_time, 4);
    light_do_step();
    
    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // brightness value below threshold
    will_return(get_brightness, 199);
    will_return(get_time, 5);
    light_do_step();

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // brightness exceeds value but no three seconds time
    will_return(get_brightness, 251);
    will_return(get_time, 6);
    light_do_step();

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // below threshold
    will_return(get_brightness, 199);
    will_return(get_time, 7);
    light_do_step();

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));

    // below threshold
    will_return(get_brightness, 251);
    will_return(get_time, 8);
    light_do_step();

    ls = get_light_state();
    ref = (light_state) {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert_true(0 == memcmp(&ls, &ref, sizeof(light_state)));
}

int main(int argc, char* argv[]) {
    const UnitTest tests[] = {
        unit_test(sequence1),
    };
    return run_tests(tests);
}
