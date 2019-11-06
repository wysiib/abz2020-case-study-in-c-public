#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <google/cmockery.h>

#include "cruise-control/scs-impl.h"

#include "test_common.h"

/** "After [engine] start, there is no previous desired speed." */
void scs1(void **state) {
    fail();
}

int main(int argc, char *argv[]) {
    // please please remember to reset state
    const UnitTest tests[] = {
        unit_test_setup_teardown(scs1, reset, reset),
        // TODO: SCS-1
        // TODO: SCS-2
        // TODO: SCS-3
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
