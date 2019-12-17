#include <stdbool.h>

#include <pthread.h>

#include "light/user-interface.h"
#include "light/sensors.h"
#include "light/actuators.h"
#include "light/light-state.h"
#include "light/light-impl.h"

#include "cruise-control/user-interface.h"
#include "cruise-control/sensors.h"
#include "cruise-control/actuators.h"
#include "cruise-control/scs-impl.h"

#include "system.h"

/* TODO: sensors are not implemented; link your implementation as you please! */

void *scs_loop(void *arg) {
    (void) arg;
    scs_do_step();
    return NULL;
}

void *els_loop(void *arg) {
    (void) arg;
    light_do_step();
    return NULL;
}


int main(int argc, char *argv[]) {
    // TODO: get parameters from arguments in order to initialize the car
    (void) argc; (void) argv;

    init_system(leftHand, false, EU,false,false);

    pthread_t els_thread;
    pthread_t scs_thread;

    pthread_create(&els_thread, NULL, els_loop, NULL);
    pthread_create(&scs_thread, NULL, scs_loop, NULL);



    return 0;
}
