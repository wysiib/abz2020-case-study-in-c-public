#include "sensors.h"
#include "user-interface.h"
#include "actuators.h"

static size_t when_light_on = 0;

void light_do_step(void) {
    // TODO: read all sensors
    brightness bb = get_brightness();
    size_t tt = get_time();

    if (get_light_rotary_switch() == lrs_auto) {
       if (bb < 200 && when_light_on == 0) {
           when_light_on = tt;
           set_low_beam_left(100);
           set_low_beam_right(100);
           set_tail_lamp_left(100);
           set_tail_lamp_right(100);
       }
    }

    if (bb >= 250 && tt - when_light_on >= 3) { // TODO: check value 250?
       set_low_beam_left(0);
       set_low_beam_right(0);
       set_tail_lamp_left(0);
       set_tail_lamp_right(0);
       when_light_on = 0;
   }
}
