#ifndef LIGHT_LIGHT_IMPL_H_INCLUDED
#define LIGHT_LIGHT_IMPL_H_INCLUDED

typedef enum {blink_left, blink_right} blinkingDirection;


void reset(void **state);
//void reset(void);

void light_do_step(void);

#endif
