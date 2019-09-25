#ifndef __COMMON_COMMON_H_INCLUDED
#define __COMMON_COMMON_H_INCLUDED

typedef enum {
    pedal_deflection_min = 0,
    // resolution: 0.2 degree
    pedal_deflection_max = 225 // 45 degree
} Pedal;

typedef enum {
    Ready,
    Dirty,
    NotReady
} sensorState;

typedef enum {NoKeyInserted, KeyInserted, KeyInIgnitionOnPosition} keyState;


keyState get_key_status(void);

bool get_engine_status(void);

Pedal get_brake_pedal_deflection(void);


#endif
