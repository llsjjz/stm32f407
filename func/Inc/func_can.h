#ifndef __FUNC_CAN_H__
#define __FUNC_CAN_H__

#include "main.h"
#include "can.h"
#include "gpio.h"

typedef struct {          
    uint16_t speed_rads;    //0-4095=>-Vmax-Vmax
    uint16_t position_rad;  //0-65535=>-Pmax-Pmax
    uint16_t moment_Nm;     //0-4095=>-Tmax-Tmax
    uint8_t err;            //0-E
    int16_t T_Mos_dc;       //°C
    int16_t T_Rotor_dc;     //°C   
} Motor_Measure_t;

extern Motor_Measure_t motor_info[4];

void func_can_init(void);
void func_can_transmit(int16_t current1, int16_t current2, int16_t current3, int16_t current4);

#endif
