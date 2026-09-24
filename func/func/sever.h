#ifndef __SEVER_H
#define __SEVER_H

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "gpio.h"



void SR_Init(void);//舵机初始化

uint8_t SR_SetAngle(uint8_t num, float angle);//舵机角度控制

void sever_setmode(uint8_t mode);



#endif

