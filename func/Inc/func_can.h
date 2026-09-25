#ifndef __FUNC_CAN_H__
#define __FUNC_CAN_H__

#include "main.h"
#include "can.h"
#include "gpio.h"

//**************************************************************************************************//
//CAN ID 定义

#define CAN_MASTER_ID 0x200u

#define Vmax 30.0f // rad/s
#define Pmax 12.5f // rad
#define Tmax 12.0f // Nm

//**************************************************************************************************//

typedef struct {          
    uint16_t speed_rads;    //0-4095=>-Vmax-Vmax
    uint16_t position_rad;  //0-65535=>-Pmax-Pmax
    uint16_t moment_Nm;     //0-4095=>-Tmax-Tmax
    uint8_t err;            //0-E
    int16_t T_Mos_dc;       //°C
    int16_t T_Rotor_dc;     //°C   

    float speed_rads_f;      // rad/s
    float position_rad_f;    // rad
    float moment_Nm_f;       // Nm

} Motor_Measure_t;

typedef struct 
{
    uint16_t position_rad;
    uint16_t speed_rads;
    uint16_t moment_Nm;
    uint16_t Kp;
    uint16_t Kd;

}Mit_Send_t;

typedef enum{
    CAN_MOTOR1 = (uint32_t)(CAN_MASTER_ID + 1),
    CAN_MOTOR2,
    CAN_MOTOR3,
    CAN_MOTOR4
}CAN_MOTOR_ID_t;

//**************************************************************************************************//

extern Motor_Measure_t motor_info[4]; 

//**************************************************************************************************//

void func_can_init(void);
void func_can_transmit_MIT(CAN_MOTOR_ID_t id,float position_rad_f,float speed_rads_f,float moment_Nm_f,float Kp_f,float Kd_f);

//**************************************************************************************************//
void func_can_transmit_Enable(CAN_MOTOR_ID_t id);//电机使能
void func_can_transmit_DisEnable(CAN_MOTOR_ID_t id);//电机失能
void func_can_transmit_SetZero(CAN_MOTOR_ID_t id);//设置当前角度为0
void func_can_transmit_ClearErr(CAN_MOTOR_ID_t id);//清除错误

#endif
