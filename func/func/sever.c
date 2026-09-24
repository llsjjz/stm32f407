#include "sever.h"


//========================================= 宏定义 =========================================//
// 舵机通道配置（魔法数字宏定义，提高可读性和可维护性）
#define SR_PWM_TIMER        &htim1                  // 舵机使用的定时器句柄
#define SR_PWM_MIN_DUTY     (0.025*(*(&htim1.Instance->ARR)+1))                       // 舵机0度对应的PWM占空比
#define SR_PWM_MAX_DUTY     (0.125*(*(&htim1.Instance->ARR)+1))                       // 舵机180度对应的PWM占空比（5 + 180*20/180 = 25）
#define SR_TOTAL_NUM        4                       // 舵机总数
#define SR_ANGLE_MIN        0.0f                    // 舵机最小角度（度）
#define SR_ANGLE_MAX        270.0f                  // 舵机最大角度（度）


// 舵机通道定义（参数具体化，注释完整化）
#define SR_CHANNEL_1        TIM_CHANNEL_1           // 舵机1对应的PWM通道
#define SR_CHANNEL_2        TIM_CHANNEL_2           // 舵机2对应的PWM通道
#define SR_CHANNEL_3        TIM_CHANNEL_3           // 舵机3对应的PWM通道
#define SR_CHANNEL_4        TIM_CHANNEL_4           // 舵机4对应的PWM通道


//========================================= 结构体定义 =========================================//
/**
 * @brief 舵机参数结构体（参数具体化，注释完整化）
 * @param pwm_timer: 舵机PWM对应的定时器句柄
 * @param pwm_channel: 舵机PWM对应的通道
 * @param mode_1_angle:模式1对应的角度
 * @param mode_2_angle:模式2对应的角度
 * @param mode_3_angle:模式3对应的角度
 */
typedef struct {
    TIM_HandleTypeDef* pwm_timer;    // PWM定时器句柄
    uint32_t pwm_channel;            // PWM通道
	uint16_t mode_1_angle;			 //模式1对应的角度
	uint16_t mode_2_angle;			 //模式2对应的角度
	uint16_t mode_3_angle;			 //模式3对应的角度
} SR_Motor_Config_t;


//========================================= 全局变量 =========================================//
/**
 * @brief 4路舵机参数数组（初始化为对应的定时器和通道）
 */
static const SR_Motor_Config_t SR_motor_config[SR_TOTAL_NUM] = {
    {SR_PWM_TIMER, SR_CHANNEL_1,130,0,0},
    {SR_PWM_TIMER, SR_CHANNEL_2,45,220,0},
    {SR_PWM_TIMER, SR_CHANNEL_3,90,220,0},
    {SR_PWM_TIMER, SR_CHANNEL_4,0,100,0}
};


//========================================= 私有函数声明（此部分可选） =========================================//
static uint32_t SR_AngleToDuty(float angle);  // 角度转PWM占空比
static uint8_t SR_CheckParams(uint8_t num, float angle); // 参数合法性校验

//========================================= 公有函数实现 =========================================//
void SR_Init(void)
{
    // Set PSC + ARR BEFORE starting PWM (50Hz = 168MHz / 168 / 20000)
    __HAL_TIM_SET_PRESCALER(SR_PWM_TIMER, 168-1);
    __HAL_TIM_SET_AUTORELOAD(SR_PWM_TIMER, 20000-1);

    // Start PWM output for all channels
    for (uint8_t i = 0; i < SR_TOTAL_NUM; i++)
    {
        if (HAL_OK != HAL_TIM_PWM_Start(SR_motor_config[i].pwm_timer, SR_motor_config[i].pwm_channel))
        {
            Error_Handler();
        }
    }

    // Drive all servos to mode-1 default angle
    for (uint8_t i = 0; i < SR_TOTAL_NUM; i++)
    {
        SR_SetAngle(i, (float)SR_motor_config[i].mode_2_angle);
    }
}

uint8_t SR_SetAngle(uint8_t num, float angle)
{
    if (SR_CheckParams(num, angle) != 0)
    {
        return 1;
    }

    uint32_t duty = SR_AngleToDuty(angle);

    __HAL_TIM_SET_COMPARE(SR_motor_config[num].pwm_timer,
                          SR_motor_config[num].pwm_channel,
                          duty);

    return 0;
}

//========================================= 私有函数实现 =========================================//
static uint32_t SR_AngleToDuty(float angle)
{
    return (uint32_t)(SR_PWM_MIN_DUTY + (angle / SR_ANGLE_MAX) * (SR_PWM_MAX_DUTY - SR_PWM_MIN_DUTY));
}

static uint8_t SR_CheckParams(uint8_t num, float angle)
{
    if (num >= SR_TOTAL_NUM)
    {
        return 1;
    }

    if (angle < SR_ANGLE_MIN || angle > SR_ANGLE_MAX)
    {
        return 2;
    }

    return 0;
}


//========================================= 扩展实现 =========================================//
void sever_setmode(uint8_t mode)
{
	if(mode <=3 && mode >=1)
	{
		int i;
		for(i=0;i<4;i++)
		{
			if(mode==1)SR_SetAngle(i,SR_motor_config[i].mode_1_angle);
			if(mode==2)SR_SetAngle(i,SR_motor_config[i].mode_2_angle);
			if(mode==3)SR_SetAngle(i,SR_motor_config[i].mode_3_angle);
		}
	}
	else
	{
		if(mode/10==1)
		{
			if(mode%10==1)
			{
				SR_SetAngle(0,SR_motor_config[0].mode_1_angle);
				SR_SetAngle(1,SR_motor_config[1].mode_1_angle);
			}
			else if(mode%10==2)
			{
				SR_SetAngle(2,SR_motor_config[2].mode_1_angle);
				SR_SetAngle(3,SR_motor_config[3].mode_1_angle);
			}
		}
		else if(mode/10==2)
		{
			if(mode%10==1)
			{
				SR_SetAngle(0,SR_motor_config[0].mode_2_angle);
				SR_SetAngle(1,SR_motor_config[1].mode_2_angle);
			}
			else if(mode%10==2)
			{
				SR_SetAngle(2,SR_motor_config[2].mode_2_angle);
				SR_SetAngle(3,SR_motor_config[3].mode_2_angle);
			}
		}
		else if(mode/10==3)
		{
			if(mode%10==1)
			{
				SR_SetAngle(0,SR_motor_config[0].mode_3_angle);
				SR_SetAngle(1,SR_motor_config[1].mode_3_angle);
			}
			else if(mode%10==2)
			{
				SR_SetAngle(2,SR_motor_config[2].mode_3_angle);
				SR_SetAngle(3,SR_motor_config[3].mode_3_angle);
			}
		}
	}
}
