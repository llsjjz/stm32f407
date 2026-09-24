#ifndef __PID_H__
#define __PID_H__

#include "main.h"

/* 电流极值 / Current extremes — 20A corresponds to 16384 on C620 */
/* 20A 在 C620 上对应 16384 */
#define MAXNUMA 16384
#define MINNUMA -16384

/* 转速极值 / Speed extremes (RPM) */
/* 转速极值（RPM） */
#define MAXRPM 600
#define MINRPM -600

/* PID 结构体 / PID controller
 * 输入/反馈：电机转速（RPM，int16）
 * 输出：    电机电流（-16384~16384，对应 -20A~20A）
 *
 * 设计要点：
 *   1. 位置式 PID + 微分-on-measurement：抑制超调，越障失速时自动追加扭矩
 *   2. 积分限幅式抗饱和（anti-windup），适合嵌入式
 *   3. 非线性幂次电流限幅曲线：低速段留足扭矩，高速段收紧限流
 *   4. 越障/堵转检测：转速极低 + 误差极大 → 电流上限拉至 current_max_stall
 */
typedef struct {
    /* ---- 整定参数 ---- */
    float kp;
    float ki;
    float kd;

    /* ---- 通用限幅 ---- */
    float integral_limit;   /* 积分项绝对值上限                  */
    float out_min;          /* 输出电流硬下限（-16384）           */
    float out_max;          /* 输出电流硬上限（+16384）           */

    /* ---- 动态电流限制（按转速的非线性曲线） ---- */
    float current_max_stall;   /* 堵转时（转速=0）允许的最大电流    */
    float current_max_noload;  /* 空载全速时允许的最大电流          */
    float speed_limit;         /* 电流缩减的转速分界点（RPM）       */
    float curve_exponent;      /* 幂次：>1 低速段扭矩更足            */

    /* ---- 越障/堵转检测 ---- */
    float obstacle_speed_thr;  /* 转速低于此值视为可能堵转          */
    float obstacle_error_thr;  /* 误差大于此值确认堵转              */

    /* ---- 运行时状态（由 PID_run 自动更新，PID_reset 清零） ---- */
    float integral;          /* 误差积分累加                      */
    float prev_measurement;  /* 上一次实际转速，用于微分-on-measurement */
} PID_struct;

/**
 * @brief 初始化 PID 控制器
 * @param pid   控制器实例指针
 * @param kp ki kd  PID 三参数
 *
 * 其余字段（限幅、动态电流、越障阈值、状态）采用默认值：
 *   out_min = MINNUMA, out_max = MAXNUMA
 *   current_max_stall = MAXNUMA, current_max_noload = 0.6*MAXNUMA
 *   speed_limit = MAXRPM, curve_exponent = 2.0
 *   obstacle_speed_thr = 0.1*MAXRPM, obstacle_error_thr = 0.3*MAXRPM
 * 用户可在调用后按需覆盖任意字段。
 */
void PID_Init(PID_struct *pid, float kp, float ki, float kd);

/**
 * @brief 执行一次 PID 计算，返回控制电流
 *
 * 建议在固定周期（如 1ms/2ms）的定时中断中调用。
 *
 * @param pid     控制器实例指针
 * @param target  目标转速（RPM）
 * @param actual  实际转速（RPM，来自 motor_info[].speed_rpm）
 * @param dt      距上次调用的时间间隔（秒），如 2ms 传 0.002f
 * @return        电机电流值（-16384~16384）
 *
 * 计算流程：
 *   1. error = target - actual
 *   2. P = kp × error
 *   3. D = kd × (prev_meas - actual) / dt   （= -kd × dω/dt，微分-on-measurement）
 *   4. I 累加并积分限幅：I = ki × integral
 *   5. 原始输出 out = P + I + D
 *   6. 按转速的非线性幂次曲线计算动态电流上限 dynamic_max
 *   7. 越障检测：|actual| < obstacle_speed_thr 且 |error| > obstacle_error_thr
 *      → dynamic_max = current_max_stall
 *   8. 钳位到 [out_min, dynamic_max] 后返回
 */
int16_t PID_run(PID_struct *pid, int16_t target, int16_t actual, float dt);

/**
 * @brief 复位 PID 内部状态（清零积分与上一次测量值）
 * @param pid 控制器实例指针
 */
void PID_reset(PID_struct *pid);

#endif
