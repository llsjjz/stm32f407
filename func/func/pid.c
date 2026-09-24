/**
 * @file    pid.c
 * @brief   电机转速 PID 控制器 —— 位置式，带动态电流限幅与越障增强
 *          输入/反馈：电机转速（RPM）
 *          输出：    电机电流（-16384~16384，对应 -20A~20A）
 *
 * 项目要求（见 项目概述.md 与 CLAUDE.md）：
 *   1. 电机堵转时能快速提高电流以提升扭矩进行爬楼梯
 *   2. 全程严禁电机转速超出规定（保护车轮）
 *   3. C620 必须持续更新电流，否则自动清零（调用方需以稳定周期调用本函数）
 */

#include "pid.h"
#include <math.h>   /* fabsf, powf */

/* ========================================================================
 * 初始化
 * ======================================================================== */

void PID_Init(PID_struct *pid, float kp, float ki, float kd)
{
    /* ---- 整定参数 ---- */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* ---- 通用限幅 ---- */
    pid->integral_limit = (float)MAXNUMA;   /* 积分项上限默认与电流上限一致 */
    pid->out_min        = (float)MINNUMA;
    pid->out_max        = (float)MAXNUMA;

    /* ---- 动态电流限制（非线性幂次曲线） ---- */
    pid->current_max_stall  = (float)MAXNUMA;          /* 堵转时允许全力     */
    pid->current_max_noload = 0.6f * (float)MAXNUMA;   /* 空载全速时收紧    */
    pid->speed_limit        = (float)MAXRPM;
    pid->curve_exponent     = 2.0f;                    /* 二次曲线：低速扭矩足 */

    /* ---- 越障/堵转检测（默认阈值基于 speed_limit 自动计算） ---- */
    pid->obstacle_speed_thr = 0.1f * (float)MAXRPM;    /* 60 RPM  以下视为可能堵转 */
    pid->obstacle_error_thr = 0.3f * (float)MAXRPM;    /* 180 RPM 以上误差确认堵转 */

    /* ---- 运行时状态 ---- */
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
}

/* ========================================================================
 * 动态电流限幅
 * ======================================================================== */

/**
 * @brief 根据当前转速计算允许的最大电流（幂次曲线）
 *
 *         current_max_stall  ┤＼
 *                            │  ＼___  exponent=3.0
 *                            │      ＼________  exponent=1.0 (线性)
 *         current_max_noload ┤·······················
 *                            │                      │
 *                            0               speed_limit
 *
 * 幂次 > 1 时低速段电流衰减缓慢，留出更多扭矩用于越障。
 */
static float calc_current_limit(const PID_struct *pid, float abs_speed)
{
    if (pid->speed_limit <= 0.0f) {
        return pid->out_max;
    }

    if (abs_speed >= pid->speed_limit) {
        return pid->current_max_noload;
    }

    float ratio = abs_speed / pid->speed_limit;

    float curve;
    if (pid->curve_exponent <= 1.0f) {
        curve = ratio;                          /* 线性，省去 powf 开销 */
    } else {
        curve = powf(ratio, pid->curve_exponent);
    }

    return pid->current_max_stall
           - (pid->current_max_stall - pid->current_max_noload) * curve;
}

/* ========================================================================
 * 核心计算
 * ======================================================================== */

int16_t PID_run(PID_struct *pid, int16_t target, int16_t actual, float dt)
{
    /* dt 必须为正，避免除零 */
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    /* ---- 1. 误差 ---- */
    float error = (float)target - (float)actual;

    /* ---- 2. 比例项 ---- */
    float p_out = pid->kp * error;

    /* ---- 3. 微分项（微分-on-measurement） ----
     * derivative = (prev_ω - ω) / dt = -dω/dt
     * 转速上升 → 负微分 → 削减输出 → 抑制超调（保护车轮不超速）
     * 转速下跌（越障失速）→ 正微分 → 追加输出 → 辅助冲障
     */
    float derivative = (pid->prev_measurement - (float)actual) / dt;
    pid->prev_measurement = (float)actual;
    float d_out = pid->kd * derivative;

    /* ---- 4. 积分项 + 抗饱和 ---- */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) {
        pid->integral =  pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->ki * pid->integral;

    /* ---- 5. 原始输出 ---- */
    float output = p_out + i_out + d_out;

    /* ---- 6. 动态电流上限（幂次曲线） ---- */
    float abs_speed = fabsf((float)actual);
    float dynamic_max = calc_current_limit(pid, abs_speed);

    /* ---- 7. 越障/堵转检测 ----
     * 条件：转速极低 + 误差极大 → 判定堵转/越障，拉满堵转电流
     * 实现「堵转时能快速提高电流以提升扭矩进行爬楼梯」
     */
    if (abs_speed < pid->obstacle_speed_thr
        && fabsf(error) > pid->obstacle_error_thr) {
        dynamic_max = pid->current_max_stall;
    }

    /* 动态上限不超过硬件硬上限 */
    if (dynamic_max > pid->out_max) {
        dynamic_max = pid->out_max;
    }

    /* ---- 8. 输出限幅 ---- */
    /* 对称限幅：负向同样受 dynamic_max 约束（反向堵转也保护） */
    if (output >  dynamic_max) {
        output =  dynamic_max;
    } else if (output < -dynamic_max) {
        output = -dynamic_max;
    }

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return (int16_t)output;
}

/* ========================================================================
 * 复位
 * ======================================================================== */

void PID_reset(PID_struct *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
}
