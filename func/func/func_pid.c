/**
 * @file    func_pid.c
 * @brief   电机转速PID控制器 —— 位置式实现，带动态电流限幅与越障增强
 */

#include "func_pid.h"
#include <math.h>   /* fabsf, fminf, fmaxf, powf */

/* ========================================================================
 * 初始化
 * ======================================================================== */

void pid_init(pid_ctrl_t *pid,
              float kp, float ki, float kd,
              float integral_limit,
              float out_min, float out_max,
              float stall_current, float noload_current,
              float speed_limit)
{
    /* ---- 整定参数 ---- */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = 0.0f;                      /* 默认关闭前馈，向后兼容 */

    /* ---- 内部状态（不在 init 中复位，由 pid_reset 单独处理） ---- */
    /* pid->integral         = 0.0f; */
    /* pid->prev_measurement = 0.0f; */

    /* ---- 通用限幅 ---- */
    pid->integral_limit = integral_limit;
    pid->out_min        = out_min;
    pid->out_max        = out_max;

    /* ---- 动态电流限制 ---- */
    pid->current_max_stall  = stall_current;
    pid->current_max_noload = noload_current;
    pid->speed_limit        = speed_limit;
    pid->curve_exponent     = 1.0f;       /* 默认线性，向后兼容 */

    /* ---- 越障检测（默认阈值基于 speed_limit 自动计算） ---- */
    pid->obstacle_speed_thr = speed_limit * 0.1f;
    pid->obstacle_error_thr = speed_limit * 0.3f;
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
static float calc_current_limit(const pid_ctrl_t *pid, float abs_speed)
{
    if (pid->speed_limit <= 0.0f) {
        return pid->out_max;
    }

    if (abs_speed >= pid->speed_limit) {
        return pid->current_max_noload;
    }

    float ratio = abs_speed / pid->speed_limit;

    /* 幂次曲线：ratio ^ exponent，exp=1 退化为线性 */
    float curve;
    if (pid->curve_exponent <= 1.0f) {
        curve = ratio;                           /* 线性，省去 powf 开销 */
    } else {
        curve = powf(ratio, pid->curve_exponent);
    }

    return pid->current_max_stall
           - (pid->current_max_stall - pid->current_max_noload) * curve;
}

/* ========================================================================
 * 核心计算
 * ======================================================================== */

/**
 * @brief 执行一次完整的PID计算，返回控制电流
 *
 * 算法要点：
 *   P  = Kp × error
 *   FF = Kf × target                      ← 前馈：加速响应
 *   D  = Kd × (prev_ω - ω) / dt          ← 微分-on-measurement：抑超调、助越障
 *   I  = Ki × Σ(error × dt)
 *   out = P + I + D + FF
 *
 * 越障检测：|ω| < obstacle_speed_thr 且 |error| > obstacle_error_thr
 *   → 动态上限直接拉至 current_max_stall，全力输出
 */
float pid_compute(pid_ctrl_t *pid, float target_speed, float actual_speed, float dt)
{
    /* ---- 1. 误差 ---- */
    float error = target_speed - actual_speed;

    /* ---- 2. 比例项 ---- */
    float p_out = pid->kp * error;

    /* ---- 3. 前馈项 ---- */
    float ff_out = pid->kf * target_speed;

    /* ---- 4. 微分项（微分-on-measurement） ----
     * derivative = (prev_ω - ω) / dt = -dω/dt
     * 转速上升 → 负微分 → 削减输出 → 抑制超调
     * 转速下跌（越障失速）→ 正微分 → 追加输出 → 辅助冲障 */
    float derivative = (pid->prev_measurement - actual_speed) / dt;
    pid->prev_measurement = actual_speed;
    float d_out = pid->kd * derivative;

    /* ---- 5. 积分项 + 抗饱和 ----
     * 累加误差并限幅，防止积分器深度饱和 */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->ki * pid->integral;

    /* ---- 6. 原始输出 ---- */
    float output = p_out + i_out + d_out + ff_out;

    /* ---- 7. 动态电流上限（幂次曲线） ---- */
    float abs_speed = fabsf(actual_speed);
    float dynamic_max = calc_current_limit(pid, abs_speed);

    /* ---- 8. 越障检测 ----
     * 条件：转速极低 + 误差极大 → 判定为堵转/越障，拉满堵转电流 */
    if (abs_speed < pid->obstacle_speed_thr
        && fabsf(error) > pid->obstacle_error_thr) {
        dynamic_max = pid->current_max_stall;
    }

    /* 动态上限不超过硬件硬上限 */
    if (dynamic_max > pid->out_max) {
        dynamic_max = pid->out_max;
    }

    /* ---- 9. 输出限幅 ---- */
    if (output > dynamic_max) {
        output = dynamic_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}

/* ========================================================================
 * 复位
 * ======================================================================== */

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
}
