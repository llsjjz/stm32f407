/**
 * @file    func_pid.h
 * @brief   电机转速PID控制器 —— 位置式，带动态电流限幅与越障增强
 *
 * 控制量说明：
 *   - 输入 / 反馈：电机转速（单位：RPM）
 *   - 输出：           电机电流（范围 -16532~16532，对应 -20A~20A）
 *
 * 特点：
 *   1. 位置式PID + 前馈项，加快响应
 *   2. 微分项基于测量值（-dω/dt），避免目标突变时的微分冲击，越障失速时自动追加扭矩
 *   3. 非线性幂次电流限幅曲线：低速段保持高扭矩，高速段收紧限流
 *   4. 越障检测：堵转/极低速 + 大误差 → 自动拉满堵转电流
 *   5. 积分限幅式抗饱和（anti-windup），计算量小，适合嵌入式
 */

#ifndef __FUNC_PID_H__
#define __FUNC_PID_H__

#include <stdint.h>

/**
 * @brief PID控制器结构体
 *
 * 所有参数均为 float 类型，方便在 STM32F4 上使用 FPU 进行硬件浮点运算。
 */
typedef struct {
    /* ---- 整定参数（用户配置） ---- */
    float kp;             /**< 比例系数                                         */
    float ki;             /**< 积分系数                                         */
    float kd;             /**< 微分系数                                         */
    float kf;             /**< 前馈增益（0=禁用），典型值 0.01~0.1               */

    /* ---- 内部状态（运行时自动更新，复位时可清零） ---- */
    float integral;       /**< 误差积分累加值，受 integral_limit 限制             */
    float prev_measurement; /**< 上一次的测量值（实际转速），用于微分-on-measurement */

    /* ---- 通用限幅参数 ---- */
    float integral_limit; /**< 积分项绝对值的上限                                */
    float out_min;        /**< 输出电流的绝对下限（通常 -16532 或 0）             */
    float out_max;        /**< 输出电流的绝对上限（通常 +16532）                  */

    /* ---- 基于转速的动态电流限制 ---- */
    float current_max_stall;  /**< 堵转时的最大允许电流（转速=0时）               */
    float current_max_noload; /**< 空载全速时的最大允许电流（转速≥speed_limit时） */
    float speed_limit;        /**< 电流缩减的转速分界点（RPM）                    */
    float curve_exponent;     /**< 电流曲线幂次（1.0=线性, 2.0=二次, 越大低速扭矩越足） */

    /* ---- 越障检测 ---- */
    float obstacle_speed_thr; /**< 越障判定转速阈值（RPM），低于此值认为可能堵转  */
    float obstacle_error_thr; /**< 越障判定误差阈值（RPM），超过此值确认堵转      */
} pid_ctrl_t;

/**
 * @brief 初始化PID控制器
 *
 * @param pid             控制器实例指针
 * @param kp              比例系数
 * @param ki              积分系数
 * @param kd              微分系数
 * @param integral_limit  积分限幅值（对称限幅）
 * @param out_min         输出电流硬下限
 * @param out_max         输出电流硬上限
 * @param stall_current   堵转允许的最大电流
 * @param noload_current  全速时允许的最大电流
 * @param speed_limit     电流开始缩减的转速分界点（RPM）
 */
void pid_init(pid_ctrl_t *pid,
              float kp, float ki, float kd,
              float integral_limit,
              float out_min, float out_max,
              float stall_current, float noload_current,
              float speed_limit);

/**
 * @brief 执行一次PID计算
 *
 * 建议在定时中断中以固定周期调用。
 *
 * @param  pid           控制器实例指针
 * @param  target_speed  目标转速（RPM）
 * @param  actual_speed  实际转速（RPM，来自编码器/霍尔传感器等反馈）
 * @param  dt            距离上次调用的时间间隔（秒），如 10ms 传 0.01f
 * @return               控制输出——电机电流值
 *
 * 内部处理流程：
 *   1. 计算误差 = 目标 - 实际
 *   2. 比例项 P = Kp × error
 *   3. 前馈项 FF = Kf × target
 *   4. 微分项 D = Kd × (prev_measurement - actual) / dt  （= -Kd × dω/dt）
 *   5. 积分累加并限幅 I = Ki × integral
 *   6. 求和得到原始输出
 *   7. 根据实际转速计算动态电流上限（幂次曲线）
 *   8. 越障检测：转速<阈值 且 误差>阈值 → 电流上限拉至 stall_current
 *   9. 输出钳位到 [out_min, dynamic_max]
 */
float pid_compute(pid_ctrl_t *pid, float target_speed, float actual_speed, float dt);

/**
 * @brief 复位PID控制器内部状态
 *
 * 清零积分累加值和上一次测量值。通常在电机启动前、模式切换时调用。
 *
 * @param pid 控制器实例指针
 */
void pid_reset(pid_ctrl_t *pid);

#endif
