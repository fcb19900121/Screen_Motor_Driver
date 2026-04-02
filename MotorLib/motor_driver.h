/**
  ******************************************************************************
  * @file    motor_driver.h
  * @brief   双直流有刷电机全桥驱动模块（BTN9970 半桥 × 2）
  *          TIM8: CH1=MOTOR1_P, CH2=MOTOR1_N, CH3=MOTOR2_P, CH4=MOTOR2_N
  *          EN_1 / EN_2 使能引脚
  ******************************************************************************
  */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* ---------- 电机编号（与 hall_sensor.h 一致） ---------- */
#ifndef MOTOR_1
#define MOTOR_1     0
#define MOTOR_2     1
#define MOTOR_NUM   2
#endif

/* ---------- PWM 参数 ---------- */
#define MOTOR_PWM_PERIOD        3199    /* TIM8 ARR，PWM 频率 = 64 MHz / 3200 = 20 kHz */
#define MOTOR_PWM_MAX           MOTOR_PWM_PERIOD  /* 占空比最大值 */

/* ---------- 方向定义 ---------- */
#define MOTOR_DIR_FORWARD        1
#define MOTOR_DIR_REVERSE       (-1)
#define MOTOR_DIR_STOP           0

/* ---------- 公共接口 ---------- */

/** @brief 初始化电机驱动：启动 TIM8 PWM，使能驱动芯片 */
void Motor_Init(void);

/**
 * @brief 设置电机速度与方向
 * @param motor_id  MOTOR_1 / MOTOR_2
 * @param speed     -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
 *                  正值=正转，负值=反转，0=停止
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed);

/** @brief 停止指定电机（两路PWM均置0） */
void Motor_Stop(uint8_t motor_id);

/** @brief 停止所有电机 */
void Motor_StopAll(void);

/** @brief 使能/失能指定电机驱动芯片 */
void Motor_Enable(uint8_t motor_id, uint8_t enable);

/** @brief 刹车（两路PWM同时输出高电平） */
void Motor_Brake(uint8_t motor_id);

#endif /* MOTOR_DRIVER_H */
