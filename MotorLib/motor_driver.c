/**
  ******************************************************************************
  * @file    motor_driver.c
  * @brief   双直流有刷电机全桥 PWM 驱动实现
  *          使用 TIM8 的 4 个通道驱动两个 BTN9970 全桥
  *            CH1(PC6)=MOTOR2_P   CH2(PC7)=MOTOR2_N
  *            CH3(PC8)=MOTOR1_P   CH4(PC9)=MOTOR1_N
  *          EN_1(PB15) / EN_2(PB14) 控制驱动使能
  *
  *  正转: MOTOR_P = duty, MOTOR_N = 0
  *  反转: MOTOR_P = 0,    MOTOR_N = duty
  *  刹车: MOTOR_P = max,  MOTOR_N = max
  *  滑行: MOTOR_P = 0,    MOTOR_N = 0
  ******************************************************************************
  */
#include "motor_driver.h"
#include "main.h"

/* ---------- 外部变量（CubeMX 生成） ---------- */
extern TIM_HandleTypeDef htim8;

/* ========================= 初始化 ========================= */
void Motor_Init(void)
{
    /* 使能驱动芯片 */
    HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_SET);

    /* 启动 TIM8 四通道 PWM 输出 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* 初始占空比为 0（停止） */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}

/* ========================= 速度/方向控制 ========================= */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed)
{
    uint16_t duty_p = 0;
    uint16_t duty_n = 0;

    /* 限幅 */
    if (speed > (int16_t)MOTOR_PWM_MAX)  speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    if (speed > 0) {
        /* 正转 */
        duty_p = (uint16_t)speed;
        duty_n = 0;
    } else if (speed < 0) {
        /* 反转 */
        duty_p = 0;
        duty_n = (uint16_t)(-speed);
    } else {
        /* 停止（滑行） */
        duty_p = 0;
        duty_n = 0;
    }

    if (motor_id == MOTOR_1) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty_p);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, duty_n);
    } else if (motor_id == MOTOR_2) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty_p);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty_n);
    }
}

/* ========================= 停止 ========================= */
void Motor_Stop(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
    } else if (motor_id == MOTOR_2) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    }
}

void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_1);
    Motor_Stop(MOTOR_2);
}

/* ========================= 使能控制 ========================= */
void Motor_Enable(uint8_t motor_id, uint8_t enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (motor_id == MOTOR_1) {
        HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, state);
    } else if (motor_id == MOTOR_2) {
        HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, state);
    }
}

/* ========================= 刹车 ========================= */
void Motor_Brake(uint8_t motor_id)
{
    if (motor_id == MOTOR_1) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, MOTOR_PWM_MAX);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MOTOR_PWM_MAX);
    } else if (motor_id == MOTOR_2) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, MOTOR_PWM_MAX);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, MOTOR_PWM_MAX);
    }
}
