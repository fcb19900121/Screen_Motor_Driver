/**
  ******************************************************************************
  * @file    hall_sensor.h
  * @brief   AB两相编码器驱动模块（11线，四倍频44脉冲/转）
  ******************************************************************************
  */
#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdint.h>

/* ---------- 编码器参数 ---------- */
#define ENCODER_LINES           11      /* 编码器线数 */
#define ENCODER_PPR             (ENCODER_LINES * 4)  /* 四倍频脉冲/转 = 44 */

/* ---------- 减速比 ---------- */
#define GEAR_RATIO_M1           1400    /* 电机1减速比 1:1400 */
#define GEAR_RATIO_M2           1000    /* 电机2减速比 1:1000（待确认） */

/* 输出轴每转脉冲数 = ENCODER_PPR * GEAR_RATIO */
#define OUTPUT_PPR_M1           ((int32_t)ENCODER_PPR * GEAR_RATIO_M1)  /* 61600 */
#define OUTPUT_PPR_M2           ((int32_t)ENCODER_PPR * GEAR_RATIO_M2)  /* 44000 */

/* ---------- 电机编号 ---------- */
#define MOTOR_1                 0
#define MOTOR_2                 1
#define MOTOR_NUM               2

/* ---------- 编码器数据结构 ---------- */
typedef struct {
    volatile int32_t  count;            /* 四倍频脉冲累计计数（有符号） */
    volatile int32_t  last_count;       /* 上次速度采样时的计数值 */
    volatile int8_t   direction;        /* 当前方向: +1 正转, -1 反转 */
    volatile int32_t  speed_dps10;       /* 输出轴速度（0.1°/s），由定时器刷新 */
} Encoder_t;

/* ---------- 公共接口 ---------- */

/** @brief 初始化编码器：重新配置 GPIO 为双边沿中断，使能 NVIC */
void Hall_Init(void);

/** @brief 获取指定电机累计脉冲计数 */
int32_t Hall_GetCount(uint8_t motor_id);

/** @brief 获取指定电机输出轴角度（0.01° 为单位，范围 0~35999） */
uint16_t Hall_GetAngle(uint8_t motor_id);

/** @brief 获取指定电机输出轴速度（0.1°/s） */
int32_t Hall_GetSpeed(uint8_t motor_id);

/** @brief 复位指定电机编码器计数 */
void Hall_ResetCount(uint8_t motor_id);

/** @brief 速度计算，应在固定周期定时器中调用（如 10ms 周期） */
void Hall_CalcSpeed(uint32_t period_ms);

/** @brief EXTI 回调处理，在 HAL_GPIO_EXTI_Callback 中调用 */
void Hall_EXTI_Callback(uint16_t GPIO_Pin);

/** @brief 获取编码器结构体指针（供外部高级用途） */
Encoder_t* Hall_GetEncoder(uint8_t motor_id);

#endif /* HALL_SENSOR_H */
