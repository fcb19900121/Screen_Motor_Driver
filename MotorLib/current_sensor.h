/**
  ******************************************************************************
  * @file    current_sensor.h
  * @brief   电流采集模块 — BTN9970 IS 引脚 ADC 采样
  *          Motor1: AIS1(PA0/ADC1_IN0), AIS2(PA1/ADC1_IN1)
  *          Motor2: BIS1(PC3/ADC2_IN13), BIS2(PC2/ADC2_IN12)
  *          VBUS:   PA4/ADC2_IN4
  ******************************************************************************
  */
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdint.h>

/* ---------- 电机编号 ---------- */
#ifndef MOTOR_1
#define MOTOR_1     0
#define MOTOR_2     1
#define MOTOR_NUM   2
#endif

/* ---------- 半桥编号 ---------- */
#define HALF_BRIDGE_P   0   /* 正侧半桥（AIS1 / BIS1） */
#define HALF_BRIDGE_N   1   /* 负侧半桥（AIS2 / BIS2） */

/* ---------- ADC 参数 ---------- */
#define ADC_VREF_MV         3300    /* 参考电压 mV */
#define ADC_RESOLUTION      4096    /* 12-bit */

/* BTN9970 IS 引脚电流比例系数：Iload = Vis / (Ris * kilis)
 * 典型 kilis = 8700, Ris 为外部采样电阻(Ω)
 * 若 Ris = 2kΩ，则 I(mA) = V_IS(mV) * 8700 / 2000 / 1000
 * 用户需根据实际电路修改以下参数
 */
#define BTN9970_KILIS       8700    /* 电流镜比例 */
#define BTN9970_RIS_OHM     2000    /* IS 采样电阻 Ω */

/* VBUS 分压电阻: R_HIGH(56kΩ) 串联 R_LOW(2.2kΩ)
 * Vbus = Vadc * (R_HIGH + R_LOW) / R_LOW
 * 为避免浮点运算，用整数表示（单位Ω） */
#define VBUS_R_HIGH         56000   /* 上拉电阻 Ω */
#define VBUS_R_LOW          2200    /* 下拉电阻 Ω */

/* ---------- 电流数据结构 ---------- */
typedef struct {
    uint16_t raw_p;         /* 正侧半桥 ADC 原始值 */
    uint16_t raw_n;         /* 负侧半桥 ADC 原始值 */
    int16_t  current_ma;    /* 电机总电流（mA，有符号表示方向） */
    uint16_t vbus_mv;       /* 母线电压 mV */
} CurrentData_t;

/* ---------- 公共接口 ---------- */

/** @brief 初始化 ADC 校准 */
void Current_Init(void);

/** @brief 触发一次 ADC 注入组转换并读取结果 */
void Current_Sample(void);

/** @brief 获取指定电机的电流（mA） */
int16_t Current_GetMotor(uint8_t motor_id);

/** @brief 获取指定电机指定半桥的 ADC 原始值 */
uint16_t Current_GetRaw(uint8_t motor_id, uint8_t half_bridge);

/** @brief 获取母线电压（mV） */
uint16_t Current_GetVbus(void);

/** @brief 获取完整电流数据结构 */
CurrentData_t* Current_GetData(uint8_t motor_id);

#endif /* CURRENT_SENSOR_H */
