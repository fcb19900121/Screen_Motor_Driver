/**
  ******************************************************************************
  * @file    current_sensor.c
  * @brief   电流采集模块实现
  *          ADC1 注入组: rank1=AIS1(CH0), rank2=AIS2(CH1), rank3=保留
  *          ADC2 注入组: rank1=BIS2(CH12), rank2=BIS1(CH13), rank3=VBUS(CH4)
  ******************************************************************************
  */
#include "current_sensor.h"
#include "main.h"

/* ---------- 外部变量（CubeMX 生成） ---------- */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* ---------- 私有变量 ---------- */
static CurrentData_t current_data[MOTOR_NUM];
static uint16_t vbus_raw = 0;
static uint16_t is_offset_ais1 = 0;
static uint16_t is_offset_ais2 = 0;
static uint16_t is_offset_bis1 = 0;
static uint16_t is_offset_bis2 = 0;

/* ========================= 辅助函数 ========================= */

/**
 * @brief  将 ADC 原始值转换为 mV
 */
static uint16_t adc_to_mv(uint16_t raw)
{
    return (uint16_t)((uint32_t)raw * ADC_VREF_MV / ADC_RESOLUTION);
}

/* 常规通道软件采样，避免注入组触发链路导致无更新 */
static uint16_t adc_read_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }

    uint16_t raw = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return raw;
}

static uint16_t adc_read_channel_avg(ADC_HandleTypeDef *hadc, uint32_t channel, uint8_t samples)
{
    if (samples == 0) return 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += adc_read_channel(hadc, channel);
    }
    return (uint16_t)(sum / samples);
}

/**
 * @brief  将 IS 引脚电压(mV)转换为负载电流(mA)
 *         Iload(mA) = Vis(mV) * kilis / Ris / 1000
 *         为避免整数除法截断丢失精度，调整计算顺序
 */
static int16_t is_voltage_to_current_ma(uint16_t vis_mv)
{
    /* Iload(mA) = vis_mv * KILIS / (RIS * 1000)
     * 先乘大数再除，避免中间结果被截断为0 */
    return (int16_t)((uint32_t)vis_mv * BTN9970_KILIS / ((uint32_t)BTN9970_RIS_OHM * 1000UL));
}

/* ========================= 初始化 ========================= */
void Current_Init(void)
{
    /* 清零数据 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        current_data[i].raw_p      = 0;
        current_data[i].raw_n      = 0;
        current_data[i].current_ma = 0;
        current_data[i].vbus_mv    = 0;
    }
    vbus_raw = 0;

    /* ADC 校准 */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    /*
     * CubeMX 配置注入组触发源为 T1_TRGO，但 TIM1 仅作时基未产生 TRGO 触发，
     * 这里将 JEXTSEL 改为 111b（软件触发 JSWSTART），使注入组可由软件启动。
     */
    MODIFY_REG(hadc1.Instance->CR2, ADC_CR2_JEXTSEL, ADC_CR2_JEXTSEL);
    MODIFY_REG(hadc2.Instance->CR2, ADC_CR2_JEXTSEL, ADC_CR2_JEXTSEL);

    /* 上电零点校准：电机静止时采样IS基线 */
    {
        uint32_t sum_ais1 = 0;
        uint32_t sum_ais2 = 0;
        uint32_t sum_bis1 = 0;
        uint32_t sum_bis2 = 0;
        const uint8_t cal_n = 64;

        for (uint8_t i = 0; i < cal_n; i++) {
            sum_ais1 += adc_read_channel(&hadc1, ADC_CHANNEL_0);
            sum_ais2 += adc_read_channel(&hadc1, ADC_CHANNEL_1);
            sum_bis2 += adc_read_channel(&hadc2, ADC_CHANNEL_12);
            sum_bis1 += adc_read_channel(&hadc2, ADC_CHANNEL_13);
        }

        is_offset_ais1 = (uint16_t)(sum_ais1 / cal_n);
        is_offset_ais2 = (uint16_t)(sum_ais2 / cal_n);
        is_offset_bis1 = (uint16_t)(sum_bis1 / cal_n);
        is_offset_bis2 = (uint16_t)(sum_bis2 / cal_n);
    }
}

/* ========================= 采样 ========================= */
void Current_Sample(void)
{
    /* 使用常规通道软件采样并平均，降低PWM纹波影响 */
    uint16_t ais1_raw = adc_read_channel_avg(&hadc1, ADC_CHANNEL_0, 8);
    uint16_t ais2_raw = adc_read_channel_avg(&hadc1, ADC_CHANNEL_1, 8);
    uint16_t bis2_raw = adc_read_channel_avg(&hadc2, ADC_CHANNEL_12, 8);
    uint16_t bis1_raw = adc_read_channel_avg(&hadc2, ADC_CHANNEL_13, 8);
    vbus_raw          = adc_read_channel_avg(&hadc2, ADC_CHANNEL_4, 8);

    /* ----- Motor 1: AIS1(正侧), AIS2(负侧) ----- */
    current_data[MOTOR_1].raw_p = ais1_raw;
    current_data[MOTOR_1].raw_n = ais2_raw;

    uint16_t ais1_eff_raw = (ais1_raw > is_offset_ais1) ? (uint16_t)(ais1_raw - is_offset_ais1) : 0;
    uint16_t ais2_eff_raw = (ais2_raw > is_offset_ais2) ? (uint16_t)(ais2_raw - is_offset_ais2) : 0;
    uint16_t ais1_mv = adc_to_mv(ais1_eff_raw);
    uint16_t ais2_mv = adc_to_mv(ais2_eff_raw);
    int16_t  i_p1    = is_voltage_to_current_ma(ais1_mv);
    int16_t  i_n1    = is_voltage_to_current_ma(ais2_mv);
    /* 电机电流取当前导通半桥的电流（取较大值，符号表示方向） */
    if (i_p1 >= i_n1) {
        current_data[MOTOR_1].current_ma = i_p1;
    } else {
        current_data[MOTOR_1].current_ma = -i_n1;
    }

    if ((i_p1 < 20) && (i_n1 < 20)) {
        current_data[MOTOR_1].current_ma = 0;
    }

    /* ----- Motor 2: BIS1(正侧), BIS2(负侧) ----- */
    current_data[MOTOR_2].raw_p = bis1_raw;
    current_data[MOTOR_2].raw_n = bis2_raw;

    uint16_t bis1_eff_raw = (bis1_raw > is_offset_bis1) ? (uint16_t)(bis1_raw - is_offset_bis1) : 0;
    uint16_t bis2_eff_raw = (bis2_raw > is_offset_bis2) ? (uint16_t)(bis2_raw - is_offset_bis2) : 0;
    uint16_t bis1_mv = adc_to_mv(bis1_eff_raw);
    uint16_t bis2_mv = adc_to_mv(bis2_eff_raw);
    int16_t  i_p2    = is_voltage_to_current_ma(bis1_mv);
    int16_t  i_n2    = is_voltage_to_current_ma(bis2_mv);
    if (i_p2 >= i_n2) {
        current_data[MOTOR_2].current_ma = i_p2;
    } else {
        current_data[MOTOR_2].current_ma = -i_n2;
    }

    if ((i_p2 < 20) && (i_n2 < 20)) {
        current_data[MOTOR_2].current_ma = 0;
    }

    /* ----- VBUS ----- */
    uint16_t vadc_mv = adc_to_mv(vbus_raw);
    /* 分压还原: Vbus = Vadc * (R_HIGH + R_LOW) / R_LOW */
    uint32_t vbus_mv_val = (uint32_t)vadc_mv * (VBUS_R_HIGH + VBUS_R_LOW) / VBUS_R_LOW;
    current_data[MOTOR_1].vbus_mv = (uint16_t)vbus_mv_val;
    current_data[MOTOR_2].vbus_mv = (uint16_t)vbus_mv_val;
}

/* ========================= 读取接口 ========================= */
int16_t Current_GetMotor(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return 0;
    return current_data[motor_id].current_ma;
}

uint16_t Current_GetRaw(uint8_t motor_id, uint8_t half_bridge)
{
    if (motor_id >= MOTOR_NUM) return 0;
    return (half_bridge == HALF_BRIDGE_P) ?
        current_data[motor_id].raw_p : current_data[motor_id].raw_n;
}

uint16_t Current_GetVbus(void)
{
    return current_data[0].vbus_mv;
}

CurrentData_t* Current_GetData(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return &current_data[0];
    return &current_data[motor_id];
}
