/**
  ******************************************************************************
  * @file    hall_sensor.c
  * @brief   AB两相编码器驱动 — 四倍频解码（11线 × 4 = 44 PPR）
  *          使用 EXTI 双边沿中断，在 A/B 任一信号跳变时解码方向和计数
  ******************************************************************************
  */
#include "hall_sensor.h"
#include "main.h"

/* ========================= 私有变量 ========================= */
static Encoder_t encoders[MOTOR_NUM];

/* ========================= 初始化 ========================= */
/* GPIO 和 NVIC 已由 CubeMX 在 MX_GPIO_Init() 中配置为双边沿中断，
 * 此函数仅清零编码器数据 */
void Hall_Init(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        encoders[i].count      = 0;
        encoders[i].last_count = 0;
        encoders[i].direction  = 0;
        encoders[i].speed_dps10 = 0;
    }
}

/* ========================= 四倍频解码核心 ========================= */
/**
 * @brief  AB 四倍频解码表
 *
 * 状态编码: (prevA << 3) | (prevB << 2) | (curA << 1) | curB
 * 值为 +1 / -1 / 0(无效/未变)
 */
static const int8_t quad_table[16] = {
/*  prevAB\curAB   00   01   10   11  */
/*  00 */           0,  -1,  +1,   0,
/*  01 */          +1,   0,   0,  -1,
/*  10 */          -1,   0,   0,  +1,
/*  11 */           0,  +1,  -1,   0
};

/**
 * @brief  对指定电机进行一次四倍频解码
 * @param  id: 电机编号 MOTOR_1 / MOTOR_2
 * @param  cur_a: 当前 A 相电平 (0 或 1)
 * @param  cur_b: 当前 B 相电平 (0 或 1)
 */
static uint8_t prev_state[MOTOR_NUM] = {0, 0};

static void quad_decode(uint8_t id, uint8_t cur_a, uint8_t cur_b)
{
    uint8_t cur_state = (cur_a << 1) | cur_b;
    uint8_t index     = (prev_state[id] << 2) | cur_state;
    int8_t  delta     = quad_table[index];

    if (delta != 0) {
        encoders[id].count     += delta;
        encoders[id].direction  = delta;
    }
    prev_state[id] = cur_state;
}

/* ========================= EXTI 回调 ========================= */
void Hall_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* ------ Motor 1: HALL1_A (PB11) / HALL1_B (PB10) ------ */
    if (GPIO_Pin == HALL1_A_Pin || GPIO_Pin == HALL1_B_Pin) {
        uint8_t a = HAL_GPIO_ReadPin(HALL1_A_GPIO_Port, HALL1_A_Pin);
        uint8_t b = HAL_GPIO_ReadPin(HALL1_B_GPIO_Port, HALL1_B_Pin);
        quad_decode(MOTOR_1, a, b);
    }

    /* ------ Motor 2: HALL2_A (PB7) / HALL2_B (PB6) ------ */
    if (GPIO_Pin == HALL2_A_Pin || GPIO_Pin == HALL2_B_Pin) {
        uint8_t a = HAL_GPIO_ReadPin(HALL2_A_GPIO_Port, HALL2_A_Pin);
        uint8_t b = HAL_GPIO_ReadPin(HALL2_B_GPIO_Port, HALL2_B_Pin);
        quad_decode(MOTOR_2, a, b);
    }
}

/* ========================= 读取接口 ========================= */
int32_t Hall_GetCount(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return 0;
    return encoders[motor_id].count;
}

uint16_t Hall_GetAngle(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return 0;

    /* 根据电机选择对应的输出轴每转脉冲数 */
    int32_t output_ppr = (motor_id == MOTOR_1) ? OUTPUT_PPR_M1 : OUTPUT_PPR_M2;

    /* 将累计脉冲映射到 0~(output_ppr-1) */
    int32_t cnt = encoders[motor_id].count % output_ppr;
    if (cnt < 0) cnt += output_ppr;

    /* angle_001 = cnt * 36000 / output_ppr
     * 为避免溢出，先除后乘，或用 64-bit */
    uint32_t angle = (uint32_t)((uint64_t)cnt * 36000ULL / (uint32_t)output_ppr);
    return (uint16_t)angle;
}

int32_t Hall_GetSpeed(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return 0;
    return encoders[motor_id].speed_dps10;
}

void Hall_ResetCount(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return;
    encoders[motor_id].count      = 0;
    encoders[motor_id].last_count = 0;
    encoders[motor_id].speed_dps10 = 0;
    encoders[motor_id].direction  = 0;
}

/* ========================= 速度计算 ========================= */
void Hall_CalcSpeed(uint32_t period_ms)
{
    if (period_ms == 0) return;
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        int32_t delta = encoders[i].count - encoders[i].last_count;
        encoders[i].last_count = encoders[i].count;

        int32_t output_ppr = (i == MOTOR_1) ? OUTPUT_PPR_M1 : OUTPUT_PPR_M2;

        /* 输出轴速度 (0.1°/s)
         * = delta_count / output_ppr * 360° / period_s * 10
         * = delta * 3600000 / (output_ppr * period_ms)
         */
        encoders[i].speed_dps10 = (int32_t)((int64_t)delta * 3600000LL
                                   / ((int64_t)output_ppr * (int64_t)period_ms));
    }
}

Encoder_t* Hall_GetEncoder(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return &encoders[0];
    return &encoders[motor_id];
}
