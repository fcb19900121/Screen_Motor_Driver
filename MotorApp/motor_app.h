#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"

/* Pitch axis uses Motor2: linear actuator with hall encoder */
#define PITCH_POS_MIN_COUNT            0
#define PITCH_POS_MAX_COUNT            40960

/* Homing parameters */
#define PITCH_HOME_DUTY                MOTOR_PWM_MAX
#define PITCH_HOME_MIN_RUN_MS          300U
#define PITCH_HOME_STABLE_STOP_MS      300U
#define PITCH_HOME_SPEED_STOP_TH_DPS10 2
#define PITCH_HOME_TIMEOUT_MS          10000U  /* 回零超时(ms)，超时进FAULT */

/* 回零方向为正占空比，远离零点方向为负占空比，编码器计数随位置增加；若硬件相反则改为 -1 */
#define PITCH_COUNT_SIGN               (1)

/* Position control parameters */
#define PITCH_MOVE_KP_NUM              8       /* P增益分子 */
#define PITCH_MOVE_KP_DEN              1       /* P增益分母 (Kp = NUM/DEN) */
#define PITCH_MOVE_DEADBAND            20      /* 到位死区(编码器计数) */
#define PITCH_MOVE_MIN_DUTY            300     /* 最小驱动占空比(克服摩擦) */
#define PITCH_MOVE_MAX_DUTY            MOTOR_PWM_MAX
#define PITCH_MOVE_TIMEOUT_MS          15000U  /* 运动超时(ms) */

typedef enum {
    PITCH_STATE_UNINIT = 0,
    PITCH_STATE_IDLE,
    PITCH_STATE_HOMING,
    PITCH_STATE_HOMED,
    PITCH_STATE_MOVING,
    PITCH_STATE_FAULT
} PitchState_t;

void MotorApp_Init(void);
void MotorApp_StartHoming(void);
void MotorApp_Task(uint32_t now_ms);

bool MotorApp_IsPitchHomed(void);
PitchState_t MotorApp_GetPitchState(void);

/* Position based on homed zero point, clamped to 0..40960 */
int32_t MotorApp_GetPitchPositionCount(void);
int32_t MotorApp_GetPitchRawCount(void);

/* Manual duty for pitch actuator (-MOTOR_PWM_MAX..MOTOR_PWM_MAX) */
void MotorApp_PitchSetDuty(int16_t duty);

/* Move to target position (0..40960), requires HOMED state */
bool MotorApp_PitchMoveTo(int32_t target_count);

/* Check if position move is in progress */
bool MotorApp_IsPitchMoving(void);

/* Get current target count (-1 if not moving) */
int32_t MotorApp_GetPitchTarget(void);

/* Limit switch EXTI callback, call from HAL_GPIO_EXTI_Callback */
void MotorApp_LimitSwitch_Callback(uint16_t GPIO_Pin);

/* Limit switch status (bit0=LIMIT1/竖屏, bit1=LIMIT2/横屏) */
uint8_t MotorApp_GetLimitFlags(void);

/* 屏幕方向 */
typedef enum {
    SCREEN_UNKNOWN              = 0,  /* 未到达任何限位 */
    SCREEN_PORTRAIT             = 1,  /* 竖屏 (LIMIT1) — 已到位 */
    SCREEN_LANDSCAPE            = 2,  /* 横屏 (LIMIT2) — 已到位 */
    SCREEN_MOVING_TO_PORTRAIT   = 3,  /* 正在转向竖屏（Flash中间态） */
    SCREEN_MOVING_TO_LANDSCAPE  = 4   /* 正在转向横屏（Flash中间态） */
} ScreenOrientation_t;

ScreenOrientation_t MotorApp_GetScreenOrientation(void);

/* Motor1 横竖屏切换控制 */
#define SCREEN_ROTATE_DUTY             MOTOR_PWM_MAX/2  /* 旋转占空比(50%) */
#define SCREEN_OVERRUN_MS              0U   /* 限位触发后继续运行时间(ms)，调此值使停在限位中间 */
#define SCREEN_PROBE_DUTY              (MOTOR_PWM_MAX/2) /* 自检探测占空比(50%低速) */
#define SCREEN_PROBE_TIMEOUT_MS        20000U /* 自检总超时(ms) */
#define SCREEN_PROBE_STEP_MS           800U   /* 每步增幅时间(ms)，即每个半周期增加的时长 */
#define SCREEN_PROBE_INIT_MS           800U   /* 初始半周期时长(ms) */
#define SCREEN_PROBE_MAX_HALF_MS       3000U  /* 单方向最大运行时间(ms) */

/* 屏幕旋转角度保护：首次检测到限位清零编码器，对端不超过 95° */
#define SCREEN_ANGLE_MAX_COUNT         16300  /* 软件限位计数值，约 95° (15400=90°) */
void MotorApp_ScreenRotateTo(ScreenOrientation_t target);
void MotorApp_ScreenTask(void);

/* 获取屏幕旋转角度 (0.01° 单位，有符号；如 -568 表示 -5.68°) */
int32_t MotorApp_GetScreenAngleDeg100(void);

/* 上电自检：读限位GPIO判定当前方向，若未在限位则低速探测 */
void MotorApp_ScreenDetect(void);

#endif /* MOTOR_APP_H */