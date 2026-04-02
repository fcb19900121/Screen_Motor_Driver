#include "motor_app.h"

#include "hall_sensor.h"
#include "motor_driver.h"
#include "main.h"
#include "eeprom_emul.h"

/* 限位标志: bit0=LIMIT1(竖屏), bit1=LIMIT2(横屏) */
static volatile uint8_t g_limit_flags = 0;
 volatile ScreenOrientation_t g_screen_orient = SCREEN_UNKNOWN;
static volatile ScreenOrientation_t g_screen_target = SCREEN_UNKNOWN; /* 目标方向 */
static volatile uint8_t g_screen_rotating = 0; /* 正在旋转中 */
static volatile uint32_t g_limit_hit_tick = 0; /* 限位触发时刻(HAL_GetTick) */
static volatile uint8_t g_limit_hit = 0;       /* 限位已触发，等待过冲延时 */
static volatile uint8_t g_screen_probing = 0;   /* 正在自检探测 */
static volatile uint32_t g_probe_start_tick = 0; /* 自检总开始时刻 */
static volatile uint32_t g_probe_swing_tick = 0; /* 当前半周期开始时刻 */
static volatile uint32_t g_probe_half_ms = 0;    /* 当前半周期时长 */
static volatile int8_t   g_probe_dir = 1;        /* 当前摆动方向: +1/-1 */
static volatile uint8_t  g_screen_calibrated = 0;  /* 上电后首次到达限位已校准编码器 */

typedef struct {
    PitchState_t state;
    uint32_t last_tick_ms;
    uint32_t homing_run_ms;
    uint32_t stall_stable_ms;
    int32_t pos_count;
    int32_t target_count;
    uint32_t move_run_ms;
} MotorAppCtx_t;

static MotorAppCtx_t g_app = {
    PITCH_STATE_UNINIT,
    0,
    0,
    0,
    0,
    -1,
    0,
};

static int32_t clamp_pitch_pos(int32_t v)
{
    if (v < PITCH_POS_MIN_COUNT) {
        return PITCH_POS_MIN_COUNT;
    }
    if (v > PITCH_POS_MAX_COUNT) {
        return PITCH_POS_MAX_COUNT;
    }
    return v;
}

void MotorApp_Init(void)
{
    g_app.state = PITCH_STATE_IDLE;
    g_app.last_tick_ms = 0;
    g_app.homing_run_ms = 0;
    g_app.stall_stable_ms = 0;
    g_app.pos_count = clamp_pitch_pos(PITCH_COUNT_SIGN * Hall_GetCount(MOTOR_2));
    g_app.target_count = -1;
    g_app.move_run_ms = 0;
}

void MotorApp_StartHoming(void)
{
    g_app.state = PITCH_STATE_HOMING;
    g_app.homing_run_ms = 0;
    g_app.stall_stable_ms = 0;

    /* Positive 100% duty: move actuator to mechanical zero end */
    Motor_SetSpeed(MOTOR_2, PITCH_HOME_DUTY);
}

void MotorApp_Task(uint32_t now_ms)
{
    uint32_t dt_ms = 0;

    if (g_app.last_tick_ms == 0U) {
        dt_ms = 0;
    } else {
        dt_ms = now_ms - g_app.last_tick_ms;
    }
    g_app.last_tick_ms = now_ms;

    if (g_app.state == PITCH_STATE_HOMING) {
        int32_t spd_dps10 = Hall_GetSpeed(MOTOR_2);

        if (dt_ms > 0U) {
            g_app.homing_run_ms += dt_ms;
        }

        /* 回零超时保护 */
        if (g_app.homing_run_ms >= PITCH_HOME_TIMEOUT_MS) {
            Motor_Stop(MOTOR_2);
            g_app.state = PITCH_STATE_FAULT;
            return;
        }

        /* Consider stop only after startup transient time */
        if (g_app.homing_run_ms >= PITCH_HOME_MIN_RUN_MS) {
            if ((spd_dps10 <= PITCH_HOME_SPEED_STOP_TH_DPS10) &&
                (spd_dps10 >= -PITCH_HOME_SPEED_STOP_TH_DPS10)) {
                g_app.stall_stable_ms += dt_ms;
            } else {
                g_app.stall_stable_ms = 0;
            }

            if (g_app.stall_stable_ms >= PITCH_HOME_STABLE_STOP_MS) {
                Motor_Stop(MOTOR_2);
                Hall_ResetCount(MOTOR_2);
                g_app.pos_count = 0;
                g_app.state = PITCH_STATE_HOMED;
                return;
            }
        }
    }

    /* ========== 位置控制状态 ========== */
    if (g_app.state == PITCH_STATE_MOVING) {

        /* 更新当前位置 */
        g_app.pos_count = clamp_pitch_pos(PITCH_COUNT_SIGN * Hall_GetCount(MOTOR_2));

        int32_t error = g_app.target_count - g_app.pos_count;

        /* 到位判定 */
        if (error >= -PITCH_MOVE_DEADBAND && error <= PITCH_MOVE_DEADBAND) {
            Motor_Stop(MOTOR_2);
            g_app.state = PITCH_STATE_HOMED;
            return;
        }

        /* P控制器计算占空比：回零为正占空比方向，远离零点需要负占空比，故 duty = -error * KP */
        int32_t duty = -(int32_t)error * PITCH_MOVE_KP_NUM / PITCH_MOVE_KP_DEN;

        /* 施加最小占空比以克服静摩擦 */
        if (duty > 0 && duty < PITCH_MOVE_MIN_DUTY) {
            duty = PITCH_MOVE_MIN_DUTY;
        } else if (duty < 0 && duty > -PITCH_MOVE_MIN_DUTY) {
            duty = -PITCH_MOVE_MIN_DUTY;
        }

        /* 限幅 */
        if (duty > PITCH_MOVE_MAX_DUTY) {
            duty = PITCH_MOVE_MAX_DUTY;
        } else if (duty < -PITCH_MOVE_MAX_DUTY) {
            duty = -PITCH_MOVE_MAX_DUTY;
        }

        Motor_SetSpeed(MOTOR_2, (int16_t)duty);
    }

    g_app.pos_count = clamp_pitch_pos(PITCH_COUNT_SIGN * Hall_GetCount(MOTOR_2));
}

bool MotorApp_IsPitchHomed(void)
{
    return (g_app.state == PITCH_STATE_HOMED);
}

PitchState_t MotorApp_GetPitchState(void)
{
    return g_app.state;
}

int32_t MotorApp_GetPitchPositionCount(void)
{
    return g_app.pos_count;
}

int32_t MotorApp_GetPitchRawCount(void)
{
    return Hall_GetCount(MOTOR_2);
}

void MotorApp_PitchSetDuty(int16_t duty)
{
    /* 回零/运动过程中禁止外部控制 */
    if (g_app.state == PITCH_STATE_HOMING || g_app.state == PITCH_STATE_MOVING) {
        return;
    }
    Motor_SetSpeed(MOTOR_2, duty);
}

bool MotorApp_PitchMoveTo(int32_t target_count)
{
    /* 仅上电回零过程中不接受命令，其他任何状态均可 */
    if (g_app.state == PITCH_STATE_HOMING) {
        return false;
    }

    /* 目标位置限幅 */
    target_count = clamp_pitch_pos(target_count);

    g_app.target_count = target_count;
    g_app.move_run_ms = 0;
    g_app.state = PITCH_STATE_MOVING;
    return true;
}

bool MotorApp_IsPitchMoving(void)
{
    return (g_app.state == PITCH_STATE_MOVING);
}

int32_t MotorApp_GetPitchTarget(void)
{
    return g_app.target_count;
}

/* ========================= 限位开关（电机1 横屏/竖屏切换） ========================= */
void MotorApp_LimitSwitch_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LIMIT1_Pin) {
        /* 正常旋转时：转向横屏则忽略LIMIT1（离开竖屏弹跳） */
        if (g_screen_rotating && !g_screen_probing && g_screen_target == SCREEN_LANDSCAPE) return;

        g_limit_flags |= 0x01;
        g_screen_orient = SCREEN_PORTRAIT;

        /* 上电首次到达限位：清零编码器，竖屏=0 */
        if (!g_screen_calibrated) {
            Hall_ResetCount(MOTOR_1);
            g_screen_calibrated = 1;
        }

        if (SCREEN_OVERRUN_MS == 0U) {
            Motor_Stop(MOTOR_1);
            g_screen_rotating = 0;
            g_screen_probing = 0;
        } else {
            g_limit_hit_tick = HAL_GetTick();
            g_limit_hit = 1;
        }
    }

    if (GPIO_Pin == LIMIT2_Pin) {
        /* 正常旋转时：转向竖屏则忽略LIMIT2（离开横屏弹跳） */
        if (g_screen_rotating && !g_screen_probing && g_screen_target == SCREEN_PORTRAIT) return;

        g_limit_flags |= 0x02;
        g_screen_orient = SCREEN_LANDSCAPE;

        /* 上电首次到达限位：清零编码器，横屏=0 */
        if (!g_screen_calibrated) {
            Hall_ResetCount(MOTOR_1);
            g_screen_calibrated = 1;
        }

        if (SCREEN_OVERRUN_MS == 0U) {
            Motor_Stop(MOTOR_1);
            g_screen_rotating = 0;
            g_screen_probing = 0;
        } else {
            g_limit_hit_tick = HAL_GetTick();
            g_limit_hit = 1;
        }
    }
}

uint8_t MotorApp_GetLimitFlags(void)
{
    return g_limit_flags;
}

ScreenOrientation_t MotorApp_GetScreenOrientation(void)
{
    return g_screen_orient;
}

int32_t MotorApp_GetScreenAngleDeg100(void)
{
    /* 返回 Motor1 输出轴角度，0.01°单位，有符号 */
    /* 例如 count=-972 → -972*36000/61600 = -568 → -5.68° */
    int32_t count = Hall_GetCount(MOTOR_1);
    return (int32_t)((int64_t)count * 36000 / OUTPUT_PPR_M1);
}

void MotorApp_ScreenRotateTo(ScreenOrientation_t target)
{
    if (target != SCREEN_PORTRAIT && target != SCREEN_LANDSCAPE) return;
    /* 已在目标位置则不动 */
    if (target == g_screen_orient && !g_screen_rotating) return;

    g_screen_target = target;
    g_screen_rotating = 1;
    g_limit_flags = 0;  /* 清除旧限位标志 */
    g_limit_hit = 0;    /* 清除残留限位触发 */

    /* 先写 MOVING_TO 中间态到 Flash，掉电后可检测到 */
    uint16_t movingState = (target == SCREEN_PORTRAIT)
                           ? (uint16_t)SCREEN_MOVING_TO_PORTRAIT
                           : (uint16_t)SCREEN_MOVING_TO_LANDSCAPE;
    EE_WriteVariable(EE_VAR_SCREEN_ORIENT, movingState);

    if (target == SCREEN_PORTRAIT) {
        /* 反转到竖屏，由LIMIT1中断停止 */
        Motor_SetSpeed(MOTOR_1, -(int16_t)SCREEN_ROTATE_DUTY);
    } else {
        /* 正转到横屏，由LIMIT2中断停止 */
        Motor_SetSpeed(MOTOR_1, (int16_t)SCREEN_ROTATE_DUTY);
    }
}

void MotorApp_ScreenTask(void)
{
    /* 限位已触发，等待过冲延时后停止电机 */
    if (g_limit_hit && (g_screen_rotating || g_screen_probing)) {
        if (HAL_GetTick() - g_limit_hit_tick >= SCREEN_OVERRUN_MS) {
            Motor_Stop(MOTOR_1);
            g_screen_rotating = 0;
            g_screen_probing = 0;
            g_limit_hit = 0;
            /* 保存当前屏幕方向到Flash */
            if (g_screen_orient != SCREEN_UNKNOWN) {
                EE_WriteVariable(EE_VAR_SCREEN_ORIENT, (uint16_t)g_screen_orient);
            }
        }
        return;
    }

    /* 角度软件保护：校准后，只拦截远离零点方向，允许回零方向 */
    if (g_screen_calibrated && (g_screen_rotating || g_screen_probing)) {
        int32_t cnt = Hall_GetCount(MOTOR_1);
        uint8_t over = 0;
        if (cnt > SCREEN_ANGLE_MAX_COUNT) {
            /* 正方向超限，只有继续往正方向才停（正duty） */
            if (g_screen_target == SCREEN_PORTRAIT)
                over = 1;  /* 根据实际：正count对应竖屏方向 */
            /* 往回走（目标横屏）放行 */
        } else if (cnt < -SCREEN_ANGLE_MAX_COUNT) {
            /* 负方向超限，只有继续往负方向才停 */
            if (g_screen_target == SCREEN_LANDSCAPE)
                over = 1;  /* 负count对应横屏方向 */
            /* 往回走（目标竖屏）放行 */
        }
        if (over) {
            Motor_Stop(MOTOR_1);
            g_screen_rotating = 0;
            g_screen_probing = 0;
            g_limit_hit = 0;
        }
    }

    /* 往返探测状态机 */
    if (g_screen_probing && !g_limit_hit) {
        uint32_t now = HAL_GetTick();
        /* 总超时 */
        if (now - g_probe_start_tick >= SCREEN_PROBE_TIMEOUT_MS) {
            Motor_Stop(MOTOR_1);
            g_screen_probing = 0;
            return;
        }
        /* 当前半周期到期 → 反转并加大幅度 */
        if (now - g_probe_swing_tick >= g_probe_half_ms) {
            g_probe_dir = (int8_t)(-g_probe_dir);
            g_probe_half_ms += SCREEN_PROBE_STEP_MS;
            if (g_probe_half_ms > SCREEN_PROBE_MAX_HALF_MS)
                g_probe_half_ms = SCREEN_PROBE_MAX_HALF_MS;
            g_probe_swing_tick = now;
            int16_t duty = (g_probe_dir > 0)
                           ? (int16_t)SCREEN_PROBE_DUTY
                           : -(int16_t)SCREEN_PROBE_DUTY;
            Motor_SetSpeed(MOTOR_1, duty);
        }
    }
}

static void ScreenDetect_StartProbe(void)
{
    g_screen_probing = 1;
    g_screen_rotating = 0;
    g_limit_hit = 0;
    g_limit_flags = 0;
    g_screen_orient = SCREEN_UNKNOWN;
    g_probe_dir = 1;
    g_probe_half_ms = SCREEN_PROBE_INIT_MS;
    g_probe_start_tick = HAL_GetTick();
    g_probe_swing_tick = g_probe_start_tick;
    Motor_SetSpeed(MOTOR_1, (int16_t)SCREEN_PROBE_DUTY);
}

void MotorApp_ScreenDetect(void)
{
    uint16_t saved = 0;
    if (EE_ReadVariable(EE_VAR_SCREEN_ORIENT, &saved) == EE_OK) {
        if (saved == SCREEN_PORTRAIT || saved == SCREEN_LANDSCAPE) {
            /* 上次正常到位，直接恢复 */
            g_screen_orient = (ScreenOrientation_t)saved;
            return;
        }
        if (saved == SCREEN_MOVING_TO_PORTRAIT ||
            saved == SCREEN_MOVING_TO_LANDSCAPE) {
            /* 上次旋转途中掉电，位置不确定，重新探测 */
            ScreenDetect_StartProbe();
            return;
        }
    }

    /* Flash无记录（首次上电），启动往返探测 */
    ScreenDetect_StartProbe();
}
