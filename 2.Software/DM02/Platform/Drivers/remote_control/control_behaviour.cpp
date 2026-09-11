#include "control_behaviour.h"
#include "system_timer_task.h"

/* 原始范围约为 ±660。死区用于吸收中位噪声，死区外从 0 连续映射到 1，
 * 避免摇杆刚越过死区就突然产生较大的速度指令。 */
#define RC_DEADBAND     80
#define RC_NORM         660.0f

/* 一阶低通只平滑当前有效的前进或转向指令；退出运动状态时清空历史值，
 * 防止下一次启动继承上一次的残余速度。 */
#define RC_ALPHA_VX  0.20f
#define RC_ALPHA_WZ  0.20f

static fp32 smooth_vx = 0.0f;
static fp32 smooth_wz = 0.0f;

static void reset_motion_filter(void)
{
    smooth_vx = 0.0f;
    smooth_wz = 0.0f;
}

/*==============================================================================
 * 一阶低通：out = α × raw + (1-α) × out_prev
 *============================================================================*/
static fp32 lpf_rc(fp32 raw, fp32 *prev, fp32 alpha)
{
    *prev = alpha * raw + (1.0f - alpha) * (*prev);
    return *prev;
}

/* 连续死区映射：(|raw|-deadband)/(max-deadband)，最后恢复原符号。 */
static fp32 rc_map(int16_t raw)
{
    int32_t magnitude = (raw < 0) ? -(int32_t)raw : (int32_t)raw;
    if (magnitude <= RC_DEADBAND) return 0.0f;

    fp32 value = ((fp32)magnitude - (fp32)RC_DEADBAND) /
                 (RC_NORM - (fp32)RC_DEADBAND);
    if (value > 1.0f) value = 1.0f;
    return (raw < 0) ? -value : value;
}

/*==============================================================================
 * 初始化
 *============================================================================*/
void behaviour_control_class::Init()
{
    remote_control_init();
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;
    control_mode = Dog_control_system_init;
    last_buttons = 0U;
    active_direction = 0;
    center_seen = false;
    button_input_ready = false;
    reset_motion_filter();
}

void behaviour_control_class::Set_control_mode(remote_control_mode mode)
{
    /* 上层完成动作或报告故障时使用。切换状态同时撤销所有旧运动意图，
     * 下一次进入 walk/turn 前必须重新观测到摇杆回中。 */
    control_mode = mode;
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;
    active_direction = 0;
    center_seen = false;
    reset_motion_filter();
}

void behaviour_control_class::mode_set()
{
    if (control_task_init_bl == 0) return;

    remote_control_update();
    Remote_Message_Moniter(&remote_ctrl);

    /* 失联不触发新的起立/趴下动作。正在 walk/turn 时退回站立并清零指令；
     * 已经开始的 Get_up 保持原状态，让上层安全完成起立过渡。 */
    if (remote_ctrl.rc_lost) {
        vx = 0.0f;
        vy = 0.0f;
        wz = 0.0f;
        active_direction = 0;
        center_seen = false;
        button_input_ready = false;
        last_buttons = 0U;
        reset_motion_filter();

        if (control_mode == Dog_control_walk ||
            control_mode == Dog_control_turn) {
            control_mode = Dog_control_stand;
        }
        return;
    }

    uint16_t buttons = remote_ctrl.key.v;
    uint16_t rising = 0U;
    /* 首次上线或失联重连时先同步当前键值，不把一直按住的键误判为新按下；
     * 后续仅使用 0→1 上升沿，因此长按不会反复切换动作。 */
    if (!button_input_ready) {
        last_buttons = buttons;
        button_input_ready = true;
    } else {
        rising = (uint16_t)(buttons & (uint16_t)(~last_buttons));
        last_buttons = buttons;
    }

    /* 仅 START/SELECT 参与状态机，L1/L2/R2 等按键不再触发旧功能。
     * SELECT 优先，作为任何非 fault 状态下的趴下命令。 */
    if ((rising & PS2_BUTTON_SELECT_MASK) != 0U) {
        if (control_mode != Dog_control_fault) {
            Set_control_mode(Dog_control_Get_down);
        }
        return;
    }

    if ((rising & PS2_BUTTON_START_MASK) != 0U) {
        if (control_mode == Dog_control_system_init ||
            control_mode == Dog_control_Get_down) {
            Set_control_mode(Dog_control_Get_up);
        } else if (control_mode == Dog_control_stand ||
                   control_mode == Dog_control_walk ||
                   control_mode == Dog_control_turn) {
            Set_control_mode(Dog_control_stand);
        }
        return;
    }

    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;

    /* 只有已经直立的 stand/walk/turn 接受摇杆运动命令；过渡态和 fault
     * 始终输出零运动指令。vy 固定为零，避免向无侧向执行器的腿发送伪指令。 */
    if (control_mode != Dog_control_stand &&
        control_mode != Dog_control_walk &&
        control_mode != Dog_control_turn) {
        return;
    }

    fp32 raw_vx = rc_map(remote_ctrl.rc.ch[3]);
    fp32 raw_wz = rc_map(remote_ctrl.rc.ch[0]);
    bool vx_centered = (raw_vx == 0.0f);
    bool wz_centered = (raw_wz == 0.0f);

    /* 运动互锁：两轴同时回中才打开 center_seen。walk 中若请求反向或转弯，
     * turn 中若请求反向或前行，都先退到 stand 并关闭门闩；只有再次明确
     * 回中后才能进入新方向，避免一步内反扭腿或突然切换支撑相。 */
    if (vx_centered && wz_centered) {
        control_mode = Dog_control_stand;
        active_direction = 0;
        center_seen = true;
        reset_motion_filter();
        return;
    }

    if (control_mode == Dog_control_stand) {
        if (!center_seen) return;

        if (!vx_centered && wz_centered) {
            control_mode = Dog_control_walk;
            active_direction = (raw_vx > 0.0f) ? 1 : -1;
            center_seen = false;
        } else if (vx_centered && !wz_centered) {
            control_mode = Dog_control_turn;
            active_direction = (raw_wz > 0.0f) ? 1 : -1;
            center_seen = false;
        } else {
            center_seen = false;
            reset_motion_filter();
            return;
        }
    }

    if (control_mode == Dog_control_walk) {
        int8_t requested_direction = (raw_vx > 0.0f) ? 1 : -1;
        if (vx_centered || !wz_centered ||
            requested_direction != active_direction) {
            control_mode = Dog_control_stand;
            active_direction = 0;
            center_seen = false;
            reset_motion_filter();
            return;
        }

        vx = lpf_rc(raw_vx, &smooth_vx, RC_ALPHA_VX);
        return;
    }

    if (control_mode == Dog_control_turn) {
        int8_t requested_direction = (raw_wz > 0.0f) ? 1 : -1;
        if (wz_centered || !vx_centered ||
            requested_direction != active_direction) {
            control_mode = Dog_control_stand;
            active_direction = 0;
            center_seen = false;
            reset_motion_filter();
            return;
        }

        wz = lpf_rc(raw_wz, &smooth_wz, RC_ALPHA_WZ);
    }
}
