#include "robot_control.h"

#include "Curve.h"
#include "math_support.h"
#include "motor_task.h"
#include "posture_pace.h"

#include <math.h>

kinematics_connection_12bot_robot robot_dog_ik[ROBOT8_LEG_COUNT];

volatile uint8_t robot_control_fault_code = ROBOT_CONTROL_FAULT_NONE;
volatile uint8_t robot_control_fault_leg = 0xFFU;
volatile uint8_t robot_control_fault_joint = 0xFFU;
volatile uint8_t robot_control_getup_stage_watch = 0U;
volatile fp32 robot_control_fault_start_pos = 0.0f;
volatile fp32 robot_control_fault_target_pos = 0.0f;
volatile fp32 robot_control_fault_delta_pos = 0.0f;

static connection_dog_leg_curve gait_curve[ROBOT8_LEG_COUNT];

/* 腿序固定为 RF、LF、RH、LH；CAN1 接前腿，CAN2 接后腿。
 * 表内是各总线 motor_lz_data[] 的下标，不是 CAN ID（CAN ID=下标+1）。 */
static const uint8_t leg_can_channel[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG] = {
    {0U, 1U}, {2U, 3U}, {0U, 1U}, {2U, 3U},
};

static remote_control_mode active_mode = Dog_control_system_init;
/* 起立、趴下和回站姿都复用同一套“当前电机角 -> 目标电机角”过渡器。 */
static fp32 transition_start[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];
static fp32 transition_target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];
static fp32 transition_elapsed = 0.0f;
static uint8_t transition_prepared = 0U;
static uint8_t transition_done = 0U;
static uint8_t getup_stage = 0U;
static uint32_t pose_debug_retarget_seq_last = 0U;

static fp32 gait_phase = 0.0f;
static fp32 gait_ramp = 0.0f;

static Struct_send_motor_Lz *motor_send_ptr(uint8_t leg, uint8_t joint)
{
    uint8_t channel = leg_can_channel[leg][joint];
    if (leg < 2U) return &motor_lz_data_can1[channel].send;
    return &motor_lz_data_can2[channel].send;
}

static fp32 motor_recv_pos(uint8_t leg, uint8_t joint)
{
    uint8_t channel = leg_can_channel[leg][joint];
    if (leg < 2U) return motor_lz_data_can1[channel].recv.Now_Pos;
    return motor_lz_data_can2[channel].recv.Now_Pos;
}

static fp32 clampf(fp32 value, fp32 lower, fp32 upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static fp32 smoothstep5(fp32 u)
{
    u = clampf(u, 0.0f, 1.0f);
    /* 五次平滑步进：起点和终点的速度、加速度均为 0，避免突然发力。 */
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

static void latch_fault(uint8_t code, uint8_t leg)
{
    /* 只保留第一个故障，避免后续连锁错误覆盖真正根因。 */
    if (robot_control_fault_code == ROBOT_CONTROL_FAULT_NONE) {
        robot_control_fault_code = code;
        robot_control_fault_leg = leg;
    }
}

static void latch_target_limit_fault(uint8_t leg, uint8_t joint,
                                     fp32 start_pos, fp32 target_pos)
{
    robot_control_fault_joint = joint;
    robot_control_fault_start_pos = start_pos;
    robot_control_fault_target_pos = target_pos;
    robot_control_fault_delta_pos = fabsf(target_pos - start_pos);
    latch_fault(ROBOT_CONTROL_FAULT_TARGET_LIMIT, leg);
}

static uint8_t feedback_is_valid(void)
{
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        for (uint8_t joint = 0U; joint < ROBOT8_MOTORS_PER_LEG; ++joint) {
            if (!isfinite(motor_recv_pos(leg, joint))) return 0U;
        }
    }
    return 1U;
}

static void set_ik_reference_from_feedback(void)
{
    /* 每次进入新模式都用真实反馈重锚，首次 IK 才能选择离实机最近的装配支链。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        robot_dog_ik[leg].Set_Motor_Reference(
            motor_recv_pos(leg, 0U), motor_recv_pos(leg, 1U));
    }
}

static uint8_t solve_foot_targets(
    const fp32 foot[ROBOT8_LEG_COUNT][3],
    fp32 motor_target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG])
{
    /* 四条腿必须全部解算成功后，调用方才会统一提交电机目标。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        if (!robot_dog_ik[leg].leg_inverse_calculation(
                foot[leg][0], foot[leg][1], foot[leg][2])) {
            latch_fault(ROBOT_CONTROL_FAULT_IK, leg);
            return 0U;
        }

        motor_target[leg][0] = robot_dog_ik[leg].Get_Motor_Out_Pos1();
        motor_target[leg][1] = robot_dog_ik[leg].Get_Motor_Out_Pos2();

        for (uint8_t joint = 0U; joint < ROBOT8_MOTORS_PER_LEG; ++joint) {
            fp32 target = motor_target[leg][joint];
            if (!isfinite(target) || target < g_safe_pos_min || target > g_safe_pos_max) {
                latch_target_limit_fault(leg, joint, motor_recv_pos(leg, joint), target);
                return 0U;
            }
        }
    }
    return 1U;
}

static void send_motor_targets(
    const fp32 target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG],
    fp32 kp,
    fp32 kd)
{
    kp = clampf(kp, 0.0f, g_safe_kp_max);
    kd = clampf(kd, 0.0f, g_safe_kd_max);

    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        for (uint8_t joint = 0U; joint < ROBOT8_MOTORS_PER_LEG; ++joint) {
            Struct_send_motor_Lz *send = motor_send_ptr(leg, joint);
            send->Pos = target[leg][joint];
            /* 首版只使用电机内部位置阻抗：目标速度和前馈力矩均为 0。 */
            send->W = 0.0f;
            send->T_ff = 0.0f;
            send->Kp = kp;
            send->Kd = kd;
        }
    }
}

static uint8_t transition_configuration_is_valid(
    const fp32 target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG])
{
    /* 电机空间直线不一定始终对应有效闭链，因此发送前逐帧做一次正解校验。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        robot_dog_ik[leg].leg_correct_solution(
            target[leg][0], target[leg][1], 0.0f);
        if (!robot_dog_ik[leg].Get_Last_Target_Valid() ||
            !isfinite(robot_dog_ik[leg].Get_X()) ||
            !isfinite(robot_dog_ik[leg].Get_Z())) {
            latch_fault(ROBOT_CONTROL_FAULT_IK, leg);
            return 0U;
        }
    }
    return 1U;
}

static uint8_t prepare_transition(const fp32 foot_target[ROBOT8_LEG_COUNT][3])
{
    if (!feedback_is_valid()) {
        latch_fault(ROBOT_CONTROL_FAULT_FEEDBACK, 0xFFU);
        return 0U;
    }

    /* “自适应起立”的起点来自此刻的 8 个真实电机角，不依赖预设趴姿。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        transition_start[leg][0] = motor_recv_pos(leg, 0U);
        transition_start[leg][1] = motor_recv_pos(leg, 1U);
    }

    set_ik_reference_from_feedback();
    if (!solve_foot_targets(foot_target, transition_target)) return 0U;

    /* 零位或支链配置错误时，目标通常会离当前角很远；
     * 任一电机的角差超过门槛就拒绝整次动作。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        for (uint8_t joint = 0U; joint < ROBOT8_MOTORS_PER_LEG; ++joint) {
            if (g_pose_delta_limit_enable != 0U &&
                fabsf(transition_target[leg][joint]
                      - transition_start[leg][joint]) > g_pose_max_delta_rad) {
                latch_target_limit_fault(leg, joint,
                                         transition_start[leg][joint],
                                         transition_target[leg][joint]);
                return 0U;
            }
        }
    }

    transition_elapsed = 0.0f;
    transition_prepared = 1U;
    transition_done = 0U;
    return 1U;
}

static void run_pose_transition(const fp32 foot_target[ROBOT8_LEG_COUNT][3],
                                fp32 dt,
                                fp32 duration,
                                fp32 kp_start,
                                fp32 kd_start,
                                fp32 kp_end,
                                fp32 kd_end)
{
    if (robot_control_faulted()) return;

    if (g_pose_debug_retarget_request != 0U ||
        g_pose_debug_retarget_seq != pose_debug_retarget_seq_last) {
        /* Watch 在线改站姿参数后，置位 request 或改变 seq 都会强制重新解算。 */
        g_pose_debug_retarget_request = 0U;
        pose_debug_retarget_seq_last = g_pose_debug_retarget_seq;
        transition_elapsed = 0.0f;
        transition_prepared = 0U;
        transition_done = 0U;
    }

    if (!transition_prepared && !prepare_transition(foot_target)) return;

    if (duration < 0.1f) duration = 0.1f;
    transition_elapsed += clampf(dt, 0.0f, 0.02f);
    fp32 u = smoothstep5(transition_elapsed / duration);
    fp32 command[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];

    /* 在电机空间插值，确保每个电机从当前反馈连续走向同一姿态目标。 */
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        for (uint8_t joint = 0U; joint < ROBOT8_MOTORS_PER_LEG; ++joint) {
            command[leg][joint] = transition_start[leg][joint]
                + (transition_target[leg][joint] - transition_start[leg][joint]) * u;
        }
    }

    if (!transition_configuration_is_valid(command)) return;
    send_motor_targets(command,
                       kp_start + (kp_end - kp_start) * u,
                       kd_start + (kd_end - kd_start) * u);

    if (transition_elapsed >= duration) {
        transition_done = 1U;
        send_motor_targets(transition_target, kp_end, kd_end);
    }
}

static void apply_gait_foot_targets(const fp32 foot[ROBOT8_LEG_COUNT][3])
{
    fp32 target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];
    if (solve_foot_targets(foot, target)) {
        send_motor_targets(target, g_walk_kp, g_walk_kd);
    }
}

void robot_control_init(void)
{
    robot_dog_ik[ROBOT8_RF].Init(1U, DH1);
    robot_dog_ik[ROBOT8_LF].Init(2U, DH2);
    robot_dog_ik[ROBOT8_RH].Init(3U, DH3);
    robot_dog_ik[ROBOT8_LH].Init(4U, DH4);

    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        gait_curve[leg].curve_init(g_gait_swing_ratio, 0.0f, 0.0f);
    }

    if (feedback_is_valid()) set_ik_reference_from_feedback();
    robot_control_enter_mode(Dog_control_system_init);
}

void robot_control_enter_mode(remote_control_mode mode)
{
    active_mode = mode;
    transition_elapsed = 0.0f;
    transition_prepared = 0U;
    transition_done = 0U;
    getup_stage = 0U;
    robot_control_getup_stage_watch = 0U;
    /* 清零相位和渐入系数，防止模式切换沿用旧步态造成足端跳变。 */
    gait_phase = 0.0f;
    gait_ramp = 0.0f;

    if (feedback_is_valid()) set_ik_reference_from_feedback();
}

void robot_control_safe_hold(void)
{
    fp32 target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];
    if (!feedback_is_valid()) return;

    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        target[leg][0] = motor_recv_pos(leg, 0U);
        target[leg][1] = motor_recv_pos(leg, 1U);
    }
    /* Kp=0：不主动拉向旧目标；仅保留 Kd 阻尼，适用于上电等待状态。 */
    send_motor_targets(target, 0.0f, g_safe_hold_kd);
}

void robot_control_Dog_control_Get_down(fp32 dt)
{
    run_pose_transition(robot_down_foot, dt, g_getdown_duration_s,
                        g_stand_kp, g_stand_kd, g_down_kp, g_down_kd);
}

void robot_control_Dog_control_Get_up(fp32 dt)
{
    robot_control_getup_stage_watch = getup_stage;
    if (getup_stage == 0U) {
        /* 先收敛到可达的趴姿高度，再进入最终站姿；避免当前趴姿到站姿
         * 单电机角差超过 g_pose_max_delta_rad 后直接触发目标保护。 */
        run_pose_transition(robot_down_foot, dt, 1.2f,
                            g_getup_kp_start, g_getup_kd_start,
                            g_getup_kp_start, g_getup_kd_start);
        if (transition_done != 0U) {
            getup_stage = 1U;
            robot_control_getup_stage_watch = getup_stage;
            transition_elapsed = 0.0f;
            transition_prepared = 0U;
            transition_done = 0U;
        }
        return;
    }

    run_pose_transition(robot_stand_foot, dt, g_getup_duration_s,
                        g_getup_kp_start, g_getup_kd_start,
                        g_stand_kp, g_stand_kd);
}

void robot_control_Dog_control_stand(fp32 dt)
{
    run_pose_transition(robot_stand_foot, dt, g_stand_transition_s,
                        g_stand_kp, g_stand_kd,
                        g_stand_kp, g_stand_kd);
}

static void run_gait(fp32 dt, fp32 command, uint8_t turn_in_place)
{
    if (robot_control_faulted()) return;

    command = clampf(command, -1.0f, 1.0f);
    if (g_gait_joystick_speed_enable == 0U && fabsf(command) > 0.0f) {
        /* 调试期摇杆只保留方向，不用幅度调速度，避免一进 walk 就给过大步态。 */
        fp32 fixed = clampf(g_gait_fixed_command, 0.05f, 1.0f);
        command = (command > 0.0f) ? fixed : -fixed;
    }
    fp32 magnitude = fabsf(command);
    /* 摇杆越大周期越短；这里的 period 单位是秒，不是相位增量。 */
    fp32 period = g_gait_period_max_s
        - (g_gait_period_max_s - g_gait_period_min_s) * magnitude;
    period = clampf(period, g_gait_period_min_s, g_gait_period_max_s);

    if (g_gait_ramp_s < 0.1f) g_gait_ramp_s = 0.1f;
    /* 步长和步高从 0 缓慢渐入，首帧只产生极小偏移，避免直接跳到完整步幅。 */
    gait_ramp = clampf(gait_ramp + dt / g_gait_ramp_s, 0.0f, 1.0f);
    gait_phase += dt / period;
    while (gait_phase >= 1.0f) gait_phase -= 1.0f;

    fp32 foot[ROBOT8_LEG_COUNT][3];
    for (uint8_t leg = 0U; leg < ROBOT8_LEG_COUNT; ++leg) {
        /* 前行时四腿使用同号步长，对角相位决定各腿当前处于摆动还是支撑；
         * 原地转弯时左右侧步长反号。 */
        fp32 side = 1.0f;
        fp32 step = command * g_walk_step_max_mm * gait_ramp;
        if (turn_in_place) {
            side = (leg == ROBOT8_RF || leg == ROBOT8_RH) ? 1.0f : -1.0f;
            step = command * g_turn_step_max_mm * gait_ramp * side;
        }

        fp32 start[3] = {
            robot_stand_foot[leg][0] - 0.5f * step,
            robot_stand_foot[leg][1],
            robot_stand_foot[leg][2],
        };
        fp32 end[3] = {
            robot_stand_foot[leg][0] + 0.5f * step,
            robot_stand_foot[leg][1],
            robot_stand_foot[leg][2],
        };

        /* 相位偏置固定为对角组，不随摆动占空比变化。 */
        fp32 phase = gait_phase + robot_gait_phase_offset[leg];
        if (phase >= 1.0f) phase -= 1.0f;
        gait_curve[leg].curve(phase, start, end, g_gait_height_mm * gait_ramp);

        foot[leg][0] = gait_curve[leg].Get_X();
        foot[leg][1] = gait_curve[leg].Get_Y();
        foot[leg][2] = gait_curve[leg].Get_Z();
    }

    apply_gait_foot_targets(foot);
}

void robot_control_Dog_control_walk(fp32 dt, fp32 forward_command)
{
    run_gait(dt, forward_command, 0U);
}

void robot_control_Dog_control_turn(fp32 dt, fp32 turn_command)
{
    run_gait(dt, turn_command, 1U);
}

uint8_t robot_control_pose_done(void)
{
    return transition_done;
}

uint8_t robot_control_faulted(void)
{
    return (robot_control_fault_code != ROBOT_CONTROL_FAULT_NONE) ? 1U : 0U;
}

void robot_control_Dog_control_debug(void)
{
    /* 该接口绕过 1.2 rad 逐电机角差门槛和姿态过渡，生产状态机不可达。 */
    fp32 target[ROBOT8_LEG_COUNT][ROBOT8_MOTORS_PER_LEG];
    if (solve_foot_targets(xyz_Dog_control_debug, target)) {
        send_motor_targets(target, 4.0f, 0.9f);
    }
}
