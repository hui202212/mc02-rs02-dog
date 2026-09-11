#include "robot_application.h"

#include "fivebar.h"
#include "motor_task.h"
#include "robot_io.h"
#include "remote_control.h"
#include "simple_gait.h"
#if SIMPLE_IMU_BALANCE_ENABLE
#include "imu_task.h"
#endif

#include <math.h>

/*
 * 这是简单版上层总控，1 ms 执行一次：
 *
 * 手柄 -> 状态机 -> 姿态/步态 -> 四脚位置 -> 五连杆逆解 -> 8 个电机目标
 *
 * 它不直接发送 CAN，只把 Pos/Kp/Kd 写进 motor_task 的命令缓冲。
 */

/*========================== Keil Watch 观察变量 ==========================*/
volatile uint8_t simple_robot_state_watch = SIMPLE_STATE_WAIT;
volatile uint8_t simple_robot_fault_watch = SIMPLE_FAULT_NONE;
volatile uint8_t simple_robot_fault_leg_watch = 0xFFU;
volatile uint8_t simple_robot_fault_motor_watch = 0xFFU;
volatile uint8_t simple_robot_obstacle_mode_watch = SIMPLE_OBSTACLE_NORMAL;
volatile fp32 simple_robot_side_z_comp_mm_watch = 0.0f;
volatile uint8_t simple_robot_imu_ready_watch = 0U;
volatile fp32 simple_robot_roll_raw_watch = 0.0f;
volatile fp32 simple_robot_roll_zero_watch = 0.0f;
volatile fp32 simple_robot_roll_relative_watch = 0.0f;
volatile fp32 simple_robot_imu_comp_add_mm_watch = 0.0f;
volatile uint8_t simple_robot_jump_mode_watch = 0xFFU;
volatile uint8_t simple_robot_jump_stage_watch = 0xFFU;
volatile uint8_t simple_robot_jump_abort_watch = 0U;
volatile uint32_t simple_robot_jump_count_watch = 0U;
volatile uint8_t simple_robot_dance_stage_watch = 0xFFU;
volatile uint32_t simple_robot_dance_count_watch = 0U;
volatile fp32 simple_robot_foot_x_watch[SIMPLE_LEG_COUNT] = {0.0f};
volatile fp32 simple_robot_foot_z_watch[SIMPLE_LEG_COUNT] = {0.0f};
volatile fp32 simple_robot_motor_target_watch[SIMPLE_LEG_COUNT][2] = {{0.0f}};

static SimpleRobotState robot_state = SIMPLE_STATE_WAIT;
static FiveBarState fivebar[SIMPLE_LEG_COUNT];
static SimpleGaitState gait;

/* 起立、站立、趴下共用一套“当前角 -> 目标角”平滑过渡器。 */
static fp32 pose_start[SIMPLE_LEG_COUNT][2];
static fp32 pose_target[SIMPLE_LEG_COUNT][2];
static fp32 pose_elapsed = 0.0f;
static uint8_t pose_prepared = 0U;
static uint8_t pose_done = 0U;
static uint8_t getup_stage = 0U;
static uint8_t jump_stage = 0xFFU;
static SimpleJumpMode jump_mode = SIMPLE_JUMP_VERTICAL;
static uint8_t dance_stage = 0xFFU;

static uint16_t last_buttons = 0U;
static uint8_t buttons_ready = 0U;
static uint8_t motion_can_start = 0U;
static int8_t motion_direction = 0;
static fp32 forward_command = 0.0f;
static fp32 turn_command = 0.0f;
static SimpleObstacleMode obstacle_mode = SIMPLE_OBSTACLE_NORMAL;
static fp32 side_z_comp_current = 0.0f;
static uint8_t roll_zero_valid = 0U;
static fp32 roll_zero = 0.0f;

/* 装配造成的单腿前后偏差只在逆解入口修正，不污染公共步态轨迹。 */
static const fp32 leg_x_trim[SIMPLE_LEG_COUNT] = {
    SIMPLE_RF_X_TRIM_MM, SIMPLE_LF_X_TRIM_MM,
    SIMPLE_RH_X_TRIM_MM, SIMPLE_LH_X_TRIM_MM,
};

/* 小量单腿高度偏差在逆解入口统一补偿，不修改电机软件零位。 */
static const fp32 leg_z_trim[SIMPLE_LEG_COUNT] = {
    SIMPLE_RF_Z_TRIM_MM, SIMPLE_LF_Z_TRIM_MM,
    SIMPLE_RH_Z_TRIM_MM, SIMPLE_LH_Z_TRIM_MM,
};

/* Bench procedures need raw bus/index access. Production motion control uses
 * robot_io exclusively; these helpers only exist when a bench feature is
 * explicitly enabled in motor_task.h. */
#if ROBOT_ALL_LEG_SYNC_TEST_ENABLE || ROBOT_COAXIAL_PAIR_TEST_ENABLE || \
    ROBOT_MOTOR_CALIBRATION_ENABLE
static const uint8_t bench_leg_bus[SIMPLE_LEG_COUNT] = {
    SIMPLE_RF_BUS, SIMPLE_LF_BUS, SIMPLE_RH_BUS, SIMPLE_LH_BUS,
};

static const uint8_t bench_leg_motor_index[SIMPLE_LEG_COUNT][2] = {
    {SIMPLE_RF_MOTOR1_INDEX, SIMPLE_RF_MOTOR2_INDEX},
    {SIMPLE_LF_MOTOR1_INDEX, SIMPLE_LF_MOTOR2_INDEX},
    {SIMPLE_RH_MOTOR1_INDEX, SIMPLE_RH_MOTOR2_INDEX},
    {SIMPLE_LH_MOTOR1_INDEX, SIMPLE_LH_MOTOR2_INDEX},
};

static motor_lz_control *motor_data(uint8_t leg, uint8_t motor)
{
    const uint8_t index = bench_leg_motor_index[leg][motor];
    return bench_leg_bus[leg] == 0U ? &motor_lz_data_can1[index]
                                    : &motor_lz_data_can2[index];
}
#endif

static fp32 clamp(fp32 value, fp32 low, fp32 high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static fp32 move_toward(fp32 current, fp32 target, fp32 max_delta)
{
    fp32 delta = target - current;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return current + delta;
}

#if SIMPLE_IMU_BALANCE_ENABLE
static fp32 imu_roll_input(void)
{
    return IMU.Roll;
}

static void update_imu_watch(void)
{
    simple_robot_imu_ready_watch = IMU.imu_flag;
    fp32 raw = imu_roll_input();
    if (isfinite(raw)) simple_robot_roll_raw_watch = raw;
}
#endif

static fp32 compute_side_comp_target(void)
{
    /* 比赛时采用固定差高，避免IMU零点或方向错误改变腿长。 */
    simple_robot_roll_relative_watch = 0.0f;
    simple_robot_imu_comp_add_mm_watch = 0.0f;
    return clamp(SIMPLE_CROSS_SLOPE_FIXED_COMP_MM,
                 0.0f, SIMPLE_CROSS_SLOPE_COMP_MAX_MM);
}

static fp32 smoothstep5(fp32 u)
{
    /* 五次平滑插值：起点和终点的速度、加速度都为 0。 */
    u = clamp(u, 0.0f, 1.0f);
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

static void latch_fault(uint8_t fault, uint8_t leg, uint8_t motor)
{
    /* 只保留第一个故障，后续连锁错误不能覆盖真正根因。 */
    if (simple_robot_fault_watch != SIMPLE_FAULT_NONE) return;
    simple_robot_fault_watch = fault;
    simple_robot_fault_leg_watch = leg;
    simple_robot_fault_motor_watch = motor;
#if ROBOT_GLOBAL_FAILSAFE_ENABLE
    robot_state = SIMPLE_STATE_FAULT;
    simple_robot_state_watch = SIMPLE_STATE_FAULT;
#else
    /* 竞赛非锁存模式只记录原因；本次无效目标不提交，八电机保持上一帧命令。 */
#endif
}

static uint8_t feedback_valid(void)
{
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            if (!robot_io_joint_feedback_valid(leg, motor)) return 0U;
        }
    }
    return 1U;
}

static void reference_ik_from_feedback(void)
{
    /* 每次换状态都用真实反馈重设逆解参考，避免第一帧目标突跳。 */
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        fivebar_set_reference(&fivebar[leg],
                              robot_io_joint_position(leg, 0U),
                              robot_io_joint_position(leg, 1U));
    }
}

static void send_targets(const fp32 target[SIMPLE_LEG_COUNT][2], fp32 kp, fp32 kd)
{
    /* 第一版只用电机内部位置阻抗：目标速度和前馈力矩固定为 0。 */
    kp = clamp(kp, 0.0f, SIMPLE_KP_MAX);
    kd = clamp(kd, 0.0f, SIMPLE_KD_MAX);
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        fp32 leg_kp = kp;
        fp32 leg_kd = kd;
        if (robot_state == SIMPLE_STATE_STAND &&
            (leg == SIMPLE_RF || leg == SIMPLE_RH)) {
            /* 静止承重时右侧更易下沉，只加强右前和右后。 */
            leg_kp = clamp(SIMPLE_RIGHT_STAND_KP, 0.0f, SIMPLE_KP_MAX);
            leg_kd = clamp(SIMPLE_RIGHT_STAND_KD, 0.0f, SIMPLE_KD_MAX);
        }
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            robot_io_set_joint_target(leg, motor, target[leg][motor], 0.0f,
                                      leg_kp, leg_kd, 0.0f);
            simple_robot_motor_target_watch[leg][motor] = target[leg][motor];
        }
    }
}

static void safe_hold(void)
{
    /* WAIT 状态 Kp=0，不主动拉位置；只保留 Kd 阻尼。 */
    fp32 current[SIMPLE_LEG_COUNT][2];
    if (!feedback_valid()) return;
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        current[leg][0] = robot_io_joint_position(leg, 0U);
        current[leg][1] = robot_io_joint_position(leg, 1U);
    }
    send_targets(current, 0.0f, SIMPLE_SAFE_HOLD_KD);
}

static uint8_t solve_feet(const SimpleFootTarget foot[SIMPLE_LEG_COUNT],
                          fp32 target[SIMPLE_LEG_COUNT][2])
{
    /* 四只脚全部逆解成功后，调用者才会统一提交 8 个目标。 */
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        fp32 corrected_x = foot[leg].x + leg_x_trim[leg];
        fp32 corrected_z = foot[leg].z_down + leg_z_trim[leg];
        if (robot_state == SIMPLE_STATE_STAND) {
            /* 带载站立调平：抬高右侧机身，同时压低左侧机身。 */
            corrected_z += (leg == SIMPLE_RF || leg == SIMPLE_RH)
                         ? SIMPLE_STAND_SIDE_Z_COMP_MM
                         : -SIMPLE_STAND_SIDE_Z_COMP_MM;
        }
        simple_robot_foot_x_watch[leg] = corrected_x;
        simple_robot_foot_z_watch[leg] = corrected_z;
        if (!fivebar_inverse(&fivebar[leg], leg, corrected_x,
                             corrected_z, target[leg])) {
            latch_fault(SIMPLE_FAULT_IK, leg, 0xFFU);
            return 0U;
        }
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            if (!isfinite(target[leg][motor]) ||
                target[leg][motor] < SIMPLE_POSITION_MIN_RAD ||
                target[leg][motor] > SIMPLE_POSITION_MAX_RAD) {
                latch_fault(SIMPLE_FAULT_TARGET_LIMIT, leg, motor);
                return 0U;
            }
        }
    }
    return 1U;
}

static uint8_t prepare_foot_pose(const SimpleFootTarget foot[SIMPLE_LEG_COUNT])
{
    /* 捕获真实起点，并一次性计算该姿态对应的 8 个终点角。 */
    if (!feedback_valid()) {
        latch_fault(SIMPLE_FAULT_FEEDBACK, 0xFFU, 0xFFU);
        return 0U;
    }

    reference_ik_from_feedback();
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        pose_start[leg][0] = robot_io_joint_position(leg, 0U);
        pose_start[leg][1] = robot_io_joint_position(leg, 1U);
    }
    if (!solve_feet(foot, pose_target)) return 0U;

    /* 零位/方向错时目标通常离当前角很远，超过 1.2 rad 就拒绝整次动作。 */
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            if (SIMPLE_MAX_POSE_DELTA_RAD > 0.0f &&
                fabsf(pose_target[leg][motor] - pose_start[leg][motor]) >
                    SIMPLE_MAX_POSE_DELTA_RAD) {
                latch_fault(SIMPLE_FAULT_TARGET_LIMIT, leg, motor);
                return 0U;
            }
        }
    }

    pose_elapsed = 0.0f;
    pose_prepared = 1U;
    pose_done = 0U;
    return 1U;
}

static uint8_t prepare_pose(fp32 foot_x, fp32 foot_z_down)
{
    SimpleFootTarget foot[SIMPLE_LEG_COUNT];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].x = foot_x;
        foot[leg].z_down = foot_z_down;
    }
    return prepare_foot_pose(foot);
}

static void run_prepared_pose(fp32 duration,
                              fp32 start_kp, fp32 start_kd,
                              fp32 end_kp, fp32 end_kd)
{
    /* 在电机角空间平滑插值，每一帧同时验证五连杆闭链仍然成立。 */
    pose_elapsed += SIMPLE_CONTROL_DT_S;
    fp32 progress = smoothstep5(pose_elapsed / duration);
    fp32 command[SIMPLE_LEG_COUNT][2];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            command[leg][motor] = pose_start[leg][motor]
                + (pose_target[leg][motor] - pose_start[leg][motor]) * progress;
        }
        if (SIMPLE_POSE_GEOMETRY_GUARD &&
            !fivebar_motor_pose_valid(leg, command[leg][0], command[leg][1])) {
            latch_fault(SIMPLE_FAULT_IK, leg, 0xFFU);
            return;
        }
    }

    send_targets(command,
                 start_kp + (end_kp - start_kp) * progress,
                 start_kd + (end_kd - start_kd) * progress);
    if (pose_elapsed >= duration) {
        pose_done = 1U;
        send_targets(pose_target, end_kp, end_kd);
    }
}

static void run_pose(fp32 foot_x, fp32 foot_z_down, fp32 duration,
                     fp32 start_kp, fp32 start_kd, fp32 end_kp, fp32 end_kd)
{
    if (!pose_prepared && !prepare_pose(foot_x, foot_z_down)) return;
    run_prepared_pose(duration, start_kp, start_kd, end_kp, end_kd);
}

static void run_foot_pose(const SimpleFootTarget foot[SIMPLE_LEG_COUNT],
                          fp32 duration, fp32 kp, fp32 kd)
{
    if (!pose_prepared && !prepare_foot_pose(foot)) return;
    run_prepared_pose(duration, kp, kd, kp, kd);
}

static void enter_state(SimpleRobotState state);

static void advance_dance_stage(void)
{
    dance_stage++;
    simple_robot_dance_stage_watch = dance_stage;
    pose_elapsed = 0.0f;
    pose_prepared = 0U;
    pose_done = 0U;
    if (feedback_valid()) reference_ik_from_feedback();
}

static void run_dance_sequence(void)
{
    /* 单腿小抬、三腿支撑，按右前、左前、右后、左后再反向完成两轮。 */
    static const uint8_t leg_order[SIMPLE_DANCE_STAGE_COUNT] = {
        SIMPLE_RF, SIMPLE_LF, SIMPLE_RH, SIMPLE_LH,
        SIMPLE_LH, SIMPLE_RH, SIMPLE_LF, SIMPLE_RF,
    };
    SimpleFootTarget foot[SIMPLE_LEG_COUNT];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].x = SIMPLE_STAND_X_MM;
        foot[leg].z_down = SIMPLE_STAND_Z_DOWN_MM;
    }

    if (dance_stage >= SIMPLE_DANCE_STAGE_COUNT) {
        simple_robot_dance_count_watch++;
        enter_state(SIMPLE_STATE_STAND);
        return;
    }

    foot[leg_order[dance_stage]].z_down = SIMPLE_DANCE_LIFT_Z_MM;
    run_foot_pose(foot, SIMPLE_DANCE_LIFT_S,
                  SIMPLE_DANCE_KP, SIMPLE_DANCE_KD);
    if (pose_done) {
        if (dance_stage + 1U < SIMPLE_DANCE_STAGE_COUNT) {
            advance_dance_stage();
        } else {
            simple_robot_dance_count_watch++;
            enter_state(SIMPLE_STATE_STAND);
        }
    }
}

static void enter_state(SimpleRobotState state)
{
    /* 换状态必须清掉上一次的姿态进度、步态相位和运动指令。 */
    robot_state = state;
    simple_robot_state_watch = (uint8_t)state;
    pose_elapsed = 0.0f;
    pose_prepared = 0U;
    pose_done = 0U;
    getup_stage = 0U;
    forward_command = 0.0f;
    turn_command = 0.0f;
    simple_gait_reset(&gait);
    if (state == SIMPLE_STATE_JUMP) {
        jump_stage = 0U;
        simple_robot_jump_mode_watch = (uint8_t)jump_mode;
        simple_robot_jump_stage_watch = 0U;
        simple_robot_jump_abort_watch = 0U;
    } else {
        jump_stage = 0xFFU;
        simple_robot_jump_stage_watch = 0xFFU;
    }
    if (state == SIMPLE_STATE_DANCE) {
        dance_stage = 0U;
        simple_robot_dance_stage_watch = 0U;
    } else {
        dance_stage = 0xFFU;
        simple_robot_dance_stage_watch = 0xFFU;
    }
    if (state != SIMPLE_STATE_CRAWL) {
        /* 趴姿和站姿始终四腿等高；重新进入步态后再平滑建立坡度差。 */
        side_z_comp_current = 0.0f;
        simple_robot_side_z_comp_mm_watch = 0.0f;
    }
    if (feedback_valid()) reference_ik_from_feedback();
}

#if SIMPLE_OBSTACLE_MODE_ENABLE
static void select_obstacle_mode(SimpleObstacleMode mode)
{
    obstacle_mode = mode;
    simple_robot_obstacle_mode_watch = (uint8_t)mode;
    roll_zero_valid = 0U;
    simple_robot_roll_relative_watch = 0.0f;
    simple_robot_imu_comp_add_mm_watch = 0.0f;
#if SIMPLE_IMU_BALANCE_ENABLE
    if (mode == SIMPLE_OBSTACLE_CROSS_SLOPE_10 &&
        IMU.imu_flag && isfinite(imu_roll_input())) {
        roll_zero = imu_roll_input();
        roll_zero_valid = 1U;
        simple_robot_roll_zero_watch = roll_zero;
    }
#endif
    side_z_comp_current = 0.0f;
    simple_robot_side_z_comp_mm_watch = 0.0f;

    /* 切模式时先停步回到等高低姿；摇杆保持时会自动重新开始。 */
    if (robot_state == SIMPLE_STATE_CRAWL) {
        enter_state(SIMPLE_STATE_GET_DOWN);
    } else if (robot_state == SIMPLE_STATE_WALK || robot_state == SIMPLE_STATE_TURN) {
        enter_state(SIMPLE_STATE_STAND);
    }
}
#endif

#if SIMPLE_JUMP_ENABLE
static uint8_t motors_near_pose_target(fp32 tolerance)
{
    /* 参考实机跳跃项目：八个电机基本同步到达预压位置后才允许蹬伸。 */
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            fp32 now = robot_io_joint_position(leg, motor);
            if (!isfinite(now) || fabsf(now - pose_target[leg][motor]) > tolerance) {
                return 0U;
            }
        }
    }
    return 1U;
}

static void advance_jump_stage(void)
{
    jump_stage++;
    simple_robot_jump_stage_watch = jump_stage;
    pose_elapsed = 0.0f;
    pose_prepared = 0U;
    pose_done = 0U;
    if (feedback_valid()) reference_ik_from_feedback();
}

static void set_step_down_shift_pose(SimpleFootTarget foot[SIMPLE_LEG_COUNT])
{
    foot[SIMPLE_RF].x = SIMPLE_T_DOWN_SHIFT_FRONT_X_MM;
    foot[SIMPLE_RF].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
    foot[SIMPLE_LF].x = SIMPLE_T_DOWN_SHIFT_FRONT_X_MM;
    foot[SIMPLE_LF].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
    foot[SIMPLE_RH].x = SIMPLE_T_DOWN_SHIFT_REAR_X_MM;
    foot[SIMPLE_RH].z_down = SIMPLE_STAND_Z_DOWN_MM;
    foot[SIMPLE_LH].x = SIMPLE_T_DOWN_SHIFT_REAR_X_MM;
    foot[SIMPLE_LH].z_down = SIMPLE_STAND_Z_DOWN_MM;
}

static void set_step_up_shift_pose(SimpleFootTarget foot[SIMPLE_LEG_COUNT])
{
    foot[SIMPLE_RF].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
    foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
    foot[SIMPLE_LF].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
    foot[SIMPLE_LF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
    foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_REAR_X_MM;
    foot[SIMPLE_RH].z_down = SIMPLE_STAND_Z_DOWN_MM;
    foot[SIMPLE_LH].x = SIMPLE_T_UP_SHIFT_REAR_X_MM;
    foot[SIMPLE_LH].z_down = SIMPLE_STAND_Z_DOWN_MM;
}

static void run_step_up_sequence(void)
{
    /* 每次只上100mm一级：前腿上台、机身前移、后腿上台，全程无腾空。 */
    SimpleFootTarget foot[SIMPLE_LEG_COUNT];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].x = SIMPLE_STAND_X_MM;
        foot[leg].z_down = SIMPLE_STAND_Z_DOWN_MM;
    }

    fp32 duration = SIMPLE_T_UP_LOWER_S;
    switch (jump_stage) {
    case 0U: /* 右前近似竖直抬到台沿附近。 */
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_PRELIFT_Z_MM;
        duration = SIMPLE_T_UP_LIFT_S;
        break;
    case 1U: /* 右前继续抬高并摆到上一级。 */
        foot[SIMPLE_RF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SWING_Z_MM;
        duration = SIMPLE_T_UP_SWING_S;
        break;
    case 2U: /* 右前竖直落到上一级。 */
        foot[SIMPLE_RF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        break;
    case 3U: /* 右前支撑，左前近似竖直抬脚。 */
        foot[SIMPLE_RF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LF].z_down = SIMPLE_T_UP_PRELIFT_Z_MM;
        duration = SIMPLE_T_UP_LIFT_S;
        break;
    case 4U: /* 左前继续抬高并摆到上一级。 */
        foot[SIMPLE_RF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_LF].z_down = SIMPLE_T_UP_SWING_Z_MM;
        duration = SIMPLE_T_UP_SWING_S;
        break;
    case 5U: /* 左前竖直落脚。 */
        foot[SIMPLE_RF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LF].x = SIMPLE_T_UP_STEP_X_MM;
        foot[SIMPLE_LF].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        break;
    case 6U: /* 前腿在上一级支撑，机身缓慢前移。 */
        set_step_up_shift_pose(foot);
        duration = SIMPLE_T_UP_SHIFT_S;
        break;
    case 7U: /* 右后在下一级原位置抬脚。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_PRELIFT_Z_MM;
        duration = SIMPLE_T_UP_LIFT_S;
        break;
    case 8U: /* 右后摆到上一级。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_SWING_Z_MM;
        duration = SIMPLE_T_UP_SWING_S;
        break;
    case 9U: /* 右后竖直落脚。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        break;
    case 10U: /* 右后支撑，左后在下一级原位置抬脚。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_UP_PRELIFT_Z_MM;
        duration = SIMPLE_T_UP_LIFT_S;
        break;
    case 11U: /* 左后摆到上一级。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_UP_SWING_Z_MM;
        duration = SIMPLE_T_UP_SWING_S;
        break;
    case 12U: /* 左后竖直落脚。 */
        set_step_up_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        foot[SIMPLE_LH].x = SIMPLE_T_UP_SHIFT_FRONT_X_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_UP_SUPPORT_Z_MM;
        break;
    case 13U: /* 四脚都在上一级，缓慢恢复正常站姿。 */
        duration = SIMPLE_T_UP_RECOVER_S;
        break;
    default:
        simple_robot_jump_abort_watch = 5U;
        enter_state(SIMPLE_STATE_STAND);
        return;
    }

    run_foot_pose(foot, duration, SIMPLE_T_UP_STEP_KP, SIMPLE_T_UP_STEP_KD);
    if (!pose_done) return;
    if (jump_stage < 13U) {
        advance_jump_stage();
    } else {
        simple_robot_jump_count_watch++;
        enter_state(SIMPLE_STATE_STAND);
    }
}

static void run_step_down_sequence(void)
{
    /* 每次只挪一条腿；后腿分成原地抬、向前摆、向下找地，避免大跨步掀翻。 */
    SimpleFootTarget foot[SIMPLE_LEG_COUNT];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].x = SIMPLE_STAND_X_MM;
        foot[leg].z_down = SIMPLE_STAND_Z_DOWN_MM;
    }

    fp32 duration = SIMPLE_T_DOWN_REACH_S;
    switch (jump_stage) {
    case 0U: /* 右前腿大步抬起并伸到台沿外。 */
        foot[SIMPLE_RF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_DOWN_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_LIFT_S;
        break;
    case 1U: /* 右前腿缓慢向下找地。 */
        foot[SIMPLE_RF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_DOWN_REACH_Z_MM;
        break;
    case 2U: /* 右前支撑，左前腿抬到台沿外。 */
        foot[SIMPLE_RF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
        foot[SIMPLE_LF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_LF].z_down = SIMPLE_T_DOWN_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_LIFT_S;
        break;
    case 3U: /* 左前腿缓慢向下找地。 */
        foot[SIMPLE_RF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_RF].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
        foot[SIMPLE_LF].x = SIMPLE_T_DOWN_STEP_X_MM;
        foot[SIMPLE_LF].z_down = SIMPLE_T_DOWN_REACH_Z_MM;
        break;
    case 4U: /* 前腿在下一级支撑，四脚共同把机身缓慢推过台沿。 */
        set_step_down_shift_pose(foot);
        duration = SIMPLE_T_DOWN_SHIFT_S;
        break;
    case 5U: /* 右后腿先在原位置小幅抬起。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_SHIFT_REAR_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_REAR_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_REAR_LIFT_S;
        break;
    case 6U: /* 右后腿保持小抬高度，收敛地摆过台沿。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_REAR_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_REAR_SWING_S;
        break;
    case 7U: /* 右后腿缓慢向下找地。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_REACH_Z_MM;
        break;
    case 8U: /* 右后支撑，左后腿先在原位置小幅抬起。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
        foot[SIMPLE_LH].x = SIMPLE_T_DOWN_SHIFT_REAR_X_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_DOWN_REAR_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_REAR_LIFT_S;
        break;
    case 9U: /* 左后腿保持小抬高度，收敛地摆过台沿。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
        foot[SIMPLE_LH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_DOWN_REAR_LIFT_Z_MM;
        duration = SIMPLE_T_DOWN_REAR_SWING_S;
        break;
    case 10U: /* 左后腿缓慢向下找地。 */
        set_step_down_shift_pose(foot);
        foot[SIMPLE_RH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_RH].z_down = SIMPLE_T_DOWN_SUPPORT_Z_MM;
        foot[SIMPLE_LH].x = SIMPLE_T_DOWN_REAR_STEP_X_MM;
        foot[SIMPLE_LH].z_down = SIMPLE_T_DOWN_REACH_Z_MM;
        break;
    case 11U: /* 四脚都到下一级后恢复正常站姿。 */
        duration = SIMPLE_T_DOWN_RECOVER_S;
        break;
    default:
        simple_robot_jump_abort_watch = 3U;
        enter_state(SIMPLE_STATE_STAND);
        return;
    }

    run_foot_pose(foot, duration, SIMPLE_T_DOWN_KP, SIMPLE_T_DOWN_KD);
    if (!pose_done) return;
    if (jump_stage < 11U) {
        advance_jump_stage();
    } else {
        simple_robot_jump_count_watch++;
        enter_state(SIMPLE_STATE_STAND);
    }
}

static void set_bridge_b_gap_shift_pose(SimpleFootTarget foot[SIMPLE_LEG_COUNT])
{
    fp32 half_step = 0.5f * SIMPLE_BRIDGE_B_GAP_STEP_X_MM;
    foot[SIMPLE_RF].x = half_step;
    foot[SIMPLE_LF].x = half_step;
    foot[SIMPLE_RH].x = -half_step;
    foot[SIMPLE_LH].x = -half_step;
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].z_down = SIMPLE_BRIDGE_B_Z_DOWN_MM;
    }
}

static void run_bridge_b_gap_sequence(void)
{
    /* 400mm踏板之间有150mm缝：每条腿先竖直抬，再水平跨，再竖直落。 */
    SimpleFootTarget foot[SIMPLE_LEG_COUNT];
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        foot[leg].x = SIMPLE_STAND_X_MM;
        foot[leg].z_down = SIMPLE_BRIDGE_B_Z_DOWN_MM;
    }

    fp32 duration = SIMPLE_BRIDGE_B_GAP_LOWER_S;
    fp32 step = SIMPLE_BRIDGE_B_GAP_STEP_X_MM;
    fp32 half_step = 0.5f * step;
    switch (jump_stage) {
    case 0U: /* 右前原地竖直抬脚。 */
        foot[SIMPLE_RF].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_LIFT_S;
        break;
    case 1U: /* 右前保持最高点跨过150mm缝。 */
        foot[SIMPLE_RF].x = step;
        foot[SIMPLE_RF].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_SWING_S;
        break;
    case 2U: /* 右前竖直落到下一块踏板。 */
        foot[SIMPLE_RF].x = step;
        break;
    case 3U: /* 左前原地竖直抬脚，右前保持支撑。 */
        foot[SIMPLE_RF].x = step;
        foot[SIMPLE_LF].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_LIFT_S;
        break;
    case 4U: /* 左前保持最高点跨缝。 */
        foot[SIMPLE_RF].x = step;
        foot[SIMPLE_LF].x = step;
        foot[SIMPLE_LF].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_SWING_S;
        break;
    case 5U: /* 左前竖直落脚。 */
        foot[SIMPLE_RF].x = step;
        foot[SIMPLE_LF].x = step;
        break;
    case 6U: /* 四脚着地，把机身向前平移半步。 */
        set_bridge_b_gap_shift_pose(foot);
        duration = SIMPLE_BRIDGE_B_GAP_SHIFT_S;
        break;
    case 7U: /* 右后在旧踏板上原地竖直抬脚。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_LIFT_S;
        break;
    case 8U: /* 右后保持最高点跨缝。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].x = half_step;
        foot[SIMPLE_RH].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_SWING_S;
        break;
    case 9U: /* 右后竖直落脚。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].x = half_step;
        break;
    case 10U: /* 左后原地竖直抬脚，右后保持支撑。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].x = half_step;
        foot[SIMPLE_LH].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_LIFT_S;
        break;
    case 11U: /* 左后保持最高点跨缝。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].x = half_step;
        foot[SIMPLE_LH].x = half_step;
        foot[SIMPLE_LH].z_down = SIMPLE_BRIDGE_B_GAP_LIFT_Z_MM;
        duration = SIMPLE_BRIDGE_B_GAP_SWING_S;
        break;
    case 12U: /* 左后竖直落脚。 */
        set_bridge_b_gap_shift_pose(foot);
        foot[SIMPLE_RH].x = half_step;
        foot[SIMPLE_LH].x = half_step;
        break;
    case 13U: /* 四脚着地，再平移半步并回到脚在电机下方。 */
        duration = SIMPLE_BRIDGE_B_GAP_RECOVER_S;
        break;
    default:
        simple_robot_jump_abort_watch = 4U;
        enter_state(SIMPLE_STATE_GET_DOWN);
        return;
    }

    run_foot_pose(foot, duration,
                  SIMPLE_BRIDGE_B_GAP_KP, SIMPLE_BRIDGE_B_GAP_KD);
    if (!pose_done) return;
    if (jump_stage < 13U) {
        advance_jump_stage();
    } else {
        simple_robot_jump_count_watch++;
        enter_state(SIMPLE_STATE_GET_DOWN);
    }
}

static void run_jump_sequence(void)
{
    if (jump_mode == SIMPLE_JUMP_T_STEP_DOWN) {
        run_step_down_sequence();
        return;
    }
    if (jump_mode == SIMPLE_JUMP_T_STEP_UP) {
        run_step_up_sequence();
        return;
    }

    /* 默认是已通过实机验证的原地跳；其它动作只替换足端轨迹参数。 */
    fp32 compress_x = SIMPLE_STAND_X_MM;
    fp32 thrust_x = SIMPLE_STAND_X_MM;
    fp32 tuck_x = SIMPLE_STAND_X_MM;
    fp32 landing_x = SIMPLE_STAND_X_MM;
    fp32 absorb_x = SIMPLE_STAND_X_MM;
    fp32 compress_z = SIMPLE_JUMP_COMPRESS_Z_MM;
    fp32 extend_z = SIMPLE_JUMP_EXTEND_Z_MM;
    fp32 tuck_z = SIMPLE_JUMP_TUCK_Z_MM;
    fp32 landing_z = SIMPLE_JUMP_LANDING_Z_MM;
    fp32 absorb_z = SIMPLE_JUMP_ABSORB_Z_MM;
    fp32 thrust_s = SIMPLE_JUMP_THRUST_S;

    switch (jump_mode) {
    case SIMPLE_JUMP_FORWARD:
        compress_x = SIMPLE_FORWARD_JUMP_COMPRESS_X_MM;
        thrust_x = SIMPLE_FORWARD_JUMP_THRUST_X_MM;
        tuck_x = SIMPLE_FORWARD_JUMP_TUCK_X_MM;
        landing_x = SIMPLE_FORWARD_JUMP_LANDING_X_MM;
        absorb_x = SIMPLE_FORWARD_JUMP_ABSORB_X_MM;
        break;
    case SIMPLE_JUMP_BRIDGE_B:
        compress_x = SIMPLE_BRIDGE_B_JUMP_COMPRESS_X_MM;
        thrust_x = SIMPLE_BRIDGE_B_JUMP_THRUST_X_MM;
        tuck_x = SIMPLE_BRIDGE_B_JUMP_TUCK_X_MM;
        landing_x = SIMPLE_BRIDGE_B_JUMP_LANDING_X_MM;
        absorb_x = SIMPLE_BRIDGE_B_JUMP_ABSORB_X_MM;
        compress_z = SIMPLE_BRIDGE_B_JUMP_COMPRESS_Z_MM;
        extend_z = SIMPLE_BRIDGE_B_JUMP_EXTEND_Z_MM;
        tuck_z = SIMPLE_BRIDGE_B_JUMP_TUCK_Z_MM;
        landing_z = SIMPLE_BRIDGE_B_JUMP_LANDING_Z_MM;
        absorb_z = SIMPLE_BRIDGE_B_JUMP_ABSORB_Z_MM;
        thrust_s = SIMPLE_BRIDGE_B_JUMP_THRUST_S;
        break;
    case SIMPLE_JUMP_T_STEP_UP:
        compress_x = SIMPLE_T_UP_COMPRESS_X_MM;
        thrust_x = SIMPLE_T_UP_THRUST_X_MM;
        tuck_x = SIMPLE_T_UP_TUCK_X_MM;
        landing_x = SIMPLE_T_UP_LANDING_X_MM;
        absorb_x = SIMPLE_T_UP_ABSORB_X_MM;
        compress_z = SIMPLE_T_UP_COMPRESS_Z_MM;
        extend_z = SIMPLE_T_UP_EXTEND_Z_MM;
        tuck_z = SIMPLE_T_UP_TUCK_Z_MM;
        landing_z = SIMPLE_T_UP_LANDING_Z_MM;
        absorb_z = SIMPLE_T_UP_ABSORB_Z_MM;
        thrust_s = SIMPLE_T_UP_THRUST_S;
        break;
    case SIMPLE_JUMP_WALL:
        compress_x = SIMPLE_WALL_JUMP_COMPRESS_X_MM;
        thrust_x = SIMPLE_WALL_JUMP_THRUST_X_MM;
        tuck_x = SIMPLE_WALL_JUMP_TUCK_X_MM;
        landing_x = SIMPLE_WALL_JUMP_LANDING_X_MM;
        absorb_x = SIMPLE_WALL_JUMP_ABSORB_X_MM;
        compress_z = SIMPLE_WALL_JUMP_COMPRESS_Z_MM;
        extend_z = SIMPLE_WALL_JUMP_EXTEND_Z_MM;
        tuck_z = SIMPLE_WALL_JUMP_TUCK_Z_MM;
        landing_z = SIMPLE_WALL_JUMP_LANDING_Z_MM;
        absorb_z = SIMPLE_WALL_JUMP_ABSORB_Z_MM;
        thrust_s = SIMPLE_WALL_JUMP_THRUST_S;
        break;
    case SIMPLE_JUMP_VERTICAL:
    default:
        break;
    }

    switch (jump_stage) {
    case 0U: /* 缓慢下蹲蓄力。 */
        run_pose(compress_x, compress_z,
                 SIMPLE_JUMP_COMPRESS_S,
                 SIMPLE_STAND_KP, SIMPLE_STAND_KD,
                 SIMPLE_JUMP_COMPRESS_KP, SIMPLE_JUMP_COMPRESS_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 1U: /* 保持预压，确认八电机都基本到位。 */
        run_pose(compress_x, compress_z,
                 SIMPLE_JUMP_READY_HOLD_S,
                 SIMPLE_JUMP_COMPRESS_KP, SIMPLE_JUMP_COMPRESS_KD,
                 SIMPLE_JUMP_COMPRESS_KP, SIMPLE_JUMP_COMPRESS_KD);
        if (pose_done && motors_near_pose_target(SIMPLE_JUMP_READY_TOL_RAD)) {
            advance_jump_stage();
        } else if (pose_elapsed >= SIMPLE_JUMP_READY_TIMEOUT_S) {
            /* 不同步时回站立，不硬蹬，避免一条腿先把机身掀翻。 */
            simple_robot_jump_abort_watch = 1U;
            enter_state(SIMPLE_STATE_STAND);
        }
        break;

    case 2U: /* 四腿同步快速伸长，产生向上的蹬地冲量。 */
        run_pose(thrust_x, extend_z, thrust_s,
                 SIMPLE_JUMP_COMPRESS_KP, SIMPLE_JUMP_COMPRESS_KD,
                 SIMPLE_JUMP_THRUST_KP, SIMPLE_JUMP_THRUST_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 3U: /* 腾空后把脚收向前方，为前跳落地留出空间。 */
        run_pose(tuck_x, tuck_z,
                 SIMPLE_JUMP_TUCK_S,
                 SIMPLE_JUMP_THRUST_KP, SIMPLE_JUMP_THRUST_KD,
                 SIMPLE_JUMP_FLIGHT_KP, SIMPLE_JUMP_FLIGHT_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 4U: /* 定时伸腿准备接地；后续可再用IMU加落地检测。 */
        run_pose(landing_x, landing_z,
                 SIMPLE_JUMP_LANDING_S,
                 SIMPLE_JUMP_FLIGHT_KP, SIMPLE_JUMP_FLIGHT_KD,
                 SIMPLE_JUMP_LANDING_KP, SIMPLE_JUMP_LANDING_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 5U: /* 触地后允许腿逐渐压缩，并用较大Kd吸收第一下冲击。 */
        run_pose(absorb_x, absorb_z,
                 SIMPLE_JUMP_ABSORB_S,
                 SIMPLE_JUMP_LANDING_KP, SIMPLE_JUMP_LANDING_KD,
                 SIMPLE_JUMP_ABSORB_KP, SIMPLE_JUMP_ABSORB_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 6U: /* 在最低缓冲姿态短暂停留，让机身落稳后再伸腿。 */
        run_pose(absorb_x, absorb_z,
                 SIMPLE_JUMP_ABSORB_HOLD_S,
                 SIMPLE_JUMP_ABSORB_KP, SIMPLE_JUMP_ABSORB_KD,
                 SIMPLE_JUMP_ABSORB_KP, SIMPLE_JUMP_ABSORB_KD);
        if (pose_done) advance_jump_stage();
        break;

    case 7U: /* 从缓冲姿态平滑恢复正常站立。 */
        run_pose(SIMPLE_STAND_X_MM, SIMPLE_STAND_Z_DOWN_MM,
                 SIMPLE_JUMP_RECOVER_S,
                 SIMPLE_JUMP_ABSORB_KP, SIMPLE_JUMP_ABSORB_KD,
                 SIMPLE_STAND_KP, SIMPLE_STAND_KD);
        if (pose_done) {
            simple_robot_jump_count_watch++;
            enter_state(SIMPLE_STATE_STAND);
        }
        break;

    default:
        simple_robot_jump_abort_watch = 2U;
        enter_state(SIMPLE_STATE_STAND);
        break;
    }
}
#endif

static fp32 map_stick(int16_t raw)
{
    /* 手柄原始值经过连续死区映射为 [-1, 1]。 */
    int32_t magnitude = raw < 0 ? -(int32_t)raw : (int32_t)raw;
    if (magnitude <= SIMPLE_RC_DEADBAND) return 0.0f;
    fp32 value = ((fp32)magnitude - (fp32)SIMPLE_RC_DEADBAND) /
                 (SIMPLE_RC_FULL_SCALE - (fp32)SIMPLE_RC_DEADBAND);
    value = clamp(value, 0.0f, 1.0f);
    return raw < 0 ? -value : value;
}

static void update_remote(void)
{
    /*========================= 手柄和状态切换 =========================*/
    remote_control_update();
    Remote_Message_Moniter(&remote_ctrl);

    if (remote_ctrl.rc_lost) {
        /* 失联不会继续旧步态；走/转退回站立，其他过渡保持原状态。 */
        buttons_ready = 0U;
        motion_can_start = 0U;
        forward_command = 0.0f;
        turn_command = 0.0f;
        if (robot_state == SIMPLE_STATE_WALK || robot_state == SIMPLE_STATE_TURN) {
            enter_state(SIMPLE_STATE_STAND);
        } else if (robot_state == SIMPLE_STATE_CRAWL) {
            enter_state(SIMPLE_STATE_GET_DOWN);
        }
        return;
    }

    uint16_t buttons = remote_ctrl.key.v;
    uint16_t rising = 0U;
    if (!buttons_ready) {
        /* 首次上线先同步键值，避免一直按住的键被误判为新按下。 */
        last_buttons = buttons;
        buttons_ready = 1U;
    } else {
        rising = (uint16_t)(buttons & (uint16_t)(~last_buttons));
        last_buttons = buttons;
    }

    /* 跳跃开始后按预定落地流程完成；普通按键不能在半空中切断姿态序列。 */
    if (robot_state == SIMPLE_STATE_JUMP) return;

    if ((rising & PS2_BUTTON_SELECT_MASK) != 0U &&
        robot_state != SIMPLE_STATE_FAULT) {
        if (SIMPLE_ROBOT_CALIBRATED == 0U) {
            latch_fault(SIMPLE_FAULT_NOT_CALIBRATED, 0xFFU, 0xFFU);
        } else {
            enter_state(SIMPLE_STATE_GET_DOWN);
        }
        return;
    }

    if ((rising & PS2_BUTTON_START_MASK) != 0U) {
        if (robot_state == SIMPLE_STATE_WAIT || robot_state == SIMPLE_STATE_GET_DOWN) {
            if (SIMPLE_ROBOT_CALIBRATED == 0U) {
                latch_fault(SIMPLE_FAULT_NOT_CALIBRATED, 0xFFU, 0xFFU);
            } else {
                enter_state(SIMPLE_STATE_GET_UP);
            }
        } else if (robot_state != SIMPLE_STATE_FAULT) {
            enter_state(SIMPLE_STATE_STAND);
        }
        return;
    }

    fp32 requested_forward = map_stick(remote_ctrl.rc.ch[3]);
    fp32 requested_turn = map_stick(remote_ctrl.rc.ch[0]);

#if SIMPLE_JUMP_ENABLE
    /* 演示版R1直接触发一次向前小跳，省去先选木桥模式再按叉键。 */
    if ((rising & PS2_BUTTON_R1_MASK) != 0U &&
        (robot_state == SIMPLE_STATE_STAND ||
         robot_state == SIMPLE_STATE_GET_DOWN) && pose_done &&
        requested_forward == 0.0f && requested_turn == 0.0f) {
        jump_mode = SIMPLE_JUMP_BRIDGE_B;
        enter_state(SIMPLE_STATE_JUMP);
        return;
    }
#endif

    /* 演示版三角键执行一次安全的单腿抬脚律动。 */
    if ((rising & PS2_BUTTON_TRIANGLE_MASK) != 0U &&
        robot_state == SIMPLE_STATE_STAND && pose_done &&
        requested_forward == 0.0f && requested_turn == 0.0f) {
        enter_state(SIMPLE_STATE_DANCE);
        return;
    }

    if (robot_state != SIMPLE_STATE_STAND &&
        robot_state != SIMPLE_STATE_WALK &&
        robot_state != SIMPLE_STATE_TURN &&
        robot_state != SIMPLE_STATE_GET_DOWN &&
        robot_state != SIMPLE_STATE_CRAWL) {
        return;
    }

#if !SIMPLE_REMOTE_LOCOMOTION_ENABLE
    /* 首次落地只验证自适应起立/趴下，忽略全部摇杆运动请求。 */
    return;
#endif

    /* 趴姿完成后，任一摇杆有运动请求就进入低姿高速步态。
     * 左摇杆控制前后，右摇杆控制转弯，两者可以同时使用。 */
    if (robot_state == SIMPLE_STATE_GET_DOWN) {
        if (pose_done &&
            (requested_forward != 0.0f || requested_turn != 0.0f)) {
            enter_state(SIMPLE_STATE_CRAWL);
            forward_command = requested_forward;
            turn_command = requested_turn;
        }
        return;
    }
    if (robot_state == SIMPLE_STATE_CRAWL) {
        if (requested_forward == 0.0f && requested_turn == 0.0f) {
            enter_state(SIMPLE_STATE_GET_DOWN);
        } else {
            forward_command = requested_forward;
            turn_command = requested_turn;
        }
        return;
    }

    if (requested_forward == 0.0f && requested_turn == 0.0f) {
        /* 只有明确看见两摇杆回中，才允许开始下一种运动。 */
        if (robot_state == SIMPLE_STATE_WALK || robot_state == SIMPLE_STATE_TURN) {
            enter_state(SIMPLE_STATE_STAND);
        }
        motion_can_start = 1U;
        motion_direction = 0;
        return;
    }

    if (robot_state == SIMPLE_STATE_STAND && motion_can_start) {
#if SIMPLE_WALK_USE_LOW_POSE && SIMPLE_TURN_USE_LOW_POSE
        if (requested_forward != 0.0f || requested_turn != 0.0f) {
            /* 所有移动统一先平滑蹲下，保持摇杆即可接上低姿高速前进/转弯。 */
            enter_state(SIMPLE_STATE_GET_DOWN);
        }
#else
        if (requested_forward != 0.0f && requested_turn == 0.0f) {
            motion_direction = requested_forward > 0.0f ? 1 : -1;
#if SIMPLE_WALK_USE_LOW_POSE
            /* 所有直线行走统一使用低姿：先平滑蹲下，保持摇杆即可自动接上快速小碎步。 */
            enter_state(SIMPLE_STATE_GET_DOWN);
#else
            enter_state(SIMPLE_STATE_WALK);
#endif
        } else if (requested_forward == 0.0f && requested_turn != 0.0f) {
            motion_direction = requested_turn > 0.0f ? 1 : -1;
#if SIMPLE_TURN_USE_LOW_POSE
            enter_state(SIMPLE_STATE_GET_DOWN);
#else
            enter_state(SIMPLE_STATE_TURN);
#endif
        }
#endif
        motion_can_start = 0U;
    }

    if (robot_state == SIMPLE_STATE_WALK) {
        /* 行走中换向或请求转弯时，先回 STAND，禁止半周期内反扭。 */
        int8_t direction = requested_forward > 0.0f ? 1 : -1;
        if (requested_forward == 0.0f || requested_turn != 0.0f ||
            direction != motion_direction) {
            enter_state(SIMPLE_STATE_STAND);
            motion_can_start = 0U;
        } else {
            forward_command = requested_forward;
        }
    } else if (robot_state == SIMPLE_STATE_TURN) {
        int8_t direction = requested_turn > 0.0f ? 1 : -1;
        if (requested_turn == 0.0f || requested_forward != 0.0f ||
            direction != motion_direction) {
            enter_state(SIMPLE_STATE_STAND);
            motion_can_start = 0U;
        } else {
            turn_command = requested_turn;
        }
    }
}

#if ROBOT_MOTOR_CALIBRATION_ENABLE
/*========================= 独立单电机标定模式 ===========================*/
extern "C" {
volatile uint32_t motor_calibration_elapsed_ms = 0U;
volatile fp32 motor_calibration_base_pos = 0.0f;
volatile fp32 motor_calibration_command_pos = 0.0f;
volatile uint8_t motor_calibration_active_bus = 0U;
volatile uint8_t motor_calibration_active_index = 0U;
volatile fp32 motor_calibration_active_now_pos = 0.0f;
}

static uint8_t calibration_captured = 0U;
static fp32 calibration_start[2][motor_LZ_N];

#if ROBOT_ALL_LEG_SYNC_TEST_ENABLE
extern "C" {
volatile uint8_t all_leg_sync_test_status = 0U;
volatile fp32 all_leg_sync_test_foot_z_mm = SIMPLE_ZERO_CALIBRATION_Z_MM;
volatile fp32 all_leg_sync_test_target[SIMPLE_LEG_COUNT][2] = {{0.0f}};
volatile fp32 all_leg_sync_test_now_pos[SIMPLE_LEG_COUNT][2] = {{0.0f}};
volatile fp32 all_leg_sync_test_error[SIMPLE_LEG_COUNT][2] = {{0.0f}};
volatile uint8_t all_leg_sync_test_fault_leg = 0xFFU;
volatile uint8_t all_leg_sync_test_fault_motor = 0xFFU;
volatile uint8_t all_leg_sync_test_phase = 0U;
volatile uint8_t all_leg_sync_test_homing_leg = 0xFFU;
}

static uint8_t all_leg_sync_prepared = 0U;
static uint32_t all_leg_sync_elapsed_ms = 0U;
static uint32_t all_leg_auto_zero_elapsed_ms = 0U;
static fp32 all_leg_auto_zero_start[SIMPLE_LEG_COUNT][2] = {{0.0f}};
static fp32 all_leg_auto_zero_target[SIMPLE_LEG_COUNT][2] = {{0.0f}};

/* 四条腿同步伸缩，只验证八台电机联合控制，不涉及落地承重。 */
static void all_leg_sync_test_step(void)
{
    if (motor_task_faulted()) {
        /* 保留本测试先前锁存的 2/3/5，不能用后续连锁的底层 fault 覆盖根因。 */
        if (all_leg_sync_test_status == 0U || all_leg_sync_test_status == 1U) {
            all_leg_sync_test_status = 4U;
        }
        motor_task_disarm_all();
        return;
    }
    if (!motor_task_all_ready()) return;

    if (!all_leg_sync_prepared) {
        for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
            fp32 current0 = motor_data(leg, 0U)->recv.Now_Pos;
            fp32 current1 = motor_data(leg, 1U)->recv.Now_Pos;
            fivebar_set_reference(&fivebar[leg], current0, current1);
            if (!fivebar_inverse(&fivebar[leg], leg,
                                 SIMPLE_ZERO_CALIBRATION_X_MM,
                                 SIMPLE_ZERO_CALIBRATION_Z_MM,
                                 all_leg_auto_zero_target[leg])) {
                all_leg_sync_test_status = 2U;
                all_leg_sync_test_fault_leg = leg;
                all_leg_sync_test_fault_motor = 0xFFU;
                motor_task_disarm_all();
                return;
            }
            all_leg_auto_zero_start[leg][0] = current0;
            all_leg_auto_zero_start[leg][1] = current1;
            fp32 start_error0 = all_leg_auto_zero_target[leg][0] - current0;
            fp32 start_error1 = all_leg_auto_zero_target[leg][1] - current1;
            all_leg_sync_test_target[leg][0] = all_leg_auto_zero_target[leg][0];
            all_leg_sync_test_target[leg][1] = all_leg_auto_zero_target[leg][1];
            all_leg_sync_test_now_pos[leg][0] = current0;
            all_leg_sync_test_now_pos[leg][1] = current1;
            all_leg_sync_test_error[leg][0] = start_error0;
            all_leg_sync_test_error[leg][1] = start_error1;
            if ((ROBOT_ALL_LEG_AUTO_ZERO_GEOMETRY_GUARD &&
                 !fivebar_motor_pose_valid(leg, current0, current1)) ||
                (ROBOT_ALL_LEG_AUTO_ZERO_MAX_DELTA_RAD > 0.0f &&
                 (fabsf(start_error0) > ROBOT_ALL_LEG_AUTO_ZERO_MAX_DELTA_RAD ||
                  fabsf(start_error1) > ROBOT_ALL_LEG_AUTO_ZERO_MAX_DELTA_RAD))) {
                all_leg_sync_test_status = 3U;
                all_leg_sync_test_fault_leg = leg;
                all_leg_sync_test_fault_motor =
                    fabsf(start_error0) >= fabsf(start_error1) ? 0U : 1U;
                motor_task_disarm_all();
                return;
            }
        }
        all_leg_sync_prepared = 1U;
        all_leg_sync_elapsed_ms = 0U;
        all_leg_auto_zero_elapsed_ms = 0U;
        all_leg_sync_test_status = 1U;
#if ROBOT_ALL_LEG_AUTO_ZERO_ENABLE
        all_leg_sync_test_phase = 1U;
#else
        all_leg_sync_test_phase = 2U;
#endif
    }

#if ROBOT_ALL_LEG_AUTO_ZERO_ENABLE
    if (all_leg_sync_test_phase == 1U) {
        const uint32_t move_ms = ROBOT_ALL_LEG_AUTO_ZERO_MOVE_MS;
        const uint32_t total_ms = move_ms + ROBOT_ALL_LEG_AUTO_ZERO_SETTLE_MS;

        if (all_leg_auto_zero_elapsed_ms >= total_ms) {
            /* 四条腿归零后，用真实反馈做最终验收。 */
            for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
                for (uint8_t motor = 0U; motor < 2U; ++motor) {
                    fp32 now_pos = motor_data(leg, motor)->recv.Now_Pos;
                    fp32 error = all_leg_auto_zero_target[leg][motor] - now_pos;
                    all_leg_sync_test_now_pos[leg][motor] = now_pos;
                    all_leg_sync_test_error[leg][motor] = error;
                    if (ROBOT_ALL_LEG_AUTO_ZERO_FINAL_ERR_RAD > 0.0f &&
                        fabsf(error) > ROBOT_ALL_LEG_AUTO_ZERO_FINAL_ERR_RAD) {
                        all_leg_sync_test_status = 5U;
                        all_leg_sync_test_fault_leg = leg;
                        all_leg_sync_test_fault_motor = motor;
                        motor_task_disarm_all();
                        return;
                    }
                }
            }
            all_leg_sync_test_phase = 2U;
            all_leg_sync_test_homing_leg = 0xFFU;
            all_leg_sync_elapsed_ms = 0U;
            return;
        }

        fp32 progress = all_leg_auto_zero_elapsed_ms >= move_ms
            ? 1.0f
            : smoothstep5((fp32)all_leg_auto_zero_elapsed_ms / (fp32)move_ms);
        /* 0xFE 表示四条腿正在同时归零。 */
        all_leg_sync_test_homing_leg = 0xFEU;

        for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
            for (uint8_t motor = 0U; motor < 2U; ++motor) {
                motor_lz_control *data = motor_data(leg, motor);
                fp32 target = all_leg_auto_zero_start[leg][motor]
                           + (all_leg_auto_zero_target[leg][motor]
                           - all_leg_auto_zero_start[leg][motor]) * progress;

                fp32 error = target - data->recv.Now_Pos;
                all_leg_sync_test_target[leg][motor] = target;
                all_leg_sync_test_now_pos[leg][motor] = data->recv.Now_Pos;
                all_leg_sync_test_error[leg][motor] = error;
                if (ROBOT_ALL_LEG_AUTO_ZERO_MAX_FOLLOW_ERR > 0.0f &&
                    fabsf(error) > ROBOT_ALL_LEG_AUTO_ZERO_MAX_FOLLOW_ERR) {
                    all_leg_sync_test_status = 5U;
                    all_leg_sync_test_fault_leg = leg;
                    all_leg_sync_test_fault_motor = motor;
                    motor_task_disarm_all();
                    return;
                }

                data->send.Pos = target;
                data->send.W = 0.0f;
                data->send.Kp = ROBOT_ALL_LEG_AUTO_ZERO_KP;
                data->send.Kd = ROBOT_ALL_LEG_AUTO_ZERO_KD;
                data->send.T_ff = 0.0f;
            }
            if (ROBOT_ALL_LEG_AUTO_ZERO_GEOMETRY_GUARD &&
                !fivebar_motor_pose_valid(leg,
                    all_leg_sync_test_target[leg][0],
                    all_leg_sync_test_target[leg][1])) {
                all_leg_sync_test_status = 2U;
                all_leg_sync_test_fault_leg = leg;
                all_leg_sync_test_fault_motor = 0xFFU;
                motor_task_disarm_all();
                return;
            }
        }
        all_leg_sync_test_foot_z_mm = SIMPLE_ZERO_CALIBRATION_Z_MM;
        ++all_leg_auto_zero_elapsed_ms;
        return;
    }
#endif

    uint32_t segment_ms = ROBOT_ALL_LEG_SYNC_TEST_SEGMENT_MS;
    fp32 progress = 0.0f;
    if (all_leg_sync_elapsed_ms >= ROBOT_MOTOR_CALIBRATION_SETTLE_MS) {
        uint32_t t = all_leg_sync_elapsed_ms - ROBOT_MOTOR_CALIBRATION_SETTLE_MS;
#if ROBOT_ALL_LEG_SYNC_TEST_HOLD_AT_TARGET
        fp32 u = t >= segment_ms ? 1.0f : (fp32)t / (fp32)segment_ms;
        progress = smoothstep5(u);
#else
        t %= 2U * segment_ms;
        fp32 u = (fp32)(t % segment_ms) / (fp32)segment_ms;
        if (((t / segment_ms) & 1U) != 0U) u = 1.0f - u;
        progress = smoothstep5(u);
#endif
    }

    fp32 z_down = SIMPLE_ZERO_CALIBRATION_Z_MM
                + (ROBOT_ALL_LEG_SYNC_TEST_Z_MM - SIMPLE_ZERO_CALIBRATION_Z_MM)
                * progress;
    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        fp32 target[2];
        if (!fivebar_inverse(&fivebar[leg], leg,
                             SIMPLE_ZERO_CALIBRATION_X_MM, z_down, target)) {
            all_leg_sync_test_status = 2U;
            all_leg_sync_test_fault_leg = leg;
            all_leg_sync_test_fault_motor = 0xFFU;
            motor_task_disarm_all();
            return;
        }
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            motor_lz_control *data = motor_data(leg, motor);
            fp32 error = target[motor] - data->recv.Now_Pos;
            all_leg_sync_test_target[leg][motor] = target[motor];
            all_leg_sync_test_now_pos[leg][motor] = data->recv.Now_Pos;
            all_leg_sync_test_error[leg][motor] = error;
            if (ROBOT_ALL_LEG_SYNC_TEST_MAX_FOLLOW_ERR > 0.0f &&
                fabsf(error) > ROBOT_ALL_LEG_SYNC_TEST_MAX_FOLLOW_ERR) {
                all_leg_sync_test_status = 5U;
                all_leg_sync_test_fault_leg = leg;
                all_leg_sync_test_fault_motor = motor;
                motor_task_disarm_all();
                return;
            }
            data->send.Pos = target[motor];
            data->send.W = 0.0f;
            data->send.Kp = ROBOT_ALL_LEG_SYNC_TEST_KP;
            data->send.Kd = ROBOT_ALL_LEG_SYNC_TEST_KD;
            data->send.T_ff = 0.0f;
        }
    }
    all_leg_sync_test_foot_z_mm = z_down;
    ++all_leg_sync_elapsed_ms;
}
#endif

#if ROBOT_COAXIAL_PAIR_TEST_ENABLE
extern "C" {
volatile uint8_t coaxial_pair_test_status = 0U;
volatile uint8_t coaxial_pair_test_active_leg = ROBOT_COAXIAL_PAIR_TEST_FIRST_LEG;
volatile fp32 coaxial_pair_test_foot_x_mm = SIMPLE_ZERO_CALIBRATION_X_MM;
volatile fp32 coaxial_pair_test_foot_z_mm = SIMPLE_ZERO_CALIBRATION_Z_MM;
volatile fp32 coaxial_pair_test_target[2] = {0.0f, 0.0f};
volatile fp32 coaxial_pair_test_now_pos[2] = {0.0f, 0.0f};
volatile fp32 coaxial_pair_test_error[2] = {0.0f, 0.0f};
}

static uint8_t pair_test_prepared = 0U;
static uint32_t pair_test_elapsed_ms = 0U;
static fp32 pair_test_zero_target[SIMPLE_LEG_COUNT][2] = {{0.0f}};

/* 两台电机协同验证同轴逆解；通过 FIRST_LEG 选择前腿或后腿。 */
static void coaxial_pair_test_step(void)
{
    if (motor_task_faulted()) {
        coaxial_pair_test_status = 4U;
        motor_task_disarm_all();
        return;
    }
    if (!motor_task_all_ready()) return;

    const uint8_t first_leg = ROBOT_COAXIAL_PAIR_TEST_FIRST_LEG;
    const uint8_t leg_count = ROBOT_COAXIAL_PAIR_TEST_LEG_COUNT;
    if (leg_count == 0U || first_leg >= SIMPLE_LEG_COUNT ||
        first_leg + leg_count > SIMPLE_LEG_COUNT) {
        coaxial_pair_test_status = 2U;
        motor_task_disarm_all();
        return;
    }

    if (!pair_test_prepared) {
        for (uint8_t leg = first_leg; leg < first_leg + leg_count; ++leg) {
            fp32 current0 = motor_data(leg, 0U)->recv.Now_Pos;
            fp32 current1 = motor_data(leg, 1U)->recv.Now_Pos;
            fivebar_set_reference(&fivebar[leg], current0, current1);
            if (!fivebar_inverse(&fivebar[leg], leg,
                                 SIMPLE_ZERO_CALIBRATION_X_MM,
                                 SIMPLE_ZERO_CALIBRATION_Z_MM,
                                 pair_test_zero_target[leg])) {
                coaxial_pair_test_status = 2U;
                motor_task_disarm_all();
                return;
            }
            if (fabsf(pair_test_zero_target[leg][0] - current0) >
                    ROBOT_COAXIAL_PAIR_TEST_MAX_START_ERR ||
                fabsf(pair_test_zero_target[leg][1] - current1) >
                    ROBOT_COAXIAL_PAIR_TEST_MAX_START_ERR) {
                /* 零位姿态或零位数据不一致时禁止直接拉回目标。 */
                coaxial_pair_test_status = 3U;
                motor_task_disarm_all();
                return;
            }
        }
        pair_test_prepared = 1U;
        pair_test_elapsed_ms = 0U;
        coaxial_pair_test_status = 1U;
    }

    /* 未选中的前腿保持零刚度，只保留阻尼。 */
    for (uint8_t leg = first_leg; leg < first_leg + leg_count; ++leg) {
        for (uint8_t motor = 0U; motor < 2U; ++motor) {
            motor_lz_control *data = motor_data(leg, motor);
            data->send.Pos = data->recv.Now_Pos;
            data->send.W = 0.0f;
            data->send.Kp = 0.0f;
            data->send.Kd = SIMPLE_SAFE_HOLD_KD;
            data->send.T_ff = 0.0f;
        }
    }

    uint32_t segment_ms = ROBOT_COAXIAL_PAIR_TEST_SEGMENT_MS;
    uint32_t cycle_ms = 2U * segment_ms;
    uint8_t active_leg = first_leg;
    fp32 offset_x = 0.0f;
    fp32 offset_z = 0.0f;
    if (pair_test_elapsed_ms >= ROBOT_MOTOR_CALIBRATION_SETTLE_MS) {
        uint32_t t = pair_test_elapsed_ms - ROBOT_MOTOR_CALIBRATION_SETTLE_MS;
        active_leg = (uint8_t)(first_leg + (t / cycle_ms) % leg_count);
        t %= cycle_ms;
        fp32 u = (fp32)(t % segment_ms) / (fp32)segment_ms;
        if (((t / segment_ms) & 1U) != 0U) u = 1.0f - u;
        fp32 progress = smoothstep5(u);
        offset_x = progress * ROBOT_COAXIAL_PAIR_TEST_DELTA_X_MM;
        offset_z = progress * ROBOT_COAXIAL_PAIR_TEST_DELTA_Z_MM;
    }

    fp32 target[2];
    fp32 foot_x = SIMPLE_ZERO_CALIBRATION_X_MM + offset_x;
    fp32 z_down = SIMPLE_ZERO_CALIBRATION_Z_MM + offset_z;
    if (!fivebar_inverse(&fivebar[active_leg], active_leg,
                         foot_x, z_down, target)) {
        coaxial_pair_test_status = 2U;
        motor_task_disarm_all();
        return;
    }

    for (uint8_t motor = 0U; motor < 2U; ++motor) {
        motor_lz_control *data = motor_data(active_leg, motor);
        data->send.Pos = target[motor];
        data->send.Kp = ROBOT_COAXIAL_PAIR_TEST_KP;
        data->send.Kd = ROBOT_COAXIAL_PAIR_TEST_KD;
        coaxial_pair_test_target[motor] = target[motor];
        coaxial_pair_test_now_pos[motor] = data->recv.Now_Pos;
        coaxial_pair_test_error[motor] = target[motor] - data->recv.Now_Pos;
    }
    coaxial_pair_test_active_leg = active_leg;
    coaxial_pair_test_foot_x_mm = foot_x;
    coaxial_pair_test_foot_z_mm = z_down;
    ++pair_test_elapsed_ms;
}
#endif

#if ROBOT_MOTOR_CALIBRATION_ALL_ENABLE
/* 将“第几台被选电机”换算成 CAN 总线和 ID 下标。
 * 台架模式只枚举掩码中的电机，因此未接线的 CAN2 不会进入点动序列。 */
static uint8_t calibration_motor_selected(uint8_t bus, uint8_t index)
{
#if ROBOT_BENCH_TEST_ENABLE
    uint8_t mask = bus == 0U ? ROBOT_BENCH_TEST_CAN1_MASK
                             : ROBOT_BENCH_TEST_CAN2_MASK;
    return (mask & (1U << index)) != 0U;
#else
    (void)bus;
    (void)index;
    return 1U;
#endif
}

static uint8_t calibration_selected_count(void)
{
    uint8_t count = 0U;
    for (uint8_t bus = 0U; bus < 2U; ++bus) {
        for (uint8_t index = 0U; index < motor_LZ_N; ++index) {
            if (calibration_motor_selected(bus, index)) ++count;
        }
    }
    return count;
}

static uint8_t calibration_select_slot(uint8_t slot, uint8_t *bus_out,
                                       uint8_t *index_out)
{
    for (uint8_t bus = 0U; bus < 2U; ++bus) {
        for (uint8_t index = 0U; index < motor_LZ_N; ++index) {
            if (!calibration_motor_selected(bus, index)) continue;
            if (slot == 0U) {
                *bus_out = bus;
                *index_out = index;
                return 1U;
            }
            --slot;
        }
    }
    return 0U;
}
#endif

static void calibration_step(void)
{
#if ROBOT_ALL_LEG_SYNC_TEST_ENABLE
    all_leg_sync_test_step();
    return;
#elif ROBOT_COAXIAL_PAIR_TEST_ENABLE
    coaxial_pair_test_step();
    return;
#endif
    /* 未选中的 7 台电机 Kp=0，仅保留阻尼；选中电机围绕上电位置缓慢点动。 */
    if (motor_task_faulted()) {
        motor_task_disarm_all();
        return;
    }
#if !ROBOT_MOTOR_CALIBRATION_ALL_ENABLE
    if (ROBOT_MOTOR_CALIBRATION_BUS > 1U ||
        ROBOT_MOTOR_CALIBRATION_INDEX >= motor_LZ_N) {
        motor_task_disarm_all();
        return;
    }
    if (!motor_task_motor_ready(ROBOT_MOTOR_CALIBRATION_BUS,
                                ROBOT_MOTOR_CALIBRATION_INDEX)) return;
#else
    if (!motor_task_all_ready()) return;
#endif

    if (!calibration_captured) {
        for (uint8_t bus = 0U; bus < 2U; ++bus) {
            for (uint8_t index = 0U; index < motor_LZ_N; ++index) {
                motor_lz_control *data = bus == 0U
                    ? &motor_lz_data_can1[index] : &motor_lz_data_can2[index];
                calibration_start[bus][index] = data->recv.Now_Pos;
            }
        }
        calibration_captured = 1U;
    }

    for (uint8_t bus = 0U; bus < 2U; ++bus) {
        for (uint8_t index = 0U; index < motor_LZ_N; ++index) {
            motor_lz_control *data = bus == 0U
                ? &motor_lz_data_can1[index] : &motor_lz_data_can2[index];
            data->send.Pos = calibration_start[bus][index];
            data->send.W = 0.0f;
            data->send.Kp = 0.0f;
            data->send.Kd = SIMPLE_SAFE_HOLD_KD;
            data->send.T_ff = 0.0f;
        }
    }

    uint32_t segment_ms = ROBOT_MOTOR_CALIBRATION_SEGMENT_MS;
    fp32 offset = 0.0f;
#if ROBOT_MOTOR_CALIBRATION_ALL_ENABLE
    uint8_t selected_bus = 0U;
    uint8_t selected_index = 0U;
    uint8_t selected_count = calibration_selected_count();
    if (selected_count == 0U ||
        !calibration_select_slot(0U, &selected_bus, &selected_index)) {
        motor_task_disarm_all();
        return;
    }
#else
    uint8_t selected_bus = ROBOT_MOTOR_CALIBRATION_BUS;
    uint8_t selected_index = ROBOT_MOTOR_CALIBRATION_INDEX;
#endif

    if (motor_calibration_elapsed_ms >= ROBOT_MOTOR_CALIBRATION_SETTLE_MS) {
        uint32_t t = motor_calibration_elapsed_ms - ROBOT_MOTOR_CALIBRATION_SETTLE_MS;
#if ROBOT_MOTOR_CALIBRATION_ALL_ENABLE
        /* 每台电机用两个 segment：缓慢正向，再缓慢回原位。 */
        uint32_t motor_cycle_ms = 2U * segment_ms;
        uint8_t slot = (uint8_t)((t / motor_cycle_ms) % selected_count);
        (void)calibration_select_slot(slot, &selected_bus, &selected_index);
        t %= motor_cycle_ms;
#endif
        fp32 u = (fp32)(t % segment_ms) / (fp32)segment_ms;
        if (((t / segment_ms) & 1U) != 0U) u = 1.0f - u;
        offset = smoothstep5(u) * ROBOT_MOTOR_CALIBRATION_DELTA_RAD;
    }

    motor_lz_control *selected = selected_bus == 0U
        ? &motor_lz_data_can1[selected_index]
        : &motor_lz_data_can2[selected_index];
    motor_calibration_active_bus = selected_bus;
    motor_calibration_active_index = selected_index;
    motor_calibration_base_pos = calibration_start[selected_bus][selected_index];
    motor_calibration_active_now_pos = selected->recv.Now_Pos;
    selected->send.Pos = motor_calibration_base_pos + offset;
    selected->send.Kp = ROBOT_MOTOR_CALIBRATION_KP;
    selected->send.Kd = ROBOT_MOTOR_CALIBRATION_KD;
    motor_calibration_command_pos = selected->send.Pos;
    motor_calibration_elapsed_ms++;
}
#endif

void robot_application_init(void)
{
    remote_control_init();
    obstacle_mode = SIMPLE_OBSTACLE_NORMAL;
    side_z_comp_current = 0.0f;
    roll_zero_valid = 0U;
    roll_zero = 0.0f;
    simple_robot_obstacle_mode_watch = SIMPLE_OBSTACLE_NORMAL;
    simple_robot_side_z_comp_mm_watch = 0.0f;
    simple_robot_imu_ready_watch = 0U;
    simple_robot_roll_raw_watch = 0.0f;
    simple_robot_roll_zero_watch = 0.0f;
    simple_robot_roll_relative_watch = 0.0f;
    simple_robot_imu_comp_add_mm_watch = 0.0f;
    simple_robot_jump_mode_watch = 0xFFU;
    simple_robot_jump_stage_watch = 0xFFU;
    simple_robot_jump_abort_watch = 0U;
    simple_robot_jump_count_watch = 0U;
    simple_robot_dance_stage_watch = 0xFFU;
    simple_robot_dance_count_watch = 0U;
    simple_gait_reset(&gait);
    enter_state(SIMPLE_STATE_WAIT);
}

void robot_application_step_1ms(void)
{
    /*=========================== 1 ms 主流程 =============================*/
#if ROBOT_MOTOR_CALIBRATION_ENABLE
    calibration_step();
    return;
#endif

#if SIMPLE_IMU_BALANCE_ENABLE
    update_imu_watch();
#endif
    update_remote();
    /* 底层任一电机不健康，上层立即锁存故障并统一失能。 */
    if (motor_task_faulted()) {
        latch_fault(SIMPLE_FAULT_MOTOR, 0xFFU, 0xFFU);
    }

    if (robot_state == SIMPLE_STATE_FAULT) {
        motor_task_disarm_all();
        return;
    }
    if (!motor_task_all_ready()) return;

    switch (robot_state) {
    case SIMPLE_STATE_WAIT:
        safe_hold();
        break;

    case SIMPLE_STATE_GET_UP:
        /* 两段起立：先到统一趴姿，再从趴姿升到站姿。 */
        if (getup_stage == 0U) {
            run_pose(SIMPLE_DOWN_X_MM, SIMPLE_DOWN_Z_DOWN_MM,
                     SIMPLE_GETUP_STAGE1_S,
                     SIMPLE_GETUP_KP, SIMPLE_GETUP_KD,
                     SIMPLE_GETUP_KP, SIMPLE_GETUP_KD);
            if (pose_done) {
                getup_stage = 1U;
                pose_prepared = 0U;
                pose_done = 0U;
            }
        } else {
            run_pose(SIMPLE_STAND_X_MM, SIMPLE_STAND_Z_DOWN_MM,
                     SIMPLE_GETUP_STAGE2_S,
                     SIMPLE_GETUP_KP, SIMPLE_GETUP_KD,
                     SIMPLE_STAND_KP, SIMPLE_STAND_KD);
            if (pose_done) enter_state(SIMPLE_STATE_STAND);
        }
        break;

    case SIMPLE_STATE_STAND:
        run_pose(SIMPLE_STAND_X_MM, SIMPLE_STAND_Z_DOWN_MM,
                 SIMPLE_STAND_TRANSITION_S,
                 SIMPLE_STAND_KP, SIMPLE_STAND_KD,
                 SIMPLE_STAND_KP, SIMPLE_STAND_KD);
        break;

    case SIMPLE_STATE_WALK:
    case SIMPLE_STATE_TURN: {
        /* 步态只给脚位置；这里统一做 4 次逆解并提交 8 个电机角。 */
        SimpleFootTarget foot[SIMPLE_LEG_COUNT];
        fp32 target[SIMPLE_LEG_COUNT][2];
        simple_gait_step(&gait, SIMPLE_CONTROL_DT_S,
                         forward_command, turn_command,
                         SIMPLE_GAIT_TROT,
                         SIMPLE_STAND_Z_DOWN_MM,
                         SIMPLE_WALK_STEP_MAX_MM,
                         SIMPLE_TURN_STEP_MAX_MM,
                         SIMPLE_STEP_HEIGHT_MM, foot);
        if (solve_feet(foot, target)) {
            send_targets(target, SIMPLE_WALK_KP, SIMPLE_WALK_KD);
        }
        break;
    }

    case SIMPLE_STATE_CRAWL: {
        SimpleFootTarget foot[SIMPLE_LEG_COUNT];
        fp32 target[SIMPLE_LEG_COUNT][2];
#if SIMPLE_FAST_CRAWL_ENABLE
        SimpleGaitPattern crawl_pattern = SIMPLE_GAIT_FAST_TRANSIT;
#else
        SimpleGaitPattern crawl_pattern = SIMPLE_GAIT_CRAWL;
#endif
        fp32 base_z_down = SIMPLE_TRANSIT_Z_DOWN_MM;
        fp32 walk_step = SIMPLE_TRANSIT_STEP_MAX_MM;
        fp32 turn_step = SIMPLE_TRANSIT_TURN_STEP_MAX_MM;
        fp32 step_height = SIMPLE_TRANSIT_STEP_HEIGHT_MM;
        fp32 gait_kp = SIMPLE_TRANSIT_KP;
        fp32 gait_kd = SIMPLE_TRANSIT_KD;
        fp32 side_comp_target = 0.0f;
        fp32 gait_turn_command = turn_command;

#if SIMPLE_OBSTACLE_MODE_ENABLE
        if (obstacle_mode == SIMPLE_OBSTACLE_PIT) {
            crawl_pattern = SIMPLE_GAIT_FAST_CRAWL;
            base_z_down = SIMPLE_PIT_Z_DOWN_MM;
            walk_step = SIMPLE_PIT_STEP_MAX_MM;
            turn_step = SIMPLE_PIT_TURN_STEP_MAX_MM;
            step_height = SIMPLE_PIT_STEP_HEIGHT_MM;
            gait_kp = SIMPLE_PIT_KP;
            gait_kd = SIMPLE_PIT_KD;
        } else if (obstacle_mode == SIMPLE_OBSTACLE_LIMIT_BAR) {
            crawl_pattern = SIMPLE_GAIT_FAST_CRAWL;
            base_z_down = SIMPLE_LIMIT_BAR_Z_DOWN_MM;
            walk_step = SIMPLE_LIMIT_BAR_STEP_MAX_MM;
            turn_step = SIMPLE_LIMIT_BAR_TURN_STEP_MAX_MM;
            step_height = SIMPLE_LIMIT_BAR_STEP_HEIGHT_MM;
            gait_kp = SIMPLE_LIMIT_BAR_KP;
            gait_kd = SIMPLE_LIMIT_BAR_KD;
        } else if (obstacle_mode == SIMPLE_OBSTACLE_CROSS_SLOPE_10) {
            /* 固定左右差高 + 快速对角位控，右摇杆直接叠加转向步幅。 */
            crawl_pattern = SIMPLE_GAIT_FAST_CRAWL;
            base_z_down = SIMPLE_CROSS_SLOPE_Z_DOWN_MM;
            walk_step = SIMPLE_CROSS_SLOPE_STEP_MAX_MM;
            turn_step = SIMPLE_CROSS_SLOPE_TURN_STEP_MAX_MM;
            step_height = SIMPLE_CROSS_SLOPE_STEP_HEIGHT_MM;
            gait_kp = SIMPLE_CROSS_SLOPE_KP;
            gait_kd = SIMPLE_CROSS_SLOPE_KD;
            side_comp_target = compute_side_comp_target();
        } else if (obstacle_mode == SIMPLE_OBSTACLE_BRIDGE_A) {
            crawl_pattern = SIMPLE_GAIT_BRIDGE_A;
            base_z_down = SIMPLE_BRIDGE_A_Z_DOWN_MM;
            walk_step = SIMPLE_BRIDGE_A_STEP_MAX_MM;
            turn_step = SIMPLE_BRIDGE_A_TURN_STEP_MAX_MM;
            step_height = SIMPLE_BRIDGE_A_STEP_HEIGHT_MM;
            gait_kp = SIMPLE_BRIDGE_A_KP;
            gait_kd = SIMPLE_BRIDGE_A_KD;
            gait_turn_command = clamp(
                turn_command + forward_command * SIMPLE_BRIDGE_A_YAW_TRIM,
                -1.0f, 1.0f);
        } else if (obstacle_mode == SIMPLE_OBSTACLE_BRIDGE_B) {
            /* 木桥B改为高抬脚四拍：左摇杆连续走，右摇杆微调方向。 */
            crawl_pattern = SIMPLE_GAIT_BRIDGE_B;
            base_z_down = SIMPLE_BRIDGE_B_Z_DOWN_MM;
            walk_step = SIMPLE_BRIDGE_B_STEP_MAX_MM;
            turn_step = SIMPLE_BRIDGE_B_TURN_STEP_MAX_MM;
            step_height = SIMPLE_BRIDGE_B_STEP_HEIGHT_MM;
            gait_kp = SIMPLE_BRIDGE_B_KP;
            gait_kd = SIMPLE_BRIDGE_B_KD;
            gait_turn_command = clamp(
                turn_command + forward_command * SIMPLE_BRIDGE_B_YAW_TRIM,
                -1.0f, 1.0f);
        }
#endif

        if (obstacle_mode == SIMPLE_OBSTACLE_NORMAL) {
            /* L1前进保留实机左修正，后退默认不叠加修正，避免只拉后杆却原地打转。
             * 右摇杆转向不随前后方向反号：前进和后退都能直接叠加左转/右转。 */
            fp32 manual_turn = turn_command;
            if (forward_command != 0.0f && turn_command != 0.0f) {
                manual_turn *= SIMPLE_TRANSIT_COMBINED_TURN_SCALE;
            }
            fp32 yaw_trim = fabsf(forward_command) *
                (forward_command >= 0.0f
                 ? SIMPLE_TRANSIT_FORWARD_YAW_TRIM
                 : SIMPLE_TRANSIT_REVERSE_YAW_TRIM);
            gait_turn_command = clamp(
                manual_turn + yaw_trim,
                -1.0f, 1.0f);
        }

        side_z_comp_current = move_toward(
            side_z_comp_current, side_comp_target,
            SIMPLE_CROSS_SLOPE_COMP_RATE_MM_S * SIMPLE_CONTROL_DT_S);
        simple_robot_side_z_comp_mm_watch = side_z_comp_current;
        simple_gait_step(&gait, SIMPLE_CONTROL_DT_S,
                         forward_command, gait_turn_command,
                         crawl_pattern,
                         base_z_down,
                         walk_step,
                         turn_step,
                         step_height, foot);

        /* 横坡方向：右侧两腿在高处并缩短，左侧两腿在低处并伸长。 */
        for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
            if (leg == SIMPLE_RF || leg == SIMPLE_RH) {
                foot[leg].z_down -= side_z_comp_current;
            } else {
                foot[leg].z_down += side_z_comp_current;
            }
        }
        if (solve_feet(foot, target)) {
            send_targets(target, gait_kp, gait_kd);
        }
        break;
    }

    case SIMPLE_STATE_GET_DOWN:
        run_pose(SIMPLE_DOWN_X_MM, SIMPLE_DOWN_Z_DOWN_MM,
                 SIMPLE_GETDOWN_S,
                 SIMPLE_STAND_KP, SIMPLE_STAND_KD,
                 SIMPLE_DOWN_KP, SIMPLE_DOWN_KD);
        break;

    case SIMPLE_STATE_JUMP:
#if SIMPLE_JUMP_ENABLE
        run_jump_sequence();
#else
        enter_state(SIMPLE_STATE_STAND);
#endif
        break;

    case SIMPLE_STATE_DANCE:
        run_dance_sequence();
        break;

    case SIMPLE_STATE_FAULT:
    default:
#if ROBOT_GLOBAL_FAILSAFE_ENABLE
        motor_task_disarm_all();
#else
        /* 非锁存模式不广播失能；异常状态退回无位置刚度的等待姿态。 */
        robot_state = SIMPLE_STATE_WAIT;
        simple_robot_state_watch = SIMPLE_STATE_WAIT;
        safe_hold();
#endif
        break;
    }

#if ROBOT_GLOBAL_FAILSAFE_ENABLE
    if (simple_robot_fault_watch != SIMPLE_FAULT_NONE) {
        motor_task_disarm_all();
    }
#endif
}
