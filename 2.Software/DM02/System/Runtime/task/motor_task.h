#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "motor_LZ.h"

#define motor_LZ_N 4

/* 生产/底层总线台架选择：
 * 0 = 生产模式，初始化并控制全部 8 个电机。
 * 1 = 仅初始化并控制掩码选中的电机，只供底层通信排查。
 * 台架模式的 motor_task_all_ready() 只检查掩码选中的电机；生产模式
 * 仍然要求全部 8 台正常。单电机标定时也可以只打开对应的一位。
 *
 * 软件映射：CAN1 ID1/2=右前腿右/左电机，ID3/4=左前腿左/右电机；
 * CAN2 ID1/2=右后腿右/左电机，ID3/4=左后腿左/右电机。
 * 单独排查 CAN2 ID3/4 时，可将 CAN1 掩码设为 0x00、CAN2 掩码设为 0x0C。
 */
#define ROBOT_BENCH_TEST_ENABLE 1
/* 八电机联合在线测试：CAN1/CAN2 的 ID1~4 全部初始化。 */
#define ROBOT_BENCH_TEST_CAN1_MASK 0x0F
#define ROBOT_BENCH_TEST_CAN2_MASK 0x0F

/*
 * 独立 CAN 收发诊断：1=跳过电机握手/控制，每 500 ms 向 CAN1/CAN2 的 ID1
 * 各发送一次进入模式探测帧，只观察 BSP 原始收发计数。诊断结束必须恢复为 0。
 */
#define ROBOT_CAN_DIAGNOSTIC_ENABLE 0
/* 0=纯接收测试；1=主循环周期发送 0x123/0x124/0x125。 */
#define ROBOT_CAN_DIAGNOSTIC_TX_ENABLE 1

/* 运行保护阈值：任一已选电机反馈超时、过温、报错或出现非有限数值，
 * 都会锁存整机故障。T_ff 还会在最终下发前独立限幅。 */
/* 八电机联合运行时允许短暂 CAN 抖动，连续 300 ms 无反馈才判定掉线。 */
#define ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS 300U
#define ROBOT_BENCH_READY_SETTLE_MS     300U
#define ROBOT_MOTOR_MAX_TEMPERATURE_C 80.0f
#define ROBOT_MOTOR_TFF_LIMIT_NM 2.0f

/* 竞赛调试模式：0=运行中任何单电机异常都不再联动失能八个电机。
 * 上电初次握手仍要求八电机全部上线；电机驱动器/BMS自身保护不受此开关控制。 */
#define ROBOT_GLOBAL_FAILSAFE_ENABLE 0

/* 单电机/零位读取工具；正常遥控固件必须保持为 0。 */
#define ROBOT_MOTOR_CALIBRATION_ENABLE       0
/* 1=忽略固定 BUS/INDEX，只按两个台架掩码中已选的电机逐台巡检。 */
#define ROBOT_MOTOR_CALIBRATION_ALL_ENABLE   1
#define ROBOT_MOTOR_CALIBRATION_BUS          1U
#define ROBOT_MOTOR_CALIBRATION_INDEX        0U
#define ROBOT_MOTOR_CALIBRATION_DELTA_RAD    0.0f
#define ROBOT_MOTOR_CALIBRATION_SETTLE_MS    2000U
#define ROBOT_MOTOR_CALIBRATION_SEGMENT_MS   4000U
#define ROBOT_MOTOR_CALIBRATION_KP           0.0f
#define ROBOT_MOTOR_CALIBRATION_KD           0.6f

/* 同轴五连杆双电机协同验证。当前验证后腿 +X：
 * 右后、左后依次向狗头方向移动 15 mm 再返回。 */
#define ROBOT_COAXIAL_PAIR_TEST_ENABLE          0
#define ROBOT_COAXIAL_PAIR_TEST_FIRST_LEG       2U
#define ROBOT_COAXIAL_PAIR_TEST_LEG_COUNT       2U
#define ROBOT_COAXIAL_PAIR_TEST_DELTA_X_MM      15.0f
#define ROBOT_COAXIAL_PAIR_TEST_DELTA_Z_MM      0.0f
#define ROBOT_COAXIAL_PAIR_TEST_SEGMENT_MS      4000U
#define ROBOT_COAXIAL_PAIR_TEST_KP              12.0f
#define ROBOT_COAXIAL_PAIR_TEST_KD              1.0f
#define ROBOT_COAXIAL_PAIR_TEST_MAX_START_ERR   0.12f

/* 架空大行程站立预演：四条腿从水平主动杆零位同步伸到 z=230 mm 再返回。
 * 必须整机架空；通过后才允许进入实际落地起立测试。 */
#define ROBOT_ALL_LEG_SYNC_TEST_ENABLE           0
#define ROBOT_ALL_LEG_SYNC_TEST_Z_MM             230.0f
#define ROBOT_ALL_LEG_SYNC_TEST_SEGMENT_MS       3000U
#define ROBOT_ALL_LEG_SYNC_TEST_HOLD_AT_TARGET   1
#define ROBOT_ALL_LEG_SYNC_TEST_KP               18.0f
#define ROBOT_ALL_LEG_SYNC_TEST_KD               1.5f
#define ROBOT_ALL_LEG_SYNC_TEST_MAX_FOLLOW_ERR   0.0f
/* 自动归零：四条腿同时平滑移动，起点必须仍在正确装配支链附近。 */
#define ROBOT_ALL_LEG_AUTO_ZERO_ENABLE            0
#define ROBOT_ALL_LEG_AUTO_ZERO_MOVE_MS           6000U
#define ROBOT_ALL_LEG_AUTO_ZERO_SETTLE_MS         1500U
#define ROBOT_ALL_LEG_AUTO_ZERO_KP                 12.0f
#define ROBOT_ALL_LEG_AUTO_ZERO_KD                 1.0f
/* 下面三个阈值为 0 表示关闭对应的软件误差停机，避免有线烧录反复调试。 */
#define ROBOT_ALL_LEG_AUTO_ZERO_MAX_DELTA_RAD     0.0f
#define ROBOT_ALL_LEG_AUTO_ZERO_FINAL_ERR_RAD     0.0f
#define ROBOT_ALL_LEG_AUTO_ZERO_MAX_FOLLOW_ERR    0.0f
/* 0=自动归零插值时不做闭链姿态拦截；逆解和底层硬件保护仍保留。 */
#define ROBOT_ALL_LEG_AUTO_ZERO_GEOMETRY_GUARD    0


/* 两条 CAN 总线各挂 4 个灵足电机；数组下标 0~3 对应电机 ID 1~4。 */
extern Class_Motor_LZ motor_lz_can1[motor_LZ_N];
extern motor_lz_control motor_lz_data_can1[motor_LZ_N];

extern Class_Motor_LZ motor_lz_can2[motor_LZ_N];
extern motor_lz_control motor_lz_data_can2[motor_LZ_N];

/* init_ok 表示本次启动握手成功；last_rx_tick 记录最近一次已解码反馈时刻。
 * 是否允许整机控制仍须通过 motor_task_all_ready() 综合判断。 */
extern volatile uint8_t motor_can1_init_ok[motor_LZ_N];
extern volatile uint8_t motor_can2_init_ok[motor_LZ_N];
extern volatile uint32_t motor_can1_last_rx_tick[motor_LZ_N];
extern volatile uint32_t motor_can2_last_rx_tick[motor_LZ_N];

/* 八电机使能测试统一观察量：[bus][index]，bus 0/1=CAN1/CAN2，index 0..3=ID1..4。 */
extern volatile uint8_t motor_enable_test_all_ready_watch;
extern volatile uint8_t motor_enable_test_faulted_watch;
extern volatile uint8_t motor_enable_test_runtime_armed_watch;
extern volatile uint8_t motor_bench_init_complete_watch;
extern volatile uint8_t motor_bench_feedback_ready_watch;
extern volatile uint32_t motor_bench_ready_elapsed_watch;
extern volatile uint8_t motor_bench_not_ready_bus_watch;
extern volatile uint8_t motor_bench_not_ready_index_watch;
extern volatile uint8_t motor_bench_not_ready_reason_watch;
extern volatile uint8_t motor_enable_test_init_ok[2][motor_LZ_N];
extern volatile uint8_t motor_enable_test_status[2][motor_LZ_N];
extern volatile uint8_t motor_enable_test_mode[2][motor_LZ_N];
extern volatile uint8_t motor_enable_test_merror[2][motor_LZ_N];
extern volatile fp32 motor_enable_test_now_pos[2][motor_LZ_N];
extern volatile fp32 motor_enable_test_send_pos[2][motor_LZ_N];
extern volatile uint32_t motor_enable_test_rx_age_ms[2][motor_LZ_N];
extern volatile uint32_t motor_can_diag_ping_count[3];

void motor_LZ_Data_send(Class_Motor_LZ *__motor_lz,motor_lz_control *__data,const Enum_Motor_LZ_Mode &__Mode);
void motor_LZ_Data_recv(Class_Motor_LZ *__motor_lz,Struct_recv_motor_Lz *data);

void motor_task_init();
void motor_task();
/* CAN 诊断主循环桩：CAN1/2/3 发送标准帧 0x123/0x124/0x125。 */
void motor_can_diagnostic_poll();
/* 仅当全部 8 个电机均初始化、软件已武装且反馈未超过 100 ms 时返回 1。 */
uint8_t motor_task_all_ready();
/* 单电机台架模式只检查指定 BUS/INDEX，不要求其余 7 台在线。 */
uint8_t motor_task_motor_ready(uint8_t bus, uint8_t index);
/* 故障一旦锁存，重新执行 motor_task_init() 前始终返回 1。 */
uint8_t motor_task_faulted();
/* 锁存故障并撤销全部电机的软件武装；重复调用不会重复调用 lose()。
 * lose() 的硬件停机功能码仍有协议疑点，见 motor_LZ.cpp 的安全注释。 */
void motor_task_disarm_all();


#endif
