#include "motor_task.h"
#include "motor_LZ.h"
#include "math_support.h"
#include "robot_config.h"
#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* 软件映射：CAN1 管前腿，CAN2 管后腿；每条总线数组下标 0~3
 * 对应电机 ID 1~4。具体左右电机顺序见 motor_task.h。 */
Class_Motor_LZ motor_lz_can1[motor_LZ_N];
motor_lz_control motor_lz_data_can1[motor_LZ_N];

Class_Motor_LZ motor_lz_can2[motor_LZ_N];
motor_lz_control motor_lz_data_can2[motor_LZ_N];

/* init_ok 只记录启动握手结果；last_rx_tick 记录反馈新鲜度。
 * 软件 ENABLE 状态仍由初始化成功和统一失能流程独立管理。 */
volatile uint8_t motor_can1_init_ok[motor_LZ_N] = {0U};
volatile uint8_t motor_can2_init_ok[motor_LZ_N] = {0U};
volatile uint32_t motor_can1_last_rx_tick[motor_LZ_N] = {0U};
volatile uint32_t motor_can2_last_rx_tick[motor_LZ_N] = {0U};

/* fault_latched 重新初始化前不清除；disarm_complete 保证每个电机只调用一次 lose()。 */
static volatile uint8_t motor_object_configured[2][motor_LZ_N] = {{0U}};
static volatile uint8_t motor_fault_latched = 0U;
static volatile uint8_t motor_disarm_complete = 0U;
#if ROBOT_BENCH_TEST_ENABLE
/* 台架上电时电机可能还没准备好，因此未握手成功的电机每 200 ms 重试一次。
 * 在全部选中电机就绪前，motor_task() 不会发送任何位置控制帧。 */
static uint32_t motor_bench_next_retry_tick = 0U;
static uint32_t motor_bench_ready_since_tick = 0U;
static volatile uint8_t motor_bench_runtime_armed = 0U;
#endif

/*
 * 八电机使能测试观察矩阵：第一维 0=CAN1、1=CAN2；第二维 0..3=ID1..4。
 * 这些变量只给 Keil Watch 使用，不参与控制决策。
 */
volatile uint8_t motor_enable_test_all_ready_watch = 0U;
volatile uint8_t motor_enable_test_faulted_watch = 0U;
volatile uint8_t motor_enable_test_runtime_armed_watch = 0U;
volatile uint8_t motor_bench_init_complete_watch = 0U;
volatile uint8_t motor_bench_feedback_ready_watch = 0U;
volatile uint32_t motor_bench_ready_elapsed_watch = 0U;
volatile uint8_t motor_bench_not_ready_bus_watch = 0xFFU;
volatile uint8_t motor_bench_not_ready_index_watch = 0xFFU;
volatile uint8_t motor_bench_not_ready_reason_watch = 0U;
volatile uint8_t motor_enable_test_init_ok[2][motor_LZ_N] = {{0U}};
volatile uint8_t motor_enable_test_status[2][motor_LZ_N] = {{0U}};
volatile uint8_t motor_enable_test_mode[2][motor_LZ_N] = {{0U}};
volatile uint8_t motor_enable_test_merror[2][motor_LZ_N] = {{0U}};
volatile fp32 motor_enable_test_now_pos[2][motor_LZ_N] = {{0.0f}};
volatile fp32 motor_enable_test_send_pos[2][motor_LZ_N] = {{0.0f}};
volatile uint32_t motor_enable_test_rx_age_ms[2][motor_LZ_N] = {{0U}};
volatile uint32_t motor_can_diag_ping_count[3] = {0U, 0U, 0U};

/* 保留给 Keil Watch 的旧 CAN2 诊断量。 */
extern "C" {
volatile fp32 can1_id1_now_pos = 0.0f;
volatile fp32 can1_id2_now_pos = 0.0f;
volatile fp32 can1_id3_now_pos = 0.0f;
volatile fp32 can1_id4_now_pos = 0.0f;
volatile uint8_t can2_id1_init_ok = 0U;
volatile uint8_t can2_id2_init_ok = 0U;
volatile uint8_t can2_id3_init_ok = 0U;
volatile uint8_t can2_id4_init_ok = 0U;
volatile uint8_t can2_id1_status = 0U;
volatile uint8_t can2_id2_status = 0U;
volatile uint8_t can2_id3_status = 0U;
volatile uint8_t can2_id4_status = 0U;
volatile uint8_t can2_id1_mode = 0U;
volatile uint8_t can2_id2_mode = 0U;
volatile uint8_t can2_id3_mode = 0U;
volatile uint8_t can2_id4_mode = 0U;
volatile uint8_t can2_id1_merror = 0U;
volatile uint8_t can2_id2_merror = 0U;
volatile uint8_t can2_id3_merror = 0U;
volatile uint8_t can2_id4_merror = 0U;
volatile fp32 can2_id1_now_pos = 0.0f;
volatile fp32 can2_id2_now_pos = 0.0f;
volatile fp32 can2_id3_now_pos = 0.0f;
volatile fp32 can2_id4_now_pos = 0.0f;
volatile fp32 can2_id1_send_pos = 0.0f;
volatile fp32 can2_id2_send_pos = 0.0f;
volatile fp32 can2_id3_send_pos = 0.0f;
volatile fp32 can2_id4_send_pos = 0.0f;
volatile uint32_t can2_id1_rx_count = 0U;
volatile uint32_t can2_id2_rx_count = 0U;
volatile uint32_t can2_id3_rx_count = 0U;
volatile uint32_t can2_id4_rx_count = 0U;
volatile uint32_t can2_id1_tx_count = 0U;
volatile uint32_t can2_id2_tx_count = 0U;
volatile uint32_t can2_id3_tx_count = 0U;
volatile uint32_t can2_id4_tx_count = 0U;
}

static void motor_update_can2_lh_watch(void)
{
	uint32_t now = HAL_GetTick();
	motor_enable_test_faulted_watch = motor_fault_latched;
	#if ROBOT_BENCH_TEST_ENABLE
	motor_enable_test_runtime_armed_watch = motor_bench_runtime_armed;
	#else
	motor_enable_test_runtime_armed_watch = 1U;
	#endif
	for (uint8_t bus = 0U; bus < 2U; ++bus) {
		for (uint8_t index = 0U; index < motor_LZ_N; ++index) {
			motor_lz_control *data = (bus == 0U)
				? &motor_lz_data_can1[index] : &motor_lz_data_can2[index];
			Class_Motor_LZ *object = (bus == 0U)
				? &motor_lz_can1[index] : &motor_lz_can2[index];
			uint32_t last_rx = (bus == 0U)
				? motor_can1_last_rx_tick[index] : motor_can2_last_rx_tick[index];

			motor_enable_test_init_ok[bus][index] = (bus == 0U)
				? motor_can1_init_ok[index] : motor_can2_init_ok[index];
			motor_enable_test_status[bus][index] = (uint8_t)object->Get_Status();
			motor_enable_test_mode[bus][index] = data->recv.mode;
			motor_enable_test_merror[bus][index] = data->recv.MError;
			motor_enable_test_now_pos[bus][index] = data->recv.Now_Pos;
			motor_enable_test_send_pos[bus][index] = data->send.Pos;
			motor_enable_test_rx_age_ms[bus][index] = (last_rx == 0U)
				? 0xFFFFFFFFU : (uint32_t)(now - last_rx);
		}
	}
	motor_enable_test_all_ready_watch = motor_task_all_ready();

	can1_id1_now_pos = motor_lz_data_can1[0].recv.Now_Pos;
	can1_id2_now_pos = motor_lz_data_can1[1].recv.Now_Pos;
	can1_id3_now_pos = motor_lz_data_can1[2].recv.Now_Pos;
	can1_id4_now_pos = motor_lz_data_can1[3].recv.Now_Pos;
	can2_id1_init_ok = motor_can2_init_ok[0];
	can2_id2_init_ok = motor_can2_init_ok[1];
	can2_id3_init_ok = motor_can2_init_ok[2];
	can2_id4_init_ok = motor_can2_init_ok[3];
	can2_id1_status = (uint8_t)motor_lz_can2[0].Get_Status();
	can2_id2_status = (uint8_t)motor_lz_can2[1].Get_Status();
	can2_id3_status = (uint8_t)motor_lz_can2[2].Get_Status();
	can2_id4_status = (uint8_t)motor_lz_can2[3].Get_Status();
	can2_id1_mode = motor_lz_data_can2[0].recv.mode;
	can2_id2_mode = motor_lz_data_can2[1].recv.mode;
	can2_id3_mode = motor_lz_data_can2[2].recv.mode;
	can2_id4_mode = motor_lz_data_can2[3].recv.mode;
	can2_id1_merror = motor_lz_data_can2[0].recv.MError;
	can2_id2_merror = motor_lz_data_can2[1].recv.MError;
	can2_id3_merror = motor_lz_data_can2[2].recv.MError;
	can2_id4_merror = motor_lz_data_can2[3].recv.MError;
	can2_id1_now_pos = motor_lz_data_can2[0].recv.Now_Pos;
	can2_id2_now_pos = motor_lz_data_can2[1].recv.Now_Pos;
	can2_id3_now_pos = motor_lz_data_can2[2].recv.Now_Pos;
	can2_id4_now_pos = motor_lz_data_can2[3].recv.Now_Pos;
	can2_id1_send_pos = motor_lz_data_can2[0].send.Pos;
	can2_id2_send_pos = motor_lz_data_can2[1].send.Pos;
	can2_id3_send_pos = motor_lz_data_can2[2].send.Pos;
	can2_id4_send_pos = motor_lz_data_can2[3].send.Pos;
}

static uint8_t motor_is_enabled(uint8_t bus, uint8_t idx)
{
#if ROBOT_BENCH_TEST_ENABLE
	uint8_t mask = (bus == 0U) ? ROBOT_BENCH_TEST_CAN1_MASK
	                          : ROBOT_BENCH_TEST_CAN2_MASK;
	return (idx < motor_LZ_N) && ((mask & (1U << idx)) != 0U);
#else
	(void)bus;
	(void)idx;
	return 1U;
#endif
}

static uint8_t motor_bus_has_enabled(uint8_t bus)
{
	for (uint8_t i = 0; i < motor_LZ_N; i++) {
		if (motor_is_enabled(bus, i)) return 1U;
	}
	return 0U;
}

static volatile uint8_t *motor_init_ok_array(uint8_t bus)
{
	return (bus == 0U) ? motor_can1_init_ok : motor_can2_init_ok;
}

static volatile uint32_t *motor_last_rx_array(uint8_t bus)
{
	return (bus == 0U) ? motor_can1_last_rx_tick : motor_can2_last_rx_tick;
}

static Class_Motor_LZ *motor_object(uint8_t bus, uint8_t idx)
{
	return (bus == 0U) ? &motor_lz_can1[idx] : &motor_lz_can2[idx];
}

static motor_lz_control *motor_control_data(uint8_t bus, uint8_t idx)
{
	return (bus == 0U) ? &motor_lz_data_can1[idx] : &motor_lz_data_can2[idx];
}

/* 任一反馈报错、过温或出现 NaN/Inf，均视为电机故障。 */
static uint8_t motor_feedback_valid(const Struct_recv_motor_Lz *feedback)
{
	return feedback->MError == 0U &&
	       isfinite(feedback->Now_Pos) &&
	       isfinite(feedback->Now_Angle) &&
	       isfinite(feedback->Now_W) &&
	       isfinite(feedback->Now_T) &&
	       isfinite(feedback->Now_Temperature) &&
	       feedback->Now_Temperature <= ROBOT_MOTOR_MAX_TEMPERATURE_C;
}

/* last_rx_tick=0 表示从未收到已解码反馈；运行期允许的最大间隔为 100 ms。 */
static uint8_t motor_feedback_fresh(uint32_t last_rx_tick, uint32_t now)
{
	return last_rx_tick != 0U &&
	       (uint32_t)(now - last_rx_tick) <= ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS;
}

#if ROBOT_BENCH_TEST_ENABLE
/* 检查本次台架测试所选的电机是否已经全部完成握手。 */
static uint8_t motor_bench_init_complete(void)
{
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (motor_is_enabled(bus, i) && init_ok[i] == 0U) return 0U;
		}
	}
	return 1U;
}

/* 启动阶段要求所有选中电机连续健康一段时间，避免最后一台刚完成握手时，
 * 先完成的电机恰好超过 100 ms 而触发一次性故障锁存。 */
static uint8_t motor_bench_feedback_ready(uint32_t now)
{
	motor_bench_not_ready_bus_watch = 0xFFU;
	motor_bench_not_ready_index_watch = 0xFFU;
	motor_bench_not_ready_reason_watch = 0U;
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		volatile uint32_t *last_rx = motor_last_rx_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i)) continue;
			uint8_t reason = 0U;
			if (init_ok[i] == 0U) reason = 1U;
			else if (motor_object(bus, i)->Get_Status() != Motor_LZ_Status_ENABLE)
				reason = 2U;
			else if (last_rx[i] == 0U) reason = 3U;
			else if (!motor_feedback_fresh(last_rx[i], now)) reason = 4U;
			else if (!motor_feedback_valid(&motor_control_data(bus, i)->recv))
				reason = 5U;
			if (reason != 0U) {
				motor_bench_not_ready_bus_watch = bus;
				motor_bench_not_ready_index_watch = i;
				motor_bench_not_ready_reason_watch = reason;
				return 0U;
			}
		}
	}
	return 1U;
}

/* 非阻塞使能重试：反馈已经健康就登记成功，否则定时重发“主动上报+使能”。
 * 这与实机可用旧工程的 motor_enable_test_process() 时序一致。 */
static void motor_bench_enable_retry_process(void)
{
	uint32_t now = HAL_GetTick();
	uint8_t retry_due = (int32_t)(now - motor_bench_next_retry_tick) >= 0;

	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		volatile uint32_t *last_rx = motor_last_rx_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i) || init_ok[i] != 0U) continue;

			Class_Motor_LZ *object = motor_object(bus, i);
			motor_lz_control *data = motor_control_data(bus, i);
			if (object->Get_mode() != 0U &&
			    motor_feedback_fresh(last_rx[i], now) &&
			    motor_feedback_valid(&data->recv)) {
				/* 第一条健康反馈的位置就是该电机的安全保持起点。 */
				data->send.Angle = data->recv.Now_Angle;
				data->send.Pos = data->recv.Now_Pos;
				data->send.W = 0.0f;
				data->send.T_ff = 0.0f;
				object->Set_Status(Motor_LZ_Status_ENABLE);
				init_ok[i] = 1U;
			} else if (retry_due) {
				object->active_recv(1U);
				object->enable();
			}
		}
	}

	if (retry_due) motor_bench_next_retry_tick = now + 200U;

	uint8_t init_complete = motor_bench_init_complete();
	uint8_t feedback_ready = motor_bench_feedback_ready(now);
	motor_bench_init_complete_watch = init_complete;
	motor_bench_feedback_ready_watch = feedback_ready;
#if !ROBOT_GLOBAL_FAILSAFE_ENABLE
	/* 已完成一次稳定武装后只记录异常，不再因瞬时掉帧/MError撤销运行许可。 */
	if (motor_bench_runtime_armed != 0U) {
		if (motor_bench_ready_since_tick != 0U) {
			motor_bench_ready_elapsed_watch =
				(uint32_t)(now - motor_bench_ready_since_tick);
		}
		return;
	}
#endif
	if (!init_complete || !feedback_ready) {
		motor_bench_ready_since_tick = 0U;
		motor_bench_runtime_armed = 0U;
		motor_bench_ready_elapsed_watch = 0U;
	} else if (motor_bench_ready_since_tick == 0U) {
		motor_bench_ready_since_tick = now;
		motor_bench_ready_elapsed_watch = 0U;
	} else if ((uint32_t)(now - motor_bench_ready_since_tick) >=
	           ROBOT_BENCH_READY_SETTLE_MS) {
		motor_bench_ready_elapsed_watch = (uint32_t)(now - motor_bench_ready_since_tick);
		motor_bench_runtime_armed = 1U;
	} else {
		motor_bench_ready_elapsed_watch = (uint32_t)(now - motor_bench_ready_since_tick);
	}
}
#endif

/* 首次故障立即锁存，并撤销全部电机的软件武装。 */
static void motor_latch_fault(void)
{
#if ROBOT_GLOBAL_FAILSAFE_ENABLE
	if (motor_fault_latched != 0U) return;
	motor_fault_latched = 1U;
	motor_task_disarm_all();
#else
	/* 非锁存模式保留上一帧有效命令，不向八电机广播失能。 */
	return;
#endif
}


void motor_LZ_Data_send(Class_Motor_LZ *__motor_lz,motor_lz_control *__data,const Enum_Motor_LZ_Mode &__Mode);


void motor_LZ_Data_recv(Class_Motor_LZ *__motor_lz,Struct_recv_motor_Lz *data);


/**
 * @brief CAN1 电机反馈回调，解码后更新对应电机的最近接收时刻
 * @param Header FDCAN 接收头
 * @param *Buffer 8 字节接收数据
 * @return void
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
	
	uint8_t id = Header.Identifier >>8;
	uint8_t mode =(Header.Identifier>>24)&0x1F;
	switch (mode)
	{
		case 0x18:
		case 0x02:
		
		if(id>=1&&id<=motor_LZ_N && motor_is_enabled(0U, id - 1U) &&
		   motor_object_configured[0][id - 1U] != 0U){
			motor_lz_can1[id-1].can_recv(Header.Identifier,Buffer);
			motor_LZ_Data_recv(&motor_lz_can1[id-1],&motor_lz_data_can1[id-1].recv);
			motor_can1_last_rx_tick[id-1] = HAL_GetTick();
		
		}
	
		break;
		default:
		{
			break;
		}
	}

}



/**
 * @brief CAN2 电机反馈回调，解码后更新对应电机的最近接收时刻
 * @param Header FDCAN 接收头
 * @param *Buffer 8 字节接收数据
 * @return void
 */
void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
		
	uint8_t id = Header.Identifier >>8;
	uint8_t mode =(Header.Identifier>>24)&0x1F;
	if (id == 1U) can2_id1_rx_count++;
	else if (id == 2U) can2_id2_rx_count++;
	else if (id == 3U) can2_id3_rx_count++;
	else if (id == 4U) can2_id4_rx_count++;
	switch (mode)
	{
		case 0x18:
		case 0x02:
		
		if(id>=1&&id<=motor_LZ_N && motor_is_enabled(1U, id - 1U) &&
		   motor_object_configured[1][id - 1U] != 0U){
			motor_lz_can2[id-1].can_recv(Header.Identifier,Buffer);
			motor_LZ_Data_recv(&motor_lz_can2[id-1],&motor_lz_data_can2[id-1].recv);
			motor_can2_last_rx_tick[id-1] = HAL_GetTick();
		
		}
	
		break;
		default:
		{
			break;
		}
	}

}

/**
 * @brief CAN3 预留回调，当前未接入电机控制
 * @param Header FDCAN 接收头
 * @param *Buffer 8 字节接收数据
 * @return void
 */
void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch ( Header.Identifier)
	{


	}
}


/**
 * @brief 将驱动内部的最新反馈复制到控制缓存
 * @param __motor_lz 电机对象
 * @param *data 控制层反馈缓存
 * @return void
 */
void motor_LZ_Data_recv(Class_Motor_LZ *__motor_lz,Struct_recv_motor_Lz *data){
    data->MError = __motor_lz->Get_MError();
    data->mode = __motor_lz->Get_mode();
    data->Now_Angle = __motor_lz->Get_Now_Angle();
    data->Now_Pos = __motor_lz->Get_Now_Pos();
    data->Now_W = __motor_lz->Get_Now_W();
    data->Now_T = __motor_lz->Get_Now_T();
    data->Now_Temperature = __motor_lz->Get_Now_Temperature();
}

/**
 * @brief 生成单一电机内部阻抗命令
 * @param __motor_lz 电机对象
 * @param data 控制层命令与反馈缓存
 *
 * 下发 Pos/W/Kp/Kd/T_ff，不再叠加主控侧位置或速度 PID。
 * Pos 和 T_ff 在此使用最终限幅值；命令出现 NaN/Inf 时锁存故障。
 * @return void
 */

void motor_LZ_Data_send(Class_Motor_LZ *__motor_lz,motor_lz_control *__data,const Enum_Motor_LZ_Mode &__Mode){
	(void)__Mode;
	if (!isfinite(__data->send.Pos) || !isfinite(__data->send.W) ||
	    !isfinite(__data->send.Kp) || !isfinite(__data->send.Kd) ||
	    !isfinite(__data->send.T_ff)) {
		motor_latch_fault();
		return;
	}

	fp32 send_pos = __data->send.Pos;
	if (send_pos > __data->send.Pos_max) send_pos = __data->send.Pos_max;
	else if (send_pos < __data->send.Pos_min) send_pos = __data->send.Pos_min;
	__data->send.Pos = send_pos;

	fp32 send_tff = __data->send.T_ff;
	if (send_tff > ROBOT_MOTOR_TFF_LIMIT_NM) send_tff = ROBOT_MOTOR_TFF_LIMIT_NM;
	else if (send_tff < -ROBOT_MOTOR_TFF_LIMIT_NM) send_tff = -ROBOT_MOTOR_TFF_LIMIT_NM;
	__data->send.T_ff = send_tff;

	__motor_lz->Set_Angle(0.0f);
	__motor_lz->Set_Pos(send_pos);
	__motor_lz->Set_T(send_tff);
	__motor_lz->Set_W(__data->send.W);
	__motor_lz->Set_Kp(__data->send.Kp);
	__motor_lz->Set_Kd(__data->send.Kd);
}

/**
 * @brief 单电机启动握手与软件武装
 * @param __motor_lz 电机对象
 * @param bus CAN 总线索引，0=CAN1，1=CAN2
 * @param idx 总线内数组下标，对应电机 ID=idx+1
 * @param *data 控制层命令与反馈缓存
 * @return 1 表示收到新鲜且健康的反馈并完成武装，0 表示失败并保持失能
 *
 * enable() 只发送进入电机模式命令；只有本函数校验反馈成功后，
 * 才设置软件 ENABLE，避免“发过使能帧”被误认为“电机可控”。
 */
static uint8_t motor_LZ_Init(Class_Motor_LZ *__motor_lz, uint8_t bus, uint8_t idx,
	                         motor_lz_control *data)
{
	volatile uint32_t *last_rx = motor_last_rx_array(bus);
	last_rx[idx] = 0U;
	__motor_lz->Set_Status(Motor_LZ_Status_DISABLE);

	uint16_t wait_ms = 0U;
	while (wait_ms < 1000U) {
		/* 参考可用工程要求先打开主动上报，再发送进入电机模式帧。 */
		__motor_lz->active_recv(1U);
		HAL_Delay(2);
		__motor_lz->enable();
		HAL_Delay(2);
		if (last_rx[idx] != 0U) {
			if (!motor_feedback_valid(&data->recv)) {
				__motor_lz->lose();
				return 0U;
			}
			if (__motor_lz->Get_mode() != 0U) break;
		}
		wait_ms += 4U;
	}

	if (last_rx[idx] == 0U || __motor_lz->Get_mode() == 0U ||
	    !motor_feedback_valid(&data->recv)) {
		__motor_lz->lose();
		return 0U;
	}

	__motor_lz->active_recv(1U);
	HAL_Delay(1);
	if (!motor_feedback_fresh(last_rx[idx], HAL_GetTick()) ||
	    !motor_feedback_valid(&data->recv)) {
		__motor_lz->lose();
		return 0U;
	}
	data->send.Angle = data->recv.Now_Angle;
	data->send.Pos = data->recv.Now_Pos;
	data->send.W = 0.0f;
	data->send.T_ff = 0.0f;
	__motor_lz->Set_Status(Motor_LZ_Status_ENABLE);
	return 1U;
}


/**
 * @brief 初始化所选电机；生产模式要求全部 8 个电机依次握手成功
 * @return void
 */
void motor_task_init(){
	HAL_Delay(2000);
	motor_fault_latched = 0U;
	motor_disarm_complete = 0U;
#if ROBOT_BENCH_TEST_ENABLE
	motor_bench_next_retry_tick = HAL_GetTick();
	motor_bench_ready_since_tick = 0U;
	motor_bench_runtime_armed = 0U;
	motor_bench_init_complete_watch = 0U;
	motor_bench_feedback_ready_watch = 0U;
	motor_bench_ready_elapsed_watch = 0U;
	motor_bench_not_ready_bus_watch = 0xFFU;
	motor_bench_not_ready_index_watch = 0xFFU;
	motor_bench_not_ready_reason_watch = 0U;
#endif

	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		volatile uint32_t *last_rx = motor_last_rx_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			init_ok[i] = 0U;
			last_rx[i] = 0U;
			motor_object_configured[bus][i] = 0U;
		}
	}

	for (uint8_t i = 0U; i < motor_LZ_N; i++) {
		motor_lz_data_can1[i].send.Pos_min = SIMPLE_POSITION_MIN_RAD;
		motor_lz_data_can1[i].send.Pos_max = SIMPLE_POSITION_MAX_RAD;
		motor_lz_data_can1[i].send.Kp = 0.0f;
		motor_lz_data_can1[i].send.Kd = 1.0f;
		motor_lz_data_can1[i].send.T_ff = 0.0f;

		motor_lz_data_can2[i].send.Pos_min = SIMPLE_POSITION_MIN_RAD;
		motor_lz_data_can2[i].send.Pos_max = SIMPLE_POSITION_MAX_RAD;
		motor_lz_data_can2[i].send.Kp = 0.0f;
		motor_lz_data_can2[i].send.Kd = 1.0f;
		motor_lz_data_can2[i].send.T_ff = 0.0f;
	}

    can_filter_init();
    if (motor_bus_has_enabled(0U)) bsp_can_init(&hfdcan1,CAN1_Callback);
	if (motor_bus_has_enabled(1U)) bsp_can_init(&hfdcan2,CAN2_Callback);
#if ROBOT_CAN_DIAGNOSTIC_ENABLE
	/* 诊断时三路全部启动，避免把板上 CAN3 接口误认为 CAN1/CAN2。 */
	bsp_can_init(&hfdcan3,CAN3_Callback);
#endif

	for (uint8_t bus = 0U; bus < 2U; bus++) {
		FDCAN_HandleTypeDef *hfdcan = (bus == 0U) ? &hfdcan1 : &hfdcan2;
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i)) continue;
			Class_Motor_LZ *object = motor_object(bus, i);
			object->Init(hfdcan, i + 1U, MOTOR_LZ_02, Motor_LZ_T_control);
			motor_object_configured[bus][i] = 1U;
		}
	}

#if ROBOT_CAN_DIAGNOSTIC_ENABLE
	/*
	 * CAN 诊断只需要外设启动、回调安装和电机对象具备 ID。
	 * 不执行八电机握手，也不进入位置控制，避免 all_ready 掩盖底层收发结果。
	 */
	motor_can_diagnostic_poll();
	motor_update_can2_lh_watch();
	return;
#endif

#if ROBOT_BENCH_TEST_ENABLE
	/* 台架模式不在上电阶段阻塞八次、也不因某一台启动较慢而锁死全部电机。
	 * 周期任务会继续重试；八台全就绪前，上层点动逻辑始终保持等待。 */
	motor_bench_enable_retry_process();
#else
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i)) continue;
			Class_Motor_LZ *object = motor_object(bus, i);
			init_ok[i] = motor_LZ_Init(object, bus, i, motor_control_data(bus, i));
			if (init_ok[i] == 0U) {
				motor_latch_fault();
				return;
			}
			HAL_Delay(1);
		}
	}
#endif

	motor_update_can2_lh_watch();
#if !ROBOT_BENCH_TEST_ENABLE
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (motor_is_enabled(bus, i) && init_ok[i] == 0U) {
				motor_latch_fault();
				return;
			}
		}
	}
#endif
}

/**
 * @brief 锁存故障并撤销全部已配置电机的软件武装
 *
 * 首次调用把位置目标锚定到当前反馈，清零 W/Kp/Kd/T_ff 并调用 lose()；
 * 后续调用直接返回，避免持续占用 CAN。硬件停机帧的协议疑点见 motor_LZ.cpp。
 */
void motor_task_disarm_all()
{
	motor_fault_latched = 1U;
	if (motor_disarm_complete != 0U) return;
	motor_disarm_complete = 1U;
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			Class_Motor_LZ *object = motor_object(bus, i);
			motor_lz_control *data = motor_control_data(bus, i);
			data->send.W = 0.0f;
			data->send.Kp = 0.0f;
			data->send.Kd = 0.0f;
			data->send.T_ff = 0.0f;
			if (isfinite(data->recv.Now_Pos)) data->send.Pos = data->recv.Now_Pos;
			if (motor_object_configured[bus][i] != 0U) object->lose();
			else object->Set_Status(Motor_LZ_Status_DISABLE);
		}
	}
	motor_update_can2_lh_watch();
}

// 查询故障锁存状态；只能通过重新初始化清除
uint8_t motor_task_faulted()
{
	return motor_fault_latched;
}

/* 生产模式要求 8 台全部正常；台架模式只检查掩码选中的电机。
 * 所有被检查电机都必须 init_ok、已武装、反馈健康且未超过 100 ms。 */
uint8_t motor_task_all_ready()
{
	if (motor_fault_latched != 0U) return 0U;
#if ROBOT_BENCH_TEST_ENABLE
	if (motor_bench_runtime_armed == 0U) return 0U;
#endif
	uint32_t now = HAL_GetTick();
	uint8_t checked = 0U;
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		volatile uint32_t *last_rx = motor_last_rx_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i)) continue;
			checked = 1U;
#if ROBOT_GLOBAL_FAILSAFE_ENABLE
			if (init_ok[i] == 0U ||
			    motor_object(bus, i)->Get_Status() != Motor_LZ_Status_ENABLE ||
			    !motor_feedback_fresh(last_rx[i], now) ||
			    !motor_feedback_valid(&motor_control_data(bus, i)->recv)) {
				return 0U;
			}
#else
			/* 上电武装成功后不再用运行期反馈异常阻断上层控制。 */
			(void)init_ok;
			(void)last_rx;
			(void)now;
#endif
		}
	}
	return checked;
}

uint8_t motor_task_motor_ready(uint8_t bus, uint8_t index)
{
	if (bus > 1U || index >= motor_LZ_N || motor_fault_latched != 0U ||
	    !motor_is_enabled(bus, index) ||
	    motor_object_configured[bus][index] == 0U) {
		return 0U;
	}

	volatile uint8_t *init_ok = motor_init_ok_array(bus);
	volatile uint32_t *last_rx = motor_last_rx_array(bus);
	return init_ok[index] != 0U &&
	       motor_object(bus, index)->Get_Status() == Motor_LZ_Status_ENABLE &&
	       motor_feedback_fresh(last_rx[index], HAL_GetTick()) &&
	       motor_feedback_valid(&motor_control_data(bus, index)->recv);
}

/* 运行期只监控当前模式选中的电机；任一异常即触发整机故障锁存。 */
static uint8_t motor_runtime_fault_present(void)
{
	uint32_t now = HAL_GetTick();
	for (uint8_t bus = 0U; bus < 2U; bus++) {
		volatile uint8_t *init_ok = motor_init_ok_array(bus);
		volatile uint32_t *last_rx = motor_last_rx_array(bus);
		for (uint8_t i = 0U; i < motor_LZ_N; i++) {
			if (!motor_is_enabled(bus, i)) continue;
			if (init_ok[i] == 0U ||
			    motor_object(bus, i)->Get_Status() != Motor_LZ_Status_ENABLE ||
			    !motor_feedback_fresh(last_rx[i], now) ||
			    !motor_feedback_valid(&motor_control_data(bus, i)->recv)) {
				return 1U;
			}
		}
	}
	return 0U;
}

/**
 * @brief 周期安全检查并向已武装电机发送内部阻抗控制帧
 * @return void
 */
void motor_task(){
	motor_update_can2_lh_watch();
#if ROBOT_CAN_DIAGNOSTIC_ENABLE
	/* 诊断发送由 main 主循环桩执行；TIM8 在此只刷新观察量。 */
	return;
#endif
#if ROBOT_BENCH_TEST_ENABLE
	/* 未成功的电机持续握手；全部成功前不进入下面的控制帧发送。 */
	motor_bench_enable_retry_process();
	if (!motor_bench_init_complete() || motor_bench_runtime_armed == 0U) return;
#endif
	if (motor_fault_latched != 0U) return;
#if ROBOT_GLOBAL_FAILSAFE_ENABLE
	if (motor_runtime_fault_present()) {
		motor_latch_fault();
		return;
	}
#endif

    	for(uint8_t i=0;i<motor_LZ_N;i++){
        if (!motor_is_enabled(0U, i) || motor_lz_can1[i].Get_Status() != Motor_LZ_Status_ENABLE) continue;
        motor_LZ_Data_send(&motor_lz_can1[i],&motor_lz_data_can1[i],Motor_LZ_T_control);
		if (motor_fault_latched != 0U) return;
        motor_lz_can1[i].can_send();
	}
	for(uint8_t i=0;i<motor_LZ_N;i++){
		if (!motor_is_enabled(1U, i) || motor_lz_can2[i].Get_Status() != Motor_LZ_Status_ENABLE) continue;
		motor_LZ_Data_send(&motor_lz_can2[i],&motor_lz_data_can2[i],Motor_LZ_T_control);
		if (motor_fault_latched != 0U) return;
    	motor_lz_can2[i].can_send();
		if (i == 0U) can2_id1_tx_count++;
		else if (i == 1U) can2_id2_tx_count++;
		else if (i == 2U) can2_id3_tx_count++;
		else if (i == 3U) can2_id4_tx_count++;
	}
}

void motor_can_diagnostic_poll()
{
#if ROBOT_CAN_DIAGNOSTIC_ENABLE && ROBOT_CAN_DIAGNOSTIC_TX_ENABLE
	static uint32_t last_send_tick = 0U;
	static uint32_t sequence = 0U;
	static uint8_t first_send = 1U;
	uint32_t now = HAL_GetTick();

	if (first_send == 0U && (uint32_t)(now - last_send_tick) < 500U) return;
	first_send = 0U;
	last_send_tick = now;
	sequence++;

	/* 标准帧不会被灵足电机当成扩展控制帧，USB-CAN 也更容易直接显示。 */
	uint8_t can1_data[8] = {
		'C', 'A', 'N', '1',
		(uint8_t)(sequence >> 24), (uint8_t)(sequence >> 16),
		(uint8_t)(sequence >> 8), (uint8_t)sequence,
	};
	uint8_t can2_data[8] = {
		'C', 'A', 'N', '2',
		(uint8_t)(sequence >> 24), (uint8_t)(sequence >> 16),
		(uint8_t)(sequence >> 8), (uint8_t)sequence,
	};
	uint8_t can3_data[8] = {
		'C', 'A', 'N', '3',
		(uint8_t)(sequence >> 24), (uint8_t)(sequence >> 16),
		(uint8_t)(sequence >> 8), (uint8_t)sequence,
	};

	fdcan_send_data_stand(&hfdcan1, 0x123U, can1_data, FDCAN_DLC_BYTES_8);
	motor_can_diag_ping_count[0]++;
	fdcan_send_data_stand(&hfdcan2, 0x124U, can2_data, FDCAN_DLC_BYTES_8);
	motor_can_diag_ping_count[1]++;
	fdcan_send_data_stand(&hfdcan3, 0x125U, can3_data, FDCAN_DLC_BYTES_8);
	motor_can_diag_ping_count[2]++;
#endif
}
