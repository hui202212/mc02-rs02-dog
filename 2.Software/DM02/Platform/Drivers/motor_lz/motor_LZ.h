//
// 初始版本：20159，2026/3/13。
//

#ifndef MOTOR_LZ_H
#define MOTOR_LZ_H

#include "struct_typedef.h"
#include "bsp_fdcan.h"

// 主机侧 CAN ID
#define motor_LZ_user_id 0

/* 软件发送状态：ENABLE 仅表示上层已确认初始化并允许下发控制帧，
 * 与“收到过反馈”是两件事；反馈健康由 motor_task 单独监控。 */
enum Enum_Motor_LZ_Status
{
    Motor_LZ_Status_DISABLE = 0,
    Motor_LZ_Status_ENABLE,
};

/* 旧接口的命令单位标记：只有 Angle 模式会在驱动内把 deg 换算为 rad；
 * 当前生产链直接下发 Pos/W/Kp/Kd/T_ff，调用参数 __Mode 不再切换控制算法。 */
enum Enum_Motor_LZ_Mode
{
    Motor_LZ_Pos_control = 0,
    Motor_LZ_Angle_control,
	Motor_LZ_T_control,
	Motor_LZ_W_control,
};

// 电机型号决定协议量程
enum motor_LZ_Model{
	MOTOR_LZ_00=0,
	MOTOR_LZ_01,
	MOTOR_LZ_02,
	MOTOR_LZ_03,
	MOTOR_LZ_04,
	MOTOR_LZ_05,
};

// 灵足扩展帧功能码
enum canComMode{
	CANCOM_ANNOUNCE_DEVID = 0,//通告设备 ID
	CANCOM_MOTOR_CTRL, // 电机控制
	CANCOM_MOTOR_FEEDBACK, // 电机反馈
	CANCOM_MOTOR_IN, // 进入电机模式
	CANCOM_MOTOR_RESET, // 复位
	CANCOM_MOTOR_CALI, // 高速编码器标定
	CANCOM_MOTOR_ZERO, // 设置机械零位
	CANCOM_MOTOR_ID, // 设置 ID
	CANCOM_PARA_WRITE, //参数-写入
	CANCOM_PARA_READ, //参数-读取
	CANCOM_CALI_ING, //编码器标定中
	CANCOM_CALI_RST, //编码器标定结果
	CANCOM_PARA_STR_INFO, //参数-字符串信息
	CANCOM_MOTOR_BRAKE, // 进入刹车模式
	CANCOM_FAULT_WARN, //故障和警告信息
	CANCOM_MODE_TOTAL, //电机数据保存帧
	CANCOM_MODE_Bd,		//电机波特率修改帧
	CANCOM_MODE_ACTIVE_RECV = 0x18,//电机主动上报帧
	CANCOM_MODE_AGREEMENT,//电机协议修改帧
};

// 最近一次电机反馈的解码结果
struct Struct_recv_motor_Lz
{
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;                  // 位置(rad)
	fp32 Now_Angle;                 // 兼容旧接口的位置(deg)
    fp32 Now_W;                    // 角速度(rad/s)
    fp32 Now_T;                    // 力矩(N*m)
    fp32 Now_Temperature;          // 温度(摄氏度)
};

/* 电机内部阻抗命令：Pos/W/Kp/Kd 与前馈力矩 T_ff。
 * Pos_max/Pos_min 和 T_ff 的最终安全限幅由 motor_task 执行。 */
struct Struct_send_motor_Lz
{
	fp32 Pos_max;                    // 位置上限(rad)
	fp32 Pos_min;                    // 位置下限(rad)
	fp32 Angle;                      // 兼容旧接口的位置(deg)
	fp32 Pos;                        // 目标位置(rad)
	fp32 T;                          // 兼容旧接口的力矩(N*m，当前生产路径未读取)
	fp32 W;                          // 目标角速度(rad/s)
	fp32 Kp;
	fp32 Kd;
	fp32 T_ff;                       // 上层前馈力矩(N*m)
};

// 单个电机的命令与反馈缓存
struct motor_lz_control
{
	Struct_send_motor_Lz send;
	Struct_recv_motor_Lz recv;
};

// 灵足电机协议驱动
class Class_Motor_LZ{
	
	// 公共接口
	public:
	// 绑定总线、ID、电机型号和旧接口命令单位标记
	void Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,const Enum_Motor_LZ_Mode &__control_mode);

	/* 仅发送“进入电机模式”命令，不改变软件 ENABLE 状态；
	 * 上层收到并校验反馈后才可通过 Set_Status() 武装。 */
	void enable();

	/* 立即清除软件武装状态。当前发送功能码存在协议疑点，见实现处；
	 * 未核实并修正前，不能把该调用视为电机已收到硬件停机命令。 */
	void lose();
	
	// 配置主动上报：1 开启，0 关闭
	void active_recv(uint8_t F_CMD);
	
	// 写入机械零位，仅用于明确的离地标定流程
	void zero();

	// 仅在软件状态为 ENABLE 时编码并发送控制帧
	void can_send();

	// 只解码反馈，不自动改变软件 ENABLE 状态
	void can_recv(uint32_t Data , uint8_t *data);

	/* 协议疑点：当前实现未发送 CANCOM_MOTOR_ID，修正前禁止实机调用。 */
	void motor_set_CAN_ID(uint8_t set_id);

	// 状态和命令访问接口
	inline Enum_Motor_LZ_Status Get_Status();

	inline void Set_Status(const Enum_Motor_LZ_Status &__Status);
	
	inline void Set_Pos(const fp32 &__Pos);

	inline fp32 Get_Pos()const;

	inline void Set_Angle(const fp32 &__angle);

	inline fp32 Get_Angle()const;

	inline void Set_W(const fp32 &__w);

	inline fp32 Get_W()const;

	inline void Set_T(const fp32 &__T);

	inline fp32 Get_T()const;

	inline void Set_Kd(const fp32 &__Kd);

	inline fp32 Get_Kd()const;

	inline void Set_Kp(const fp32 &__Kp);

	inline fp32 Get_Kp()const;
	
	inline fp32 Get_Now_Angle()const;

	inline fp32 Get_Now_Pos()const;

	inline fp32 Get_Now_W()const;
	
	inline fp32 Get_Now_T()const;
	
	inline fp32 Get_Now_Temperature()const;

	inline uint8_t Get_MError()const;
	
	inline uint8_t Get_mode()const;
	
	// 内部状态
	protected:
	// 电机 CAN ID
	uint16_t CAN_id;
	// 协议量程对应的电机型号
	motor_LZ_Model model;
	// 所属 FDCAN 外设
	FDCAN_HandleTypeDef *FDcan;
    // 协议最大位置，与电机 PMAX 一致
	float Pos_Max;
    // 协议最大速度，与电机 VMAX 一致
    float W_Max;
    // 协议最大力矩，与电机 TMAX 一致
    float T_Max;

	// 软件武装状态，默认失能
	Enum_Motor_LZ_Status Motor_LZ_Status = Motor_LZ_Status_DISABLE;

	// 最近反馈
	Struct_recv_motor_Lz recv;
	
	// 当前待发送的协议功能码
	enum canComMode mode;
	
	// 上层命令单位/控制路径
	Enum_Motor_LZ_Mode control_mode = Motor_LZ_Pos_control;

	fp32 Angle;
	fp32 Pos;
	fp32 T;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
	// 组装扩展帧 ID
	uint32_t send_data(uint32_t Id_data);
};


inline Enum_Motor_LZ_Status Class_Motor_LZ::Get_Status()
{
	return (Motor_LZ_Status);
}

inline void Class_Motor_LZ::Set_Status(const Enum_Motor_LZ_Status &__Status){
	Motor_LZ_Status = __Status;
}

inline void Class_Motor_LZ::Set_Angle(const fp32 &__angle){
	Angle = __angle;
}

inline fp32 Class_Motor_LZ::Get_Angle() const{
	return (Angle);
}

inline void Class_Motor_LZ::Set_W(const fp32 &__w){
	W=__w;
}

inline fp32 Class_Motor_LZ::Get_W() const{
	return (W);
}

inline void Class_Motor_LZ::Set_T(const fp32 &__T){
	T=__T;
}

inline fp32 Class_Motor_LZ::Get_T()const{
	return (T);
}

inline void Class_Motor_LZ::Set_Kd(const fp32 &__Kd){
	Kd = __Kd;
}

inline fp32 Class_Motor_LZ::Get_Kd()const{
	return Kd;
}

inline void Class_Motor_LZ::Set_Kp(const fp32 &__Kp){
	Kp = __Kp;
}

inline fp32 Class_Motor_LZ::Get_Kp() const{
	return (Kp);
}

inline fp32 Class_Motor_LZ::Get_Now_Angle()const{
	return (recv.Now_Angle);
}	

inline fp32 Class_Motor_LZ::Get_Now_Pos()const{
	return (recv.Now_Pos);
}	

inline fp32 Class_Motor_LZ::Get_Now_W()const{
	return (recv.Now_W);
}

inline fp32 Class_Motor_LZ::Get_Now_T()const{
	return (recv.Now_T);
}
	
inline fp32 Class_Motor_LZ::Get_Now_Temperature()const{
	return (recv.Now_Temperature);
}

inline uint8_t Class_Motor_LZ::Get_MError()const{
	return (recv.MError);
}
	
inline uint8_t Class_Motor_LZ::Get_mode()const{
	return (recv.mode);
}

inline void Class_Motor_LZ::Set_Pos(const fp32 &__Pos){
	Pos = __Pos;
}

inline fp32 Class_Motor_LZ::Get_Pos()const{
	return Pos;
}

#endif //MOTOR_LZ_H
