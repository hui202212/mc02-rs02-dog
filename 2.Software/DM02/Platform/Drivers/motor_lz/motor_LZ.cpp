//
// 初始版本：20159，2026/3/13。
//

#include "motor_LZ.h"
#include "math_support.h"

// 控制帧数据缓存
uint8_t motor_send_LZ_Data[8];
// 进入/退出电机模式使用的固定数据
uint8_t motor_send_LZ_Data_lose_enable[8]={00,00,00,00,00,00,00,00};
// 主动上报配置数据
uint8_t motor_send_LZ_Data_active_recv[8]={01,02,03,04,05,06,00,00};
// 修改 CAN ID 和设置机械零位使用的配置数据
uint8_t motor_send_LZ_Data_zero_set_CAN_ID[8] = {01,00,00,00,00,00,00,00};


/**
 * @brief 绑定 CAN、ID 和电机型号，并装载对应协议量程
 * @param hfdcan CAN 外设
 * @param __id 电机 CAN ID
 * @param __model 电机型号
 * @param __control_mode 旧接口命令单位标记；只有 Angle 模式会把 deg 换算为 rad
 * @return void
 */
void Class_Motor_LZ::Init(FDCAN_HandleTypeDef *hfdcan,uint16_t __id,const motor_LZ_Model &__model,
        const Enum_Motor_LZ_Mode &__control_mode)
{
    FDcan = hfdcan;
    CAN_id = __id;
    model = __model;
    control_mode = __control_mode;
    switch (model)
    {
        case MOTOR_LZ_02:
            Pos_Max=12.57f;
            W_Max = 44.00f;
            T_Max = 17.0f;
        break;
        case MOTOR_LZ_05:
            Pos_Max=12.57f;
            W_Max = 50.00f;
            T_Max = 5.5f;
        break;
        default:
            /* 生产固件使用 MOTOR_LZ_02；误选其他型号时采用保守量程。 */
            Pos_Max = 12.57f;
            W_Max = 44.00f;
            T_Max = 5.5f;
        break;
    
    }

}
/**
 * @brief 组装灵足协议扩展帧 ID：功能码、数据域和电机 ID
 * @param Id_data 16 位数据域
 * @return 29 位扩展帧 ID
 */
uint32_t Class_Motor_LZ::send_data(uint32_t Id_data){
    uint32_t motor_LZ_send_box=0;
    motor_LZ_send_box = ((uint32_t)mode)<<24 | Id_data<<8 | CAN_id;
	return motor_LZ_send_box;
}

/**
 * @brief 发送进入电机模式命令，并允许驱动层随后发送控制帧
 * @return void
 */
void Class_Motor_LZ::enable(){
    mode = CANCOM_MOTOR_IN;
    /* 参考实机可用工程：进入模式帧发出后先允许后续控制帧，反馈仍由上层校验。 */
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
    fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_lose_enable,8);
}

/**
 * @brief 清除软件武装状态，并发送旧工程保留的退出帧
 *
 * 按参考实机可用工程使用 CANCOM_MOTOR_RESET，同时清除本地 ENABLE 状态。
 * @return void
 */
void Class_Motor_LZ::lose(){
    mode = CANCOM_MOTOR_RESET;
    Motor_LZ_Status = Motor_LZ_Status_DISABLE;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_lose_enable,8);
}

/**
 * @brief 配置电机主动上报
 * @param F_CMD 1 开启，0 关闭
 * @return void
 */
void Class_Motor_LZ::active_recv(uint8_t F_CMD){
    mode = CANCOM_MODE_ACTIVE_RECV;
    motor_send_LZ_Data_active_recv[6]=(F_CMD==1);
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_active_recv,8);
}

/**
 * @brief 写入机械零位；只允许在明确的离地标定流程中调用
 * @return void
 */
void Class_Motor_LZ::zero(){
    lose();
    mode = CANCOM_MOTOR_ZERO;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id),motor_send_LZ_Data_zero_set_CAN_ID,8);
    enable();
}

/**
 * @brief 旧工程保留的 CAN ID 修改接口，当前禁止实机调用
 * @param set_id 新 CAN ID
 *
 * 安全注意：当前实现发送 CANCOM_MOTOR_ZERO，未发送本驱动已有的
 * CANCOM_MOTOR_ID，随后只修改本地 CAN_id。核对并修正前调用可能误写
 * 机械零位并造成通信失联。
 * @return void
 */
void Class_Motor_LZ::motor_set_CAN_ID(uint8_t set_id){
    lose();
    mode = CANCOM_MOTOR_ZERO;
	fdcan_send_data_Exten(FDcan,send_data(motor_LZ_user_id<<8 | set_id),motor_send_LZ_Data_zero_set_CAN_ID,8);
    CAN_id=set_id;
}

/**
 * @brief 解码反馈数据；收到反馈不会自动重新武装电机
 * @param Data 反馈扩展帧 ID
 * @param *data 8 字节反馈数据
 * @return void
 */
void Class_Motor_LZ::can_recv(uint32_t Data , uint8_t *data){
    recv.Now_Pos = uint_to_float((data[0]<<8|data[1]),-Pos_Max,Pos_Max,16);
    recv.Now_Angle = recv.Now_Pos / MATH_RPM_TO_RADPS;
    recv.Now_W =  uint_to_float((data[2]<<8|data[3]),-W_Max,W_Max,16);
    recv.Now_T =  uint_to_float((data[4]<<8|data[5]),-T_Max,T_Max,16);
    recv.MError= Data>>16&0x3F;
	recv.mode  = Data>>22&0x03;
    recv.Now_Temperature = (fp32)(data[6]<<8|data[7])/10.0f;
    /* 收到合法电机反馈即确认软件在线；运行期健康检查仍会处理报错和超时。 */
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
}

/**
 * @brief 软件已武装时，限幅、编码并发送内部阻抗控制帧
 * @return void
 */
void Class_Motor_LZ::can_send(){
    if(Motor_LZ_Status == Motor_LZ_Status_ENABLE){
        mode = CANCOM_MOTOR_CTRL;

        if(control_mode == Motor_LZ_Angle_control) Pos = Angle * MATH_DEG_TO_RAD;
        
        Pos = motor_max_min(Pos,Pos_Max,-Pos_Max);
        T = motor_max_min(T,T_Max,-T_Max);
        W = motor_max_min(W,W_Max,-W_Max);
	    Kd = motor_max_min(Kd,5.0f,0.0f);
	    Kp = motor_max_min(Kp,500.0f,0.0f);

        uint16_t motor_lz_send[5]; // 位置、速度、力矩、Kp、Kd
        
        motor_lz_send[0] = (uint16_t)((Pos+Pos_Max)*65535/(Pos_Max+Pos_Max));
    	motor_lz_send[1] = (uint16_t)((W+W_Max)*65535/(W_Max+W_Max));
        motor_lz_send[2] = (uint16_t)((T+T_Max)*65535/(T_Max+T_Max));
    	motor_lz_send[3] = (uint16_t)((Kp*65535/500.0f));
	    motor_lz_send[4] = (uint16_t)((Kd*65535/5.00f));

        motor_send_LZ_Data[0]=motor_lz_send[0]>>8;
    	motor_send_LZ_Data[1]=motor_lz_send[0];
	
	    motor_send_LZ_Data[2]=motor_lz_send[1]>>8;
	    motor_send_LZ_Data[3]=motor_lz_send[1];
	
	    motor_send_LZ_Data[4]=motor_lz_send[3]>>8;
    	motor_send_LZ_Data[5]=motor_lz_send[3];	
	
	    motor_send_LZ_Data[6]=motor_lz_send[4]>>8;
	    motor_send_LZ_Data[7]=motor_lz_send[4];
        
	    fdcan_send_data_Exten(FDcan,send_data(motor_lz_send [2]),motor_send_LZ_Data,8);
    }
}
