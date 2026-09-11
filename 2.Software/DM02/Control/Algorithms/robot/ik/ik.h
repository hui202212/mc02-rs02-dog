#ifndef IK_H
#define IK_H

#include "struct_typedef.h"


extern const fp32 DH1[5][4];
extern const fp32 DH2[5][4];
extern const fp32 DH3[5][4];
extern const fp32 DH4[5][4];

/*
 * 五连杆在各腿安装平面的 X-Z 平面内运动。
 * 对外足端坐标使用机身坐标系和 mm：+X 向前、+Y 向左、+Z 向上；
 * Y 仅随目标保存，不参与当前平面五连杆逆解。腿号 1~4 依次为 RF/LF/RH/LH。
 */
struct  robot_connection_12bot
{
	fp32 set_x;
    fp32 set_y;
    fp32 set_z;
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 motor_out_pos1;
    fp32 motor_out_pos2;
    fp32 motor_out_pos3;
    fp32 out_pos1;
    fp32 out_pos2;
    fp32 out_pos3;
    fp32 out_Angle1;
    fp32 out_Angle2;
    fp32 out_Angle3;
};

class kinematics_connection_12bot_robot{
    
    public:

    void Init(const uint8_t &__leg,const fp32 __DH[5][4]);

    /* 逆解失败返回 false，并保持上一条有效电机命令不变。 */
    bool leg_inverse_calculation(const fp32 &__x,const fp32 &__y,const fp32 &__z);

    /* 用 CAN 对内的电机位置做正解，同时把反馈设为下一次选支参考。 */
    void leg_correct_solution(const fp32 &__pos1,const fp32 &__pos2,const fp32 &__pos3);

    /* 模式切换时用真实电机反馈重建连续性参考，并允许重新识别实机支链。 */
    void Set_Motor_Reference(const fp32 &__pos1,const fp32 &__pos2);

    inline bool Get_Last_Target_Valid()const;

    /* Out_Pos 为左/右主动杆几何角；Motor_Out_Pos 为该腿 CAN 顺序下的命令角。 */
    inline fp32 Get_Out_Pos1()const;
    
    inline fp32 Get_Out_Pos2()const;

    inline fp32 Get_Out_Pos3()const;

    inline fp32 Get_Motor_Out_Pos1()const;
    
    inline fp32 Get_Motor_Out_Pos2()const;

    inline fp32 Get_Motor_Out_Pos3()const;

    inline fp32 Get_Out_Angle1()const;
    
    inline fp32 Get_Out_Angle2()const;

    inline fp32 Get_Out_Angle3()const;

    inline fp32 Get_X()const;

    inline fp32 Get_Y()const;
    
    inline fp32 Get_Z()const;

    protected:
    uint8_t leg;
    fp32 h;
    fp32 h1;
    fp32 h2;
    fp32 b;
    fp32 W;
    fp32 L1;
    fp32 L2;
    fp32 L3;

    bool last_target_valid;
    bool motor_reference_valid;
    bool branch_locked;
    uint8_t locked_left_branch;
    uint8_t locked_right_branch;

    robot_connection_12bot control;

    void inverse_calculation(const fp32 &__x,const fp32 &__y,const fp32 &__z);
    void correct_solution(const fp32 &__pos1,const fp32 &__pos2,const fp32 &__pos3);
};

inline fp32 kinematics_connection_12bot_robot::
        Get_Out_Pos1()const{
    
        return control.out_pos1;
}
    
inline fp32 kinematics_connection_12bot_robot:: 
    Get_Out_Pos2()const{
    
    return control.out_pos2;
}

inline fp32 kinematics_connection_12bot_robot:: 
    Get_Out_Pos3()const{
    
    return control.out_pos3;
}

inline fp32 kinematics_connection_12bot_robot:: Get_Motor_Out_Pos1()const{
    return control.motor_out_pos1;
}
    
inline fp32 kinematics_connection_12bot_robot:: Get_Motor_Out_Pos2()const{
    return control.motor_out_pos2;
}

inline fp32 kinematics_connection_12bot_robot:: Get_Motor_Out_Pos3()const{
    return control.motor_out_pos3;
}

inline fp32 kinematics_connection_12bot_robot::
        Get_Out_Angle1()const{
    return control.out_Angle1;
}
    
inline fp32 kinematics_connection_12bot_robot:: 
        Get_Out_Angle2()const{
    return control.out_Angle2;
}

inline fp32 kinematics_connection_12bot_robot:: 
        Get_Out_Angle3()const{
    return control.out_Angle3;
}

inline fp32 kinematics_connection_12bot_robot:: 
        Get_X()const{

    return control.x;
}

inline fp32 kinematics_connection_12bot_robot:: 
        Get_Y()const{

    return control.y;
}

inline fp32 kinematics_connection_12bot_robot:: 
        Get_Z()const{

    return control.z;
}

inline bool kinematics_connection_12bot_robot::
        Get_Last_Target_Valid()const{

    return last_target_valid;
}


#endif
