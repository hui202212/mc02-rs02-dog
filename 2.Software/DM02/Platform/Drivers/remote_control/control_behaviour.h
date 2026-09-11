#ifndef CONTROL_BEHAVIOUR_H
#define CONTROL_BEHAVIOUR_H

#include "struct_typedef.h"
#include "remote_control.h"

/*==============================================================================
 * 机器人控制模式状态机
 *
 * 仅保留趴下、起立、站立、前后走、原地转弯和故障状态。
 * Get_up 完成后由上层切到 stand；Get_down 完成后保持趴姿，等待新的 START
 * 请求再进入 Get_up。两种动作执行期间，摇杆都不能跳过姿态过程。
 *============================================================================*/
enum remote_control_mode
{
    Dog_control_system_init = 0,
    Dog_control_Get_down    = 1,
    Dog_control_Get_up      = 2,
    Dog_control_stand       = 3,
    Dog_control_walk        = 4,
    Dog_control_turn        = 5,
    Dog_control_fault       = 6,
};

/*==============================================================================
 * 行为控制类
 *
 * 职责：读遥控器 → 输出最小状态机 + 前进/原地转向指令
 *   vx: 前后指令  [-1.0, 1.0]   左摇杆上下 ch[3]，无量纲
 *   vy: 恒为 0；并联五连杆腿只在机身 X/Z 平面运动，没有主动侧向自由度
 *   wz: 转向指令  [-1.0, 1.0]   右摇杆左右 ch[0]，无量纲
 *============================================================================*/
class behaviour_control_class {
public:
    void Init();
    void mode_set();
    void Set_control_mode(remote_control_mode mode);

    inline remote_control_mode Get_control_mode() const;
    inline fp32 Get_VX() const;
    inline fp32 Get_VY() const;
    inline fp32 Get_WZ() const;

protected:
    remote_control_mode control_mode;
    fp32 vx;   /* [-1.0, 1.0] */
    fp32 vy;   /* 固定为 0 */
    fp32 wz;   /* [-1.0, 1.0] */

    /* 按键沿检测与运动互锁状态：重连不误触，换向/换模式前必须回中。 */
    uint16_t last_buttons;
    int8_t active_direction;
    bool center_seen;
    bool button_input_ready;
};

/* --- inline accessors --- */
inline remote_control_mode behaviour_control_class::Get_control_mode() const
    { return control_mode; }

inline fp32 behaviour_control_class::Get_VX() const
    { return vx; }

inline fp32 behaviour_control_class::Get_VY() const
    { return vy; }

inline fp32 behaviour_control_class::Get_WZ() const
    { return wz; }

#endif
