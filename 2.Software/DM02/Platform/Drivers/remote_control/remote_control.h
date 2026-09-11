#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"

#define RC_CH_VALUE_OFFSET		1024U

/* PS2 原始协议按键低有效；驱动反相后，ps2_buttons / key.v 中1表示按下。
 * 具体障碍赛映射见 docs/障碍赛遥控与路线.md。 */
#define PS2_BUTTON_SELECT_MASK    0x0001U
#define PS2_BUTTON_START_MASK     0x0008U
#define PS2_BUTTON_UP_MASK        0x0010U
#define PS2_BUTTON_RIGHT_MASK     0x0020U
#define PS2_BUTTON_DOWN_MASK      0x0040U
#define PS2_BUTTON_LEFT_MASK      0x0080U
#define PS2_BUTTON_L2_MASK        0x0100U
#define PS2_BUTTON_R2_MASK        0x0200U
#define PS2_BUTTON_L1_MASK        0x0400U
#define PS2_BUTTON_R1_MASK        0x0800U
#define PS2_BUTTON_TRIANGLE_MASK  0x1000U
#define PS2_BUTTON_CIRCLE_MASK    0x2000U
#define PS2_BUTTON_CROSS_MASK     0x4000U
#define PS2_BUTTON_SQUARE_MASK    0x8000U

/* 保留原工程遥控数据外形以兼容现有观察工具。当前行为层只消费 ch[0]
 * (右摇杆左右)、ch[3](左摇杆上下)、key.v 和 rc_lost；s[] 始终为中性，
 * mouse 仅保留调试兼容，不参与机器人模式选择。 */
struct Remote_Info_Typedef
{
    struct
	{
		int16_t ch[5];
		uint8_t s[2];
    } rc;
	struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;

	bool rc_lost;         /*!< 连续无有效帧后的失联标志 */
	uint8_t online_cnt;   /*!< 有效帧刷新、控制周期递减的在线计数 */
} ;



extern Remote_Info_Typedef remote_ctrl;

/* Keil Watch 使用的 PS2 原始通信观察量。 */
extern volatile uint8_t ps2_rx_raw[9];
extern volatile uint8_t ps2_id;
extern volatile uint8_t ps2_ok;
extern volatile uint8_t ps2_lx;
extern volatile uint8_t ps2_ly;
extern volatile uint8_t ps2_rx;
extern volatile uint8_t ps2_ry;
extern volatile uint16_t ps2_buttons;

void remote_control_init();
void remote_control_update();
void Remote_Message_Moniter(Remote_Info_Typedef  *__remote_ctrl);



#endif

