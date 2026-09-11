#include "remote_control.h"

#include "gpio.h"
#include "string.h"

#define PS2_POLL_PERIOD_MS          10U
#define PS2_RC_MAX                  660

/* 仅用于兼容 mouse 调试字段；L1/R2 不再参与行为状态机。 */
#define PS2_BTN_R2                  0x0200U
#define PS2_BTN_L1                  0x0400U

Remote_Info_Typedef remote_ctrl;

/* Keil Watch 调试变量：
 *   ps2_ok      = 1 表示本轮 GPIO 软件时序通信和帧头正常
 *   ps2_id      = 0x41 数字模式，0x73 摇杆模式
 *   ps2_buttons = 当前按下的按键位，1 表示按下
 *   ps2_lx/ly   = 左摇杆原始值，约 128 为中位
 *   ps2_rx/ry   = 右摇杆原始值，约 128 为中位
 *   ps2_rx_raw  = 9 字节原始返回帧
 */
volatile uint8_t ps2_rx_raw[9];
volatile uint8_t ps2_id = 0U;
volatile uint8_t ps2_ok = 0U;
volatile uint8_t ps2_lx = 128U;
volatile uint8_t ps2_ly = 128U;
volatile uint8_t ps2_rx = 128U;
volatile uint8_t ps2_ry = 128U;
volatile uint16_t ps2_buttons = 0U;

static uint32_t ps2_last_poll_tick = 0U;

static void ps2_short_delay(void)
{
    for (volatile uint32_t i = 0U; i < 1200U; ++i) {
        __NOP();
    }
}

static void ps2_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = PS2_CLK_Pin | PS2_CMD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PS2_DAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PS2_ATT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PS2_ATT_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t ps2_transfer_byte(uint8_t tx)
{
    uint8_t rx = 0U;

    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        HAL_GPIO_WritePin(PS2_CMD_GPIO_Port,
                          PS2_CMD_Pin,
                          (tx & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ps2_short_delay();

        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
        ps2_short_delay();

        if (HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin) == GPIO_PIN_SET) {
            rx |= (uint8_t)(1U << bit);
        }

        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
        ps2_short_delay();
        tx >>= 1U;
    }

    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
    return rx;
}

static int16_t ps2_axis_to_rc(uint8_t value, bool invert)
{
    int32_t centered = (int32_t)value - 128;
    int32_t scaled;

    if (invert) {
        centered = -centered;
    }

    scaled = centered * PS2_RC_MAX / 127;
    if (scaled > PS2_RC_MAX) {
        scaled = PS2_RC_MAX;
    } else if (scaled < -PS2_RC_MAX) {
        scaled = -PS2_RC_MAX;
    }

    return (int16_t)scaled;
}

static void ps2_set_switch(uint8_t s1, uint8_t s0)
{
    remote_ctrl.rc.s[1] = s1;
    remote_ctrl.rc.s[0] = s0;
}

static void ps2_set_safe_init_mode(void)
{
    /* 上电先声明失联并把旧双拨杆字段置中性，等待首个有效 PS2 帧；
     * 不能用某个动作组合充当“安全默认值”，否则重连会自动触发动作。 */
    memset(&remote_ctrl, 0, sizeof(remote_ctrl));
    remote_ctrl.rc_lost = true;
    remote_ctrl.online_cnt = 0U;
    ps2_set_switch(0U, 0U);
}

static bool ps2_poll_raw(uint8_t *rx_buf)
{
    /* 标准 PS2 读取命令：ATT 拉低后发送 01 42 00 00...
     * 返回第 2 字节是模式 ID，第 3 字节正常应为 0x5A。
     */
    static const uint8_t tx_buf[9] = {
        0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
    };

    if (rx_buf == NULL) {
        return false;
    }

    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_RESET);
    ps2_short_delay();

    for (uint8_t i = 0U; i < 9U; ++i) {
        rx_buf[i] = ps2_transfer_byte(tx_buf[i]);
        ps2_short_delay();
    }

    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);
    return true;
}

static bool ps2_transfer_frame(const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len)
{
    if (tx_buf == NULL || rx_buf == NULL || len == 0U) {
        return false;
    }

    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_RESET);
    ps2_short_delay();

    for (uint8_t i = 0U; i < len; ++i) {
        rx_buf[i] = ps2_transfer_byte(tx_buf[i]);
        ps2_short_delay();
    }

    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);
    ps2_short_delay();
    return true;
}

static void ps2_try_enter_analog_mode(void)
{
    uint8_t rx_buf[9] = {0};

    static const uint8_t enter_config[9] = {
        0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
    };
    static const uint8_t set_analog_lock[9] = {
        0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U
    };
    static const uint8_t exit_config[9] = {
        0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU
    };

    for (uint8_t i = 0U; i < 3U; ++i) {
        ps2_transfer_frame(enter_config, rx_buf, 9U);
        ps2_transfer_frame(set_analog_lock, rx_buf, 9U);
        ps2_transfer_frame(exit_config, rx_buf, 9U);
    }
}

static bool ps2_response_is_valid(const uint8_t *rx_buf)
{
    if (rx_buf == NULL) {
        return false;
    }

    if (rx_buf[2] != 0x5AU) {
        return false;
    }

    return (rx_buf[1] == 0x41U || rx_buf[1] == 0x53U || rx_buf[1] == 0x73U);
}

static void ps2_update_remote_ctrl(const uint8_t *rx_buf)
{
    const uint16_t raw_buttons = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
    const uint16_t pressed = (uint16_t)(~raw_buttons);

    ps2_id = rx_buf[1];
    ps2_buttons = pressed;

    if (ps2_id == 0x41U) {
        /* 0x41 是数字模式，后 4 字节不是摇杆模拟量，必须置中避免误触发行走。 */
        ps2_rx = 128U;
        ps2_ry = 128U;
        ps2_lx = 128U;
        ps2_ly = 128U;
    } else {
        ps2_rx = rx_buf[5];
        ps2_ry = rx_buf[6];
        ps2_lx = rx_buf[7];
        ps2_ly = rx_buf[8];
    }

    /* 统一到旧遥控数据接口：ch[3] 仅负责前后，ch[0] 仅负责原地转弯。
     * 并联五连杆没有主动侧向自由度，因此 ch[2] 明确保持为零。 */
    /* 实机约定：右摇杆向左=左转、向右=右转。 */
    remote_ctrl.rc.ch[0] = ps2_axis_to_rc(ps2_rx, true);
    remote_ctrl.rc.ch[1] = ps2_axis_to_rc(ps2_ry, true);
    remote_ctrl.rc.ch[2] = 0;
    remote_ctrl.rc.ch[3] = ps2_axis_to_rc(ps2_ly, true);
    remote_ctrl.rc.ch[4] = 0;

    remote_ctrl.key.v = pressed;
    remote_ctrl.mouse.x = remote_ctrl.rc.ch[0];
    remote_ctrl.mouse.y = remote_ctrl.rc.ch[3];
    remote_ctrl.mouse.z = 0;
    remote_ctrl.mouse.press_l = (pressed & PS2_BTN_L1) ? 1U : 0U;
    remote_ctrl.mouse.press_r = (pressed & PS2_BTN_R2) ? 1U : 0U;

    /* 行为层直接从 key.v 做按键上升沿检测；旧双拨杆字段不再承载模式。 */
    ps2_set_switch(0U, 0U);

    remote_ctrl.online_cnt = 0xFAU;
    remote_ctrl.rc_lost = false;
}

void remote_control_init()
{
    ps2_set_safe_init_mode();
    ps2_gpio_init();
    ps2_try_enter_analog_mode();
    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);
    ps2_last_poll_tick = 0U;
}

void remote_control_update()
{
    uint8_t rx_buf[9] = {0};
    uint32_t now = HAL_GetTick();

    if ((now - ps2_last_poll_tick) < PS2_POLL_PERIOD_MS) {
        return;
    }
    ps2_last_poll_tick = now;

    if (!ps2_poll_raw(rx_buf)) {
        ps2_ok = 0U;
        return;
    }

    for (uint8_t i = 0U; i < 9U; ++i) {
        ps2_rx_raw[i] = rx_buf[i];
    }

    if (!ps2_response_is_valid(rx_buf)) {
        ps2_ok = 0U;
        return;
    }

    ps2_ok = 1U;
    ps2_update_remote_ctrl(rx_buf);
}

void Remote_Message_Moniter(Remote_Info_Typedef  *__remote_ctrl)
{
    if (__remote_ctrl == NULL) {
        return;
    }

    /* 有效帧把计数器恢复到高水位；长期未刷新后清空所有输入并置失联。
     * s[] 保持中性，具体的 stand/保持策略由行为状态机统一决定。 */
    if (__remote_ctrl->online_cnt <= 0x32U) {
        memset(__remote_ctrl, 0, sizeof(Remote_Info_Typedef));
        __remote_ctrl->rc_lost = true;
        __remote_ctrl->rc.s[1] = 0U;
        __remote_ctrl->rc.s[0] = 0U;
    } else if (__remote_ctrl->online_cnt > 0U) {
        __remote_ctrl->online_cnt--;
    }
}
