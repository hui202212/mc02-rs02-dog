//
// Created by 20159 on 2026/3/12.
//

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"
#include "stm32h7xx_hal.h"

//定义CAN回调函数
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

struct CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[8];
};

extern bool system_can[3];

/*
 * CAN 底层诊断量，下标 0/1/2 对应 CAN1/CAN2/CAN3。
 * 它们在协议解码之前统计，因此可以区分“没有物理回包”和“协议没认出来”。
 */
extern volatile uint8_t can_diag_filter_ok[3];
extern volatile uint8_t can_diag_start_status[3];
extern volatile uint8_t can_diag_notification_status[3];
extern volatile uint32_t can_diag_tx_attempt_count[3];
extern volatile uint32_t can_diag_tx_ok_count[3];
extern volatile uint32_t can_diag_tx_error_count[3];
extern volatile uint32_t can_diag_rx_irq_count[3];
extern volatile uint32_t can_diag_rx_frame_count[3];
extern volatile uint32_t can_diag_last_rx_identifier[3];
extern volatile uint32_t can_diag_last_rx_id_type[3];
extern volatile uint32_t can_diag_last_rx_dlc[3];
extern volatile uint32_t can_diag_hal_error_code[3];
extern volatile uint32_t can_diag_tx_fifo_free[3];
extern volatile uint32_t can_diag_last_error_code[3];
extern volatile uint32_t can_diag_bus_off[3];
extern volatile uint32_t can_diag_error_passive[3];
extern volatile uint32_t can_diag_error_warning[3];
extern volatile uint32_t can_diag_tx_error_counter[3];
extern volatile uint32_t can_diag_rx_error_counter[3];

void can_filter_init();
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function);

uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);

#endif //BSP_FDCAN_H
