//
// Created by 20159 on 2026/3/12.
//

#include "bsp_fdcan.h"

//CAN_id初始化记录
bool system_can[3];

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

//CAN结构体
CAN_Manage_Object CAN1_Manage_Object;
CAN_Manage_Object CAN2_Manage_Object;
CAN_Manage_Object CAN3_Manage_Object;

volatile uint8_t can_diag_filter_ok[3] = {0U};
volatile uint8_t can_diag_start_status[3] = {0xFFU, 0xFFU, 0xFFU};
volatile uint8_t can_diag_notification_status[3] = {0xFFU, 0xFFU, 0xFFU};
volatile uint32_t can_diag_tx_attempt_count[3] = {0U};
volatile uint32_t can_diag_tx_ok_count[3] = {0U};
volatile uint32_t can_diag_tx_error_count[3] = {0U};
volatile uint32_t can_diag_rx_irq_count[3] = {0U};
volatile uint32_t can_diag_rx_frame_count[3] = {0U};
volatile uint32_t can_diag_last_rx_identifier[3] = {0U};
volatile uint32_t can_diag_last_rx_id_type[3] = {0U};
volatile uint32_t can_diag_last_rx_dlc[3] = {0U};
volatile uint32_t can_diag_hal_error_code[3] = {0U};
volatile uint32_t can_diag_tx_fifo_free[3] = {0U};
volatile uint32_t can_diag_last_error_code[3] = {0U};
volatile uint32_t can_diag_bus_off[3] = {0U};
volatile uint32_t can_diag_error_passive[3] = {0U};
volatile uint32_t can_diag_error_warning[3] = {0U};
volatile uint32_t can_diag_tx_error_counter[3] = {0U};
volatile uint32_t can_diag_rx_error_counter[3] = {0U};

static uint8_t can_index(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1) return 0U;
    if (hfdcan->Instance == FDCAN2) return 1U;
    return 2U;
}

static void record_bus_state(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t index = can_index(hfdcan);
    FDCAN_ProtocolStatusTypeDef protocol;
    FDCAN_ErrorCountersTypeDef counters;

    can_diag_hal_error_code[index] = hfdcan->ErrorCode;
    can_diag_tx_fifo_free[index] = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    if (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol) == HAL_OK) {
        can_diag_last_error_code[index] = protocol.LastErrorCode;
        can_diag_bus_off[index] = protocol.BusOff;
        can_diag_error_passive[index] = protocol.ErrorPassive;
        can_diag_error_warning[index] = protocol.Warning;
    }
    if (HAL_FDCAN_GetErrorCounters(hfdcan, &counters) == HAL_OK) {
        can_diag_tx_error_counter[index] = counters.TxErrorCnt;
        can_diag_rx_error_counter[index] = counters.RxErrorCnt;
    }
}

static void record_rx(uint8_t index, const FDCAN_RxHeaderTypeDef *header)
{
    can_diag_rx_frame_count[index]++;
    can_diag_last_rx_identifier[index] = header->Identifier;
    can_diag_last_rx_id_type[index] = header->IdType;
    can_diag_last_rx_dlc[index] = header->DataLength;
}

/**
 * @brief CAN过滤器配置
 * @param hfdcan  can编号
 * @return void
 */
void can_filter_init( )
{
    FDCAN_FilterTypeDef fdcan_filter;
    HAL_StatusTypeDef status;

    can_diag_filter_ok[0] = 1U;
    can_diag_filter_ok[1] = 1U;
    can_diag_filter_ok[2] = 1U;
   
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    status = HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
    if (status != HAL_OK) can_diag_filter_ok[0] = 0U;
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) can_diag_filter_ok[0] = 0U;
    status = HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
    if (status != HAL_OK) can_diag_filter_ok[0] = 0U;
  
     
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    status = HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
    if (status != HAL_OK) can_diag_filter_ok[1] = 0U;
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) can_diag_filter_ok[1] = 0U;
    status = HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
    if (status != HAL_OK) can_diag_filter_ok[1] = 0U;
    
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    status = HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter);
    if (status != HAL_OK) can_diag_filter_ok[2] = 0U;
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) can_diag_filter_ok[2] = 0U;
    status = HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
    if (status != HAL_OK) can_diag_filter_ok[2] = 0U;


}

/**
 * @brief CAN的初始化
 * @param hfdcan  can编号
 * @param Callback_Function 回调函数传参
 * @return void
 */
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function)
{
	uint8_t index = can_index(hfdcan);
	if (hfdcan->Instance == FDCAN1)
		CAN1_Manage_Object.Callback_Function = Callback_Function;
    else if (hfdcan->Instance == FDCAN2)
        CAN2_Manage_Object.Callback_Function = Callback_Function;
    else if (hfdcan->Instance == FDCAN3)
        CAN3_Manage_Object.Callback_Function = Callback_Function;
    
    if (hfdcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[0]==0){
  
			can_diag_notification_status[index] = (uint8_t)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			can_diag_start_status[index] = (uint8_t)HAL_FDCAN_Start(hfdcan);
			can_diag_hal_error_code[index] = hfdcan->ErrorCode;
			system_can[0]=1;
        }

    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[1]==0){
			
			can_diag_notification_status[index] = (uint8_t)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			can_diag_start_status[index] = (uint8_t)HAL_FDCAN_Start(hfdcan);
			can_diag_hal_error_code[index] = hfdcan->ErrorCode;
            system_can[1]=1;
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        CAN3_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[2]==0){

			can_diag_notification_status[index] = (uint8_t)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			can_diag_start_status[index] = (uint8_t)HAL_FDCAN_Start(hfdcan);
			can_diag_hal_error_code[index] = hfdcan->ErrorCode;
            system_can[2]=1;
        }
    }
}

/**
 * @brief CAN的标准帧发送
 * @param hfdcan  can编号
 * @param id CAN_id(标准帧)
 * @param *data 发送数据数组指针
 * @param len 数组长度
 * @return void
 */
uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
	uint8_t index = can_index(hfdcan);
	FDCAN_TxHeaderTypeDef fdcan_tx_header;
    fdcan_tx_header.Identifier=id;
	fdcan_tx_header.IdType = FDCAN_STANDARD_ID;

    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = len;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0;
    can_diag_tx_attempt_count[index]++;
    record_bus_state(hfdcan);
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, data)!=HAL_OK) {
        can_diag_tx_error_count[index]++;
        can_diag_hal_error_code[index] = hfdcan->ErrorCode;
        return 1;
    }
    can_diag_tx_ok_count[index]++;
    record_bus_state(hfdcan);
    return 0;
}

/**
 * @brief CAN的扩展帧发送
 * @param hfdcan  can编号
 * @param id CAN_id(扩展帧)
 * @param *data 发送数据数组指针
 * @param len 数组长度
 * @return void
 */
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
	uint8_t index = can_index(hfdcan);
	FDCAN_TxHeaderTypeDef fdcan_tx_header;
    fdcan_tx_header.Identifier=id;
	fdcan_tx_header.IdType = FDCAN_EXTENDED_ID;

    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = len;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0;
    can_diag_tx_attempt_count[index]++;
    record_bus_state(hfdcan);
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, data)!=HAL_OK) {
        can_diag_tx_error_count[index]++;
        can_diag_hal_error_code[index] = hfdcan->ErrorCode;
        return 1;
	}
    can_diag_tx_ok_count[index]++;
    record_bus_state(hfdcan);
	return 0;
}

/**
 * @brief CAN的接收回调函数
 * @param hfdcan  can编号
 * @param *data 接收数据数组指针
 * @return void
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t index = can_index(hfdcan);
	can_diag_rx_irq_count[index]++;
	record_bus_state(hfdcan);

    if (hfdcan->Instance == FDCAN1)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer) == HAL_OK)
        {
            record_rx(0U, &CAN1_Manage_Object.Rx_Header);

            if (CAN1_Manage_Object.Callback_Function != nullptr)
            {
                CAN1_Manage_Object.Callback_Function(CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer) == HAL_OK)
        {            
            record_rx(1U, &CAN2_Manage_Object.Rx_Header);

            if (CAN2_Manage_Object.Callback_Function != nullptr)
            {
                CAN2_Manage_Object.Callback_Function(CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer) == HAL_OK)
        {
            record_rx(2U, &CAN3_Manage_Object.Rx_Header);
            

            if (CAN3_Manage_Object.Callback_Function != nullptr)
            {
                CAN3_Manage_Object.Callback_Function(CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer);
            }
        }
    }

}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t index = can_index(hfdcan);
	can_diag_rx_irq_count[index]++;
	record_bus_state(hfdcan);

	if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer) == HAL_OK)
        {            
            record_rx(1U, &CAN2_Manage_Object.Rx_Header);

            if (CAN2_Manage_Object.Callback_Function != nullptr)
            {
                CAN2_Manage_Object.Callback_Function(CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer);
            }
        }
    }


}


