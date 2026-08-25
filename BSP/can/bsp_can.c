//
// Created by asus on 2025/4/28.
//

#include "bsp_can.h"
#include "can.h"

#include "bsp_def.h"

static uint8_t tot;
static CAN_HandleTypeDef *handle;

static uint32_t rx_id[BSP_CAN_FILTER_LIMIT];
static void (*callback[BSP_CAN_FILTER_LIMIT]) (bsp_can_msg_t *msg);

void bsp_can_init(void) {
    handle = &hcan1;
    HAL_CAN_Start(handle);
    // HAL_CAN_ActivateNotification(h, CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO1_MSG_PENDING);
}
uint8_t bsp_can_set_callback(uint32_t id, void (*f) (bsp_can_msg_t *msg)) {
    BSP_ASSERT(tot < BSP_CAN_FILTER_LIMIT && f != NULL);
    rx_id[tot] = id;
    callback[tot] = f;

    CAN_FilterTypeDef filter = {
        .FilterMode = CAN_FILTERMODE_IDLIST,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterFIFOAssignment = (id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
        .FilterIdLow = id << 5,
        .FilterBank = tot,
        .FilterActivation = CAN_FILTER_ENABLE
    };
    BSP_ASSERT(HAL_CAN_ConfigFilter(handle, &filter) == HAL_OK);
    return tot ++;
}

void bsp_can_send(uint32_t id, uint8_t *s) {
    BSP_ASSERT(handle);
    CAN_TxHeaderTypeDef header = {
        .StdId = id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08
    };
    while(HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0) __NOP(); // 等待邮箱空闲
    uint32_t tx_mailbox = 0;
    HAL_CAN_AddTxMessage(handle, &header, s, &tx_mailbox);
}

void bsp_can_rx_sol(uint32_t fifo) {
    bsp_can_msg_t msg = { 0 };
    while(HAL_CAN_GetRxFifoFillLevel(handle, fifo)) {
        HAL_CAN_GetRxMessage(handle, fifo, &msg.header, msg.data);
        for(uint8_t i = 0; i < tot; i++) {
            if(rx_id[i] == msg.header.StdId) {
                BSP_ASSERT(callback[i] != NULL);
                callback[i](&msg);
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
    bsp_can_rx_sol(CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) {
    bsp_can_rx_sol(CAN_RX_FIFO1);
}


