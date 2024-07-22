#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"

#define RX_BUFFER_SIZE 5

/* CAN初始化配置 */
void CAN_Config(void);

/* CAN发送数据 */
uint8_t CAN_TransmitMessage(uint32_t StdId, uint8_t *msg, uint8_t len);

/* CAN接收数据 */
uint8_t CAN_ReceiveMessage(uint32_t *StdId, uint8_t *msg, uint8_t *len);

/* 中断服务函数 */
void USB_LP_CAN1_RX0_IRQHandler(void);

#endif /* __CAN_H */
