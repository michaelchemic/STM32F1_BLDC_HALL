#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"

#define RX_BUFFER_SIZE 5

/* CAN��ʼ������ */
void CAN_Config(void);

/* CAN�������� */
uint8_t CAN_TransmitMessage(uint32_t StdId, uint8_t *msg, uint8_t len);

/* CAN�������� */
uint8_t CAN_ReceiveMessage(uint32_t *StdId, uint8_t *msg, uint8_t *len);

/* �жϷ����� */
void USB_LP_CAN1_RX0_IRQHandler(void);

#endif /* __CAN_H */
