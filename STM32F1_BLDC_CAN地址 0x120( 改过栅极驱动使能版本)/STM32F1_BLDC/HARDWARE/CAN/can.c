#include "can.h"
#include "motor_ctl.h"

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

uint8_t motor_mode;
uint8_t function_code;

void CAN_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ʹ��GPIO��CANʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* ����CAN_RX (PA11)���� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ����CAN_TX (PA12)���� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��λCAN */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* ��ʼ��CAN */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* ����CAN�˲��� */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x120 << 5; //0x120 (0001 0010 0000)
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0; //��������
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* ����NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* ʹ��CAN FIFO 0��Ϣ�����ж� */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

uint8_t CAN_TransmitMessage(uint32_t StdId, uint8_t *msg, uint8_t len)
{
    int i = 0;
    CanTxMsg TxMessage;
    uint8_t mailbox;

    if (len > 8)
        return 0; // CAN���ݳ������Ϊ8�ֽ�

    TxMessage.StdId = StdId;
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = len;
    for (i = 0; i < len; i++)
    {
        TxMessage.Data[i] = msg[i];
    }

    mailbox = CAN_Transmit(CAN1, &TxMessage);

    return mailbox != CAN_TxStatus_NoMailBox;
}

// CAN �����жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    int i;
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) // Check if message pending
    {
        CanRxMsg RxMessage;
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); // Read the message from the FIFO

        if (rx_index < RX_BUFFER_SIZE)
        {
            for (i = 0; i < RxMessage.DLC; i++)
            {                                              // DLC is Data Length Code
                rx_buffer[rx_index++] = RxMessage.Data[i]; // Store received data in buffer
            }
        }

        // Check for end of frame and process data
        if (rx_buffer[4] == 0x55)
        { // Assuming the end of the frame identifier is at index 4

            // Process the received data
            motor_mode = rx_buffer[1];    // �綯�������˶�ģʽ
            function_code = rx_buffer[2]; // ������ 01 ǰ�� ��02���ˣ�03��ת��04��ת

            // Clear receive buffer after processing
            rx_index = 0;
        }
        else
        {
            rx_index = 0;
        }
    }

    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0); // Clear pending bit
}
