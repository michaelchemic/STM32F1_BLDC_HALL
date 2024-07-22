// V1.0 ����ȱ�ݣ������޴�
// V1.1 �����ϵ�ѹ���綯��̫��
// CAN��ַ 0x120

#include "stm32f10x.h"
#include "hall.h"
#include "motor_ctl.h"
#include "led.h"
#include "sys.h"
#include "delay.h"
#include "can.h"
#include "stm32f10x.h"

extern uint8_t motor_mode;    // �㶯�����������ֹ���ģʽ
extern uint8_t function_code; // ǰ�������˶�������

void can_transmit_test(void); // ���Ͳ��Ժ�������
void can_receive_test(void);  // ���ղ��Ժ�������

// can���Ͳ��� ������ 800kbps
volatile int i    = 0;
uint8_t tx_msg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // ����data����
uint8_t rx_msg[8];                                                    // ��������
uint8_t len;                                                          // ����
uint32_t id;                                                          // ֡ID

uint8_t hall_state; // ��������

int main(void)
{

    SystemInit(); // ϵͳ��ʼ��
    CAN_Config(); // ��ʼ��CAN
    Hall_Init();  // hall��������ʼ��
    Motor_Init(); // �����ʼ��
    LED_Init();
    LED_Blink();

    //  ͨ������Motor_SetSpeed��������Ť�أ�MOS�ܵ�ͨʱ�䣩
    //  Motor_SetSpeed(50);
    //  Motor_SetSpeed(1200);
    //Motor_SetSpeed(1000);
    //  Motor_SetDirection(MOTOR_BACKWARD);//������ת����

    //  can_transmit_test(); //can���Ͳ���

    while (1) {

        hall_state = Hall_Read(); // ��ȡ��ǰ��������
        Motor_ctl();              // �������Э�����ƺ���

        // ���Ե��
        // uint8_t hall_state = Hall_Read();//��ȡ��ǰ��������
        // Motor_Commutate(hall_state);
    }
}

// ��������can���Ժ���
void can_transmit_test(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x123, tx_msg, 8);

    /* �����ӳ� */
    delay_ms(2000);
}


