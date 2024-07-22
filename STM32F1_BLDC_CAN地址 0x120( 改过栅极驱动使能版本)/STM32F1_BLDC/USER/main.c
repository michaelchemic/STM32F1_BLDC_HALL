// V1.0 致命缺陷，噪音巨大。
// V1.1 相线上电压反电动势太大。
// CAN地址 0x120

#include "stm32f10x.h"
#include "hall.h"
#include "motor_ctl.h"
#include "led.h"
#include "sys.h"
#include "delay.h"
#include "can.h"
#include "stm32f10x.h"

extern uint8_t motor_mode;    // 点动，连续，两种工作模式
extern uint8_t function_code; // 前后左右运动功能码

void can_transmit_test(void); // 发送测试函数声明
void can_receive_test(void);  // 接收测试函数声明

// can发送测试 波特率 800kbps
volatile int i    = 0;
uint8_t tx_msg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // 发送data数据
uint8_t rx_msg[8];                                                    // 接收数组
uint8_t len;                                                          // 长度
uint32_t id;                                                          // 帧ID

uint8_t hall_state; // 霍尔编码

int main(void)
{

    SystemInit(); // 系统初始化
    CAN_Config(); // 初始化CAN
    Hall_Init();  // hall编码器初始化
    Motor_Init(); // 电机初始化
    LED_Init();
    LED_Blink();

    //  通过下列Motor_SetSpeed函数设置扭矩（MOS管导通时间）
    //  Motor_SetSpeed(50);
    //  Motor_SetSpeed(1200);
    //Motor_SetSpeed(1000);
    //  Motor_SetDirection(MOTOR_BACKWARD);//设置旋转方向

    //  can_transmit_test(); //can发送测试

    while (1) {

        hall_state = Hall_Read(); // 读取当前霍尔扇区
        Motor_ctl();              // 电机解析协议后控制函数

        // 测试电机
        // uint8_t hall_state = Hall_Read();//读取当前霍尔扇区
        // Motor_Commutate(hall_state);
    }
}

// 下面两个can测试函数
void can_transmit_test(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x123, tx_msg, 8);

    /* 发送延迟 */
    delay_ms(2000);
}


