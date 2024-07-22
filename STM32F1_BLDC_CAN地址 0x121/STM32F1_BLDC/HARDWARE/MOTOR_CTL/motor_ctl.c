#include "motor_ctl.h"
#include "delay.h"
extern uint8_t hall_state;
extern uint8_t function_code;
extern uint8_t motor_mode; // 点动，连续，停止

MotorDirection motor_direction = MOTOR_FORWARD;

void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 初始化PWM引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置TIM1
    TIM_TimeBaseStructure.TIM_Period = 1799; // 10kHz PWM frequency for 72MHz system clock
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 配置PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 899; // 50% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Automatic output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0x0F;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void Motor_SetSpeed(uint16_t speed)
{
    // uint16_t pulse = (15000 - 1) * speed / 100; // 根据传入的speed设置占空比
    TIM_SetCompare1(TIM1, speed);
    TIM_SetCompare2(TIM1, speed);
    TIM_SetCompare3(TIM1, speed);
}

void Motor_SetDirection(MotorDirection direction)
{
    motor_direction = direction;
}

void Motor_Commutate(uint8_t hall_state)
{
    if (motor_direction == MOTOR_FORWARD)
    {
        switch (hall_state)
        {
        case 0x5: // Hall 110
            // 通道1：U+
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

            // 通道2：V-
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
            break;

        case 0x1: // Hall 010
            // 通道1：U+
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

            // 通道3：W-
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
            break;

        case 0x3: // Hall 011
            // 通道2：V+
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

            // 通道3：W-
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
            break;

        case 0x2: // Hall 001
            // 通道2：V+
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

            // 通道1：U-
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
            break;

        case 0x6: // Hall 101
            // 通道3：W+
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

            // 通道1：U-
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
            break;

        case 0x4: // Hall 100
            // 通道3：W+
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

            // 通道2：V-
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
            break;

        default: // 无效的霍尔状态，禁用所有输出
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
            break;
        }
    }
    if (motor_direction == MOTOR_BACKWARD)
    {
        switch (hall_state)
        {
        case 0x5: // Hall 110
            // 通道1：U+
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

            // 通道2：V-
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
            break;

        case 0x1: // Hall 010
            // 通道1：U+
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

            // 通道3：W-
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
            break;

        case 0x3: // Hall 011
            // 通道2：V+
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

            // 通道3：W-
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
            break;

        case 0x2: // Hall 001
            // 通道2：V+
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

            // 通道1：U-
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
            break;

        case 0x6: // Hall 101
            // 通道3：W+
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

            // 通道1：U-
            TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
            break;

        case 0x4: // Hall 100
            // 通道3：W+
            TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

            // 通道2：V-
            TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
            break;

        default: // 无效的霍尔状态，禁用所有输出
            TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
            TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
            TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////

// 前进，后退，左转右转函数

// 电机速度通过通信协议来设置占空比
// 下面函数为连续运动模式函数
void continuous(unsigned int motor_dir)
{

    switch (motor_dir) // 设置的电机旋转方向
    {

    case 0:
        Motor_SetDirection(MOTOR_BACKWARD); // 设置旋转方向
        Motor_Commutate(hall_state);
        break;
    case 1:
        Motor_SetDirection(MOTOR_FORWARD); // 设置旋转方向
        Motor_Commutate(hall_state);
        break;
//    default:
//        GPIO_ResetBits(GPIOB, GPIO_Pin_12); // 置0 默认情况关闭电机
//        break;
    }
}
// 下面函数为停止模式函数.就是把电机EN关闭
void motor_stop(void)
{

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

// 下面函数为停止模式函数.就是把电机EN关闭
void motor_start(void)
{

    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}


// 下面函数为点动模式函数,延时1S后关闭电机，从而实现点动。。。
void point_mode(unsigned int motor_dir)
{

    switch (motor_dir) // 设置的电机旋转方向
    {

    case 0:
        Motor_SetDirection(MOTOR_BACKWARD); // 设置旋转方向
        Motor_Commutate(hall_state);
        delay_ms(500);
        motor_stop();
        break;
    case 1:
        Motor_SetDirection(MOTOR_FORWARD); // 设置旋转方向
        Motor_Commutate(hall_state);
        delay_ms(500);
        motor_stop();
        break;
//    default:
//        motor_stop(); // 默认情况关闭电机
//        break;
    }
//    function_code = 0x00; // 清除数据
}

// 电机功能旋转
void Motor_ctl(void)
{
    if (motor_mode == 0x02)
    {
        motor_start();
        switch (function_code)
        {
        case 0x01:         //
            continuous(0); // 旋转方向根据具体情况定义
            break;
        case 0x02:         //
            continuous(1); //
            break;
        default:
        motor_stop(); // 默认情况关闭电机
//        break;

        }
    }

    if (motor_mode == 0x01)
    {
        motor_start();
        switch (function_code)
        {
        case 0x01:         //
            point_mode(0); //
            break;
        case 0x02:         //
            point_mode(1); //
            break;
        default:
        motor_stop(); // 默认情况关闭电机

        }
    }
}
