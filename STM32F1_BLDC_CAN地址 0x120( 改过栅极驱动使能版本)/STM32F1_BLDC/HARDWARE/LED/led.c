#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK miniSTM32开发板
// LED驱动代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2012/9/2
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// 初始化PB5和PE5为输出口.并使能这两个口的时钟
// LED IO初始化
void LED_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_12; // LED0-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 // IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);								 // 根据设定参数初始化GPIOA.8
	GPIO_SetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_12);			 // PA.8 输出高
}

// 闪烁函数里面初始化了栅极驱动器使能，变成打开使能状态
void LED_Blink(void)
{
	// GPIO_SetBits(GPIOB,GPIO_Pin_5);
	// GPIO_SetBits(GPIOB,GPIO_Pin_6);
	// delay_ms(200);
	// GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // 栅极驱动器使能脚
	GPIO_ResetBits(GPIOB, GPIO_Pin_6); // LED1 PB6 LED2 PB5
	// delay_ms(200);
}
