#include "hall.h"

void Hall_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Hall_Read(void)
{
    uint8_t hall_state = 0;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
        hall_state |= 0x01;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
        hall_state |= 0x02;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
        hall_state |= 0x04;
    return hall_state;
}
