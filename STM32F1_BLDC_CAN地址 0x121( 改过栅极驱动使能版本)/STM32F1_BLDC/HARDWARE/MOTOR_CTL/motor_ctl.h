#ifndef __MOTOR_CTL_H
#define __MOTOR_CTL_H

#include "stm32f10x.h"

typedef enum
{
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDirection;

void GPIO_TIM1_Init(void);

void Motor_Init(void);
void Motor_SetSpeed(uint16_t speed);
void Motor_SetDirection(MotorDirection direction);
void Motor_Commutate(uint8_t hall_state);

void point_mode(unsigned int motor_dir);
void motor_stop(void);
void motor_start(void);
void continuous(unsigned int motor_dir);
void Motor_ctl(void);

#endif // __MOTOR_CTL_H
