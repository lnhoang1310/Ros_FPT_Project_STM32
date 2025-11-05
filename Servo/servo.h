#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx.h"
#define MIN_PULSE_WIDTH 400
#define MAX_PULSE_WIDTH 2500
#define SERVO_NUMER 2

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	uint8_t Angle;
}Servo_TypeDef;

extern Servo_TypeDef* servo_list[SERVO_NUMER];

void Servo_Set(Servo_TypeDef* servo, uint8_t angle);
void Servo_Init(Servo_TypeDef *servo, TIM_HandleTypeDef* htim, uint32_t channel);
#endif
