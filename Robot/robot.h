#ifndef ROBOT_H
#define ROBOT_H

#include "stm32f1xx.h"
#include "step_motor.h"

typedef enum
{
	ROBOT_STOP,
	ROBOT_RUN,
	INVALID
} Robot_State;

typedef struct
{
	StepperMotor *motorLeft;
	StepperMotor *motorRight;
	float v_left;
	float v_right;
	Robot_State state;
} Robot_Typedef;

void robot_init(Robot_Typedef *robot, StepperMotor *_motorLeft, StepperMotor *_motorRight);
void robot_control(Robot_Typedef *robot, float speed_left, float speed_right);
void Calculate_Velocity(Robot_Typedef *robot);

#endif
