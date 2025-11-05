#include "robot.h"
#include "main.h"
#include "math.h"

void robot_init(Robot_Typedef *robot, Servo_TypeDef** servo_list, StepperMotor *_motorLeft, StepperMotor *_motorRight)
{
	robot->servo_list = servo_list;
	robot->motorLeft = _motorLeft;
	robot->motorRight = _motorRight;
	robot->v_left = 0.0f;
	robot->v_right = 0.0f;
	robot->state = ROBOT_STOP;
	robot_control(robot, 0, 0);
}

void robot_control(Robot_Typedef *robot, float speed_left, float speed_right)
{
	if (robot->state == ROBOT_RUN)
	{
		Stepper_Setup(robot->motorLeft, speed_left);
		Stepper_Setup(robot->motorRight, speed_right);
	}
	else if (robot->state == ROBOT_STOP)
	{
		Stepper_Setup(robot->motorLeft, 0);
		Stepper_Setup(robot->motorRight, 0);
	}
}

void Calculate_Velocity(Robot_Typedef *robot)
{
	robot->v_left = ((robot->motorLeft->direction == BACKWARD) ? (AS5600_CalVelocity(robot->motorLeft->encoder) * (-1.0f)) : AS5600_CalVelocity(robot->motorLeft->encoder)) / RATIO_MOTOR;
	robot->v_right = ((robot->motorRight->direction == FORWARD) ? (AS5600_CalVelocity(robot->motorRight->encoder) * (-1.0f)) : AS5600_CalVelocity(robot->motorRight->encoder)) / RATIO_MOTOR;
}
