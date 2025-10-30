#include "robot.h"
#include "main.h"
#include "math.h"

void robot_init(Robot_Typedef *robot, StepperMotor *_motorLeft, StepperMotor *_motorRight)
{
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
		Direct_State direct_left = (speed_left < 0) ? BACKWARD : FORWARD;
		Direct_State direct_right = (speed_right < 0) ? FORWARD : BACKWARD;
		Stepper_Setup(robot->motorLeft, direct_left, fabs(speed_left));
		Stepper_Setup(robot->motorRight, direct_right, fabs(speed_right));
	}
	else if (robot->state == ROBOT_STOP)
	{
		Stepper_Setup(robot->motorLeft, robot->motorLeft->direction, 0);
		Stepper_Setup(robot->motorRight, robot->motorRight->direction, 0);
	}
}

void Calculate_Velocity(Robot_Typedef *robot)
{
	robot->v_left = ((robot->motorLeft->direction == BACKWARD) ? (AS5600_CalVelocity(robot->motorLeft->encoder) * (-1.0f)) : AS5600_CalVelocity(robot->motorLeft->encoder)) / RATIO_MOTOR;
	robot->v_right = ((robot->motorRight->direction == FORWARD) ? (AS5600_CalVelocity(robot->motorRight->encoder) * (-1.0f)) : AS5600_CalVelocity(robot->motorRight->encoder)) / RATIO_MOTOR;
}
