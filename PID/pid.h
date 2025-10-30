#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "robot.h"

#define PPR 220           
#define WHEEL_RADIUS 0.029f
#define L 0.12f
#define PI 3.14159265359f
#define RATIO 56

#define KP 6.0f
#define KI 0.5f
#define KD 0.2f
#define SAMPLE_TIME 0.01f

typedef struct{
	float setpoint;
	float actual;
	float error;
	float last_error;
	float intergal;
	float derivative;
	float output;
	float output_prev;
	float kp, ki, kd;
} PID_Typedef;

void PID_Init(PID_Typedef* pid, float kp, float ki, float kd);
void Calculate_Velocity(Robot_Typedef* robot, PID_Typedef* pid_left, PID_Typedef* pid_right, const float counterEncoder_Left, const float counterEncoder_Right, const float setpoint_left, const float setpoint_right);

#endif
