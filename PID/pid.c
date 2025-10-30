#include "pid.h"
#include "main.h"
#include "math.h"

float pwm_left, pwm_right;

typedef struct{
	float y_prev;
}Filter;
Filter filter_left = {0};
Filter filter_right = {0};

static float Cal_Filter(Filter *filter, float value, float alpha){
	float y = alpha * value + (1.0f - alpha) * filter->y_prev;
	filter->y_prev = y;
	return y;
}

void PID_Init(PID_Typedef* pid, float _kp, float _ki, float _kd){
	pid->kp = _kp;
	pid->ki = _ki;
	pid->kd = _kd;
}

static float PID_Calculate(PID_Typedef* pid, float _setpoint, float _actual)
{
    pid->setpoint = _setpoint;
	pid->actual = _actual;
	pid->error = _setpoint - _actual;
	
	pid->intergal += pid->error * SAMPLE_TIME;
	pid->derivative = (pid->error - pid->last_error) / SAMPLE_TIME;
	pid->output = pid->kp * pid->error + pid->ki * pid->intergal + pid->kd * pid->derivative;
	
	if(pid->output > 1) pid->output = 1;
	if(pid->output < 0) pid->output = 0;
	
	pid->output_prev = pid->output;
	pid->last_error = pid->error;
	
	return pid->output;
}

static void PID_Reset(PID_Typedef* pid){
	pid->actual = 0;
	pid->derivative = 0;
	pid->error = 0;
	pid->intergal = 0;
	pid->last_error = 0;
	pid->output = 0;
}

void Calculate_Velocity(Robot_Typedef* robot, PID_Typedef* pid_left, PID_Typedef* pid_right, const float counterEncoder_Left, const float counterEncoder_Right, const float setpoint_left, const float setpoint_right){	
    // Tính tốc độ tuyến tính (m/s)
    robot->v_left = (counterEncoder_Left * 2 * PI * WHEEL_RADIUS) / (PPR * SAMPLE_TIME * RATIO);
    robot->v_right = (counterEncoder_Right * 2 * PI * WHEEL_RADIUS) / (PPR * SAMPLE_TIME * RATIO);
	if(setpoint_left == 0){
		PID_Reset(pid_left);
		pwm_left = 0;
	}else{
		pwm_left = Cal_Filter(&filter_left, PID_Calculate(pid_left, setpoint_left, robot->v_left), 0.4);
		//pwm_left = PID_Calculate(pid_left, setpoint_left, robot->v_left);
	}
	
	if(setpoint_right == 0){
		PID_Reset(pid_right);
		pwm_right = 0;
	}else{
		pwm_right = Cal_Filter(&filter_right, PID_Calculate(pid_right, setpoint_right, robot->v_right), 0.4);
		//pwm_right = PID_Calculate(pid_right, setpoint_right, robot->v_right);
	}
	if(robot->motor_left->direct == BACKWARD) robot->v_left *= (-1.0f);
	if(robot->motor_right->direct == FORWARD) robot->v_right *= (-1.0f);
		
	if(pwm_left > 1) pwm_left = 1;
	if(pwm_right > 1) pwm_right = 1;

	robot_control(robot, pwm_left, pwm_right);
}
