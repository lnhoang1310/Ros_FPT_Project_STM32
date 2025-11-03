#include "response.h"
#include "robot.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

static int8_t get_speed(char *data)
{
	uint8_t flag = 0;
	uint8_t count_minus = 0;
	int8_t speed = 0;
	uint8_t len = strlen(data);
	for (uint8_t k = 0; k < len; k++){
		if (data[k] == '-'){
			if (count_minus >= 2 || (k > 0 && data[k - 1] == '-')){
				return -1;
			}
			flag = 1;
			continue;
		}
		speed = speed * 10 + (data[k] - '0');
	}
	if(flag) speed *= (-1.0f);
	return speed;
}

static void get_info_cmd(char **argv, int8_t* speed_left, int8_t* speed_right){
	*speed_left = get_speed(argv[0]);
	*speed_right = get_speed(argv[1]);
}

uint8_t response_uart(char **argv, int8_t* speed_left, int8_t* speed_right, uint8_t* state){
	get_info_cmd(argv, speed_left, speed_right);
	//if(*speed_left == speed_left_prev && *speed_right == speed_right_prev) return 0;
	if(*speed_left == 0 && *speed_right == 0) *state = 0;
	else *state = 1;
	if (*speed_left > 100 || *speed_right > 100 || *speed_left < -100 || *speed_right < -100)
		return 0;
	return 1;
}
