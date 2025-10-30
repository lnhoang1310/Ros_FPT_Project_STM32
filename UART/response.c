#include "response.h"
#include "robot.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

int8_t speed_left = 0;
int8_t speed_right = 0;


static int8_t get_speed(char *data, uint8_t* flag)
{
	*flag = 0;
	uint8_t count_minus = 0;
	int8_t speed = 0;
	uint8_t len = strlen(data);
	for (uint8_t k = 0; k < len; k++){
		if (data[k] == '-'){
			if (count_minus >= 2 || (k > 0 && data[k - 1] == '-')){
				return -1;
			}
			*flag = 1;
			continue;
		}
		speed = speed * 10 + (data[k] - '0');
	}
	
	return speed;
}

static void get_info_cmd(char **argv, uint8_t* flag_left, uint8_t* flag_right){
	speed_left = get_speed(argv[0], flag_left);
	speed_right = get_speed(argv[1], flag_right);
}

uint8_t response_uart(char **argv, uint8_t* flag_left, uint8_t* flag_right, uint8_t* state){
	get_info_cmd(argv, flag_left, flag_right);
	if(speed_left == 0 && speed_right == 0) *state = 0;
	else *state = 1;
	if (speed_left > 100 || speed_right > 100 || speed_left < 0 || speed_right < 0)
		return 0;
	return 1;
}
