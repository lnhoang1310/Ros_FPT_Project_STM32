#ifndef RESPONSE_H
#define RESPONSE_H

#include "stm32f1xx.h"

extern int8_t speed_left;
extern int8_t speed_right;
uint8_t response_uart(char **argv, uint8_t* flag_left, uint8_t* flag_right, uint8_t* state);

#endif
