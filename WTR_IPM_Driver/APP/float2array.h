
#ifndef __FLOAT2ARRAY
#define __FLOAT2ARRAY

#include "stm32g4xx.h"

void float_2_array(float value, uint8_t *byte_array);
float array_2_float(const uint8_t *byte_array);

#endif