
#include "float2array.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void float_2_array(float value, uint8_t *byte_array)
{
    uint32_t int_value;
    memcpy(&int_value, &value, sizeof(float));

    byte_array[0] = (int_value >> 24) & 0xFF;
    byte_array[1] = (int_value >> 16) & 0xFF;
    byte_array[2] = (int_value >> 8) & 0xFF;
    byte_array[3] = int_value & 0xFF;
}

float array_2_float(const uint8_t *byte_array)
{
    uint32_t int_value = 0;
    float result;

    int_value |= ((uint32_t)byte_array[0] << 24);
    int_value |= ((uint32_t)byte_array[1] << 16);
    int_value |= ((uint32_t)byte_array[2] << 8);
    int_value |= (uint32_t)byte_array[3];

    memcpy(&result, &int_value, sizeof(float));
    return result;
}