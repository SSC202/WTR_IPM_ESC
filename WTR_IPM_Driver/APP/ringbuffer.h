#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include "stm32g4xx.h"

uint8_t command_get_command(uint8_t *command);
uint8_t command_write(uint8_t *data, uint8_t length);

#endif // __RINGBUFFER_H