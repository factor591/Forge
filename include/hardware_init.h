#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef init_hardware(void);

void system_init(void);
uint32_t get_microseconds(void);
uint32_t get_milliseconds(void);
void delay_ms(uint32_t ms);
void Error_Handler(void);

#endif /* HARDWARE_INIT_H */
