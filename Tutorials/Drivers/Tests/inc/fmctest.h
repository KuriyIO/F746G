#ifndef __FMCTEST_H
#define __FMCTEST_H

#include "stm32f7xx_hal.h"
#include "MT48LC4M32B2.h"
#include "string.h"
#include "stdint.h"

#define TEST_OK (1);
#define TEST_FAIL (0);

#define BUFFER_SIZE         ((uint32_t)0x0100)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)

void fmc_test(UART_HandleTypeDef *huart);

#endif
