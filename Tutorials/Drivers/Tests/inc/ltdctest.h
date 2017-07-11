#ifndef __LTDCTEST_H
#define __LTDCTEST_H

#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdint.h"
#include "lcd.h"

void ltdc_test(LTDC_HandleTypeDef *hltdc, volatile uint32_t *LTDC_MEM, RNG_HandleTypeDef *hrng);

#endif
