#ifndef __LTDCTEST_H
#define __LTDCTEST_H

#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdint.h"
#include "lcd.h"

void ltdc_test(void);
void ltdc_test_565(void);
void bitmap_test(void);
uint32_t OpenBMP(uint8_t *ptr, const char* fname);

#endif
