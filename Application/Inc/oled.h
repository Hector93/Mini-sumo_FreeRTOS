/**
******************************************************************************
* File Name          : oled.h
* Description        : function definitions for the oled display
******************************************************************************
**/

#ifndef __OLED_H
#define __OLED_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx.h"
#include "Ids.h"
#include "u8g2.h"
#include "misc.h"
extern  osThreadId oledDisHandle;
extern osSemaphoreId i2cSemHandle;

void HAL_I2C_ISR(I2C_HandleTypeDef *hi2c);
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
#endif
