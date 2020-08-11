#include "string.h"
#include "oled.h"
#include "i2c.h"
#include "u8g2.h"
#include "imu.h"
#include "sensorsDist.h"
#include "sensorsFloor.h"
#include "stdio.h"

typedef struct {
  long heading;
  uint16_t LMotorSpeed;
  uint16_t RMotorSpeed;
  uint8_t irFloor;
  sensorDistData irDist;
  uint16_t irDistData[5];
  int8_t direction;
  uint8_t irCommand;
}miniStatus;

static u8g2_t u8g2;
uint8_t *buffer;
uint8_t *bufferDMA;


void oled(void const * argument){
  UNUSED(argument);
  extern volatile miniStatus status;
  uint8_t *buf = (uint8_t*)pvPortMalloc(512);
  buffer = (uint8_t*)pvPortMalloc(32);
  bufferDMA = (uint8_t*)pvPortMalloc(32);
  u8g2_Setup_ssd1306_i2c_128x32_univision_f(&u8g2, U8G2_R2, u8x8_byte_stm32_hw_i2c, u8x8_stm32_gpio_and_delay);
  u8g2_SetBufferPtr(&u8g2, buf);
  u8g2_SetI2CAddress(&u8g2, 0x3c);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_ClearDisplay(&u8g2);
  char msg[20];
  uint8_t i = 0;
  uint32_t res;
  for(;;){
    u8g2_FirstPage(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_t0_11_tr);
    do
      {
	vTaskDelay(pdMS_TO_TICKS(35));
	if(status.heading < 0){
	  extern long imuHeadingInternal;
	  u8g2_DrawFrame(&u8g2, 70, 0, 35, 8);
	  sprintf(msg, "%ld", imuHeadingInternal);
	  u8g2_DrawStr(&u8g2, 107, 8, msg);
	  switch(status.heading){
	  case -3:
	    u8g2_DrawBox(&u8g2, 70, 0, 12, 8);
	    break;
	  case -2:
	    u8g2_DrawBox(&u8g2, 70, 0, 24, 8);
	    break;
	  case -1:
	    u8g2_DrawBox(&u8g2, 70, 0, 35, 8);
	    break;
	  }
	}else{
	  sprintf(msg, "dir: %ld", status.heading);
	  u8g2_DrawStr(&u8g2, 70, 8, msg);
	}

	res = map(status.irDistData[LIPOS], 0, 4095, 0, 32);
	u8g2_DrawFrame(&u8g2, 1, res, 12, 32-res);

	res = map(status.irDistData[FDPOS], 0, 4095, 0, 32);
	u8g2_DrawFrame(&u8g2, 14, res, 12, 32-res);

	res = map(status.irDistData[FCPOS], 0, 4095, 0, 32);
	u8g2_DrawFrame(&u8g2, 28, res, 12, 32-res);

	res = map(status.irDistData[FIPOS], 0, 4095, 0, 32);
	u8g2_DrawFrame(&u8g2, 42, res, 12, 32-res);

	res = map(status.irDistData[LDPOS], 0, 4095, 0, 32);
	u8g2_DrawFrame(&u8g2, 56, res, 12, 32-res);
	i = (i+1) % 28;


	u8g2_DrawFrame(&u8g2, 75, 10, 25, 22);

	if(status.irFloor & IRPFI){
	  u8g2_DrawBox(&u8g2, 77, 12, 5, 5);
	}else{
	  u8g2_DrawFrame(&u8g2, 77, 12, 5, 5);
	}

	if(status.irFloor & IRPFD){
	  u8g2_DrawBox(&u8g2, 93, 12, 5, 5);
	}else{
	  u8g2_DrawFrame(&u8g2, 93, 12, 5, 5);
	}

	if(status.irFloor & IRPAI){
	  u8g2_DrawBox(&u8g2, 77, 25, 5, 5);
	}else{
	  u8g2_DrawFrame(&u8g2, 77, 25, 5, 5);
	}

	if(status.irFloor & IRPAC){
	  u8g2_DrawBox(&u8g2, 93, 25, 5, 5);
	}else{
	  u8g2_DrawFrame(&u8g2, 93, 25, 5, 5);
	}

	if(status.irFloor & IRPAD){
	  u8g2_DrawBox(&u8g2, 85, 25, 5, 5);
	}else{
	  u8g2_DrawFrame(&u8g2, 85, 25, 5, 5);
	}


      } while (u8g2_NextPage(&u8g2));
    // vTaskDelay(40);
  }
}



void HAL_I2C_ISR(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(i2cSemHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_ISR(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_ISR(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_ISR(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_ISR(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  I2C_ClearBusyFlagErratum();
  UNUSED(hi2c);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(pdPASS == (xSemaphoreGiveFromISR(i2cSemHandle,&xHigherPriorityTaskWoken))){
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
				  U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
				  U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      vTaskDelay(pdMS_TO_TICKS(1));
      break;
    case U8X8_MSG_DELAY_MILLI:
      vTaskDelay(pdMS_TO_TICKS(arg_int));
      break;
    case U8X8_MSG_GPIO_DC:
      break;
    case U8X8_MSG_GPIO_RESET:
      break;
    }
  return 1;
}


uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  //  static uint8_t buffer[32];  /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;
  uint8_t res = 1;

  switch(msg)
    {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
	{
	  buffer[buf_idx++] = *data;
	  data++;
	  arg_int--;
	}
      break;
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      xSemaphoreTake(i2cSemHandle, portMAX_DELAY);
      memcpy(bufferDMA, buffer, 32);
      res = HAL_I2C_Master_Transmit_IT(&hi2c1, u8x8_GetI2CAddress(u8x8) << 1, bufferDMA, buf_idx);
      if(res == HAL_ERROR || res == HAL_BUSY){
	taskENTER_CRITICAL();
	I2C_ClearBusyFlagErratum();
	taskEXIT_CRITICAL();
      }
      res = (res == HAL_OK) ? 1 : 0;
      break;
    default:
      return 0;
    }
  return res;
}
