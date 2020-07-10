#include "oled.h"
//#include "ssd1306.h"
#include "i2c.h"
#include "u8g2.h"
#include "imu.h"
#include "sensorsDist.h"
#include "stdio.h"
//uint16_t* fonts6x8;
/* FontDef Font_6x8; */
/* uint8_t* SSD1306_Buffer; */

typedef struct {
  long heading;
  uint16_t LMotorSpeed;
  uint16_t RMotorSpeed;
  uint8_t irFloor;
  sensorDistData irDist;
  int8_t direction;
  uint8_t irCommand;
}miniStatus;

static u8g2_t u8g2;

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
				  U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
				  U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      HAL_Delay(1);
      break;
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(arg_int);
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
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
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
      //i2c_transfer(u8x8_GetI2CAddress(u8x8) >> 1, buf_idx, buffer);
      taskENTER_CRITICAL();
      res = ( HAL_OK ==HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8) << 1, buffer, buf_idx, 1000) ? 1 : 0);
      taskEXIT_CRITICAL();
      break;
    default:
      return 0;
    }
  return res;
}


void oled(void const * argument){
  UNUSED(argument);

  extern volatile miniStatus status;
  uint8_t *buf = (uint8_t*)pvPortMalloc(512);
  u8g2_Setup_ssd1306_i2c_128x32_univision_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_i2c, u8x8_stm32_gpio_and_delay);
  u8g2_SetBufferPtr(&u8g2, buf);
  u8g2_SetI2CAddress(&u8g2, 0x3c);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_ClearDisplay(&u8g2);
  char msg[15];
  uint8_t i = 0;
  for(;;){
    u8g2_FirstPage(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_t0_11_tr);
    do
      {
	sprintf(msg, "comando: %d %d", status.irCommand, i++);
	u8g2_DrawStr(&u8g2, 0, 8, msg);
	u8g2_DrawFrame(&u8g2, 1, 3, 24, i);
	u8g2_DrawFrame(&u8g2, 26, 3, 24, i);
	u8g2_DrawFrame(&u8g2, 51, 3, 24, i);
	u8g2_DrawFrame(&u8g2, 76, 3, 24, i);
	u8g2_DrawFrame(&u8g2, 101, 3, 24, i);
	i = (i+1) % 28;
      } while (u8g2_NextPage(&u8g2));
    //vTaskDelay(40);
  }
}
