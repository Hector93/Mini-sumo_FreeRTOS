#include "mini.h"
#include "stdint.h"
#include "message.h"
#include "cmsis_os.h"
#include "usart.h"
#include "imu.h"
#include "adc.h"
#include "packet.h"
#include "sensorsDist.h"
#include "sensorsFloor.h"
#include "motors.h"
#include "fuzzyWrapper.h"
#include "tim.h"

volatile long imuHeading = 0;
sensorDistData distSensorData;
extern uint16_t sensorDistDataRaw[ADC_CHANELS];

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

void miniprocessMessage(message *rx);
void irMessage(message *rx);
void imuMessage(message *rx);
void sensorsFloorMsg(message *rx);
void sensorsDistMsg(message *rx);
void controller();
void motorControl(uint8_t motorLType, uint16_t motorL, uint8_t motorRType, uint16_t motorR);

char testmini[5];
volatile miniStatus status;

void mini(void const * argument){
  UNUSED(argument);
  HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim3);
  //Fuzzy* c = newFuzzy();
  message rx, tx;
  uint8_t vel = 255;
  for(;;){
    if(pdPASS == (xQueueReceive(miniQueueHandle, &rx, 100))){
      miniprocessMessage(&rx);
    }
  }
}

void controller(){
  /**
   * procesar sensores de piso
   * procesar sensores de distancia
   * procesar acelerometro
   * regresar decision
   */
  //piso [x,x,x,x,x]
  //dist 0 0 0 0 0
  //acel 23



}

void miniprocessMessage(message *rx){
  switch(rx->messageUser.IdpO){
  case sensorsFloorID:
    sensorsFloorMsg(rx);
    break;
  case sensorsDistID:
    sensorsDistMsg(rx);
    break;
  case imuId:
    imuMessage(rx);
    break;
  case irID:
    irMessage(rx);
  default:
    break;

  }
}

void motorControl(uint8_t motorLType, uint16_t motorL, uint8_t motorRType, uint16_t motorR){
  message tx;
  tx = createMessage(miniId, motorLID, motorLType, motorL);
  xQueueSend(motorLQueueHandle, &tx, 50);
  tx = createMessage(miniId, motorRID, motorRType, motorR);
  xQueueSend(motorRQueueHandle, &tx, 50);
}

void irMessage(message *rx){
  switch(rx->messageUser.type){
  case COMMAND:
    status.irCommand = rx->messageUser.data;
    break;
  }
}

void imuMessage(message *rx){
  switch(rx->messageUser.type){
  case HEADING:
    status.heading = imuHeading;
    break;
  case TAP_X_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapxu\n", 6, 100);
    break;
  case TAP_X_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapxd\n", 6, 100);
    break;
  case TAP_Y_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapyu\n", 6, 100);
    break;
  case TAP_Y_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapyd\n", 6, 100);
    break;
  case TAP_Z_U:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapzu\n", 6, 100);
    break;
  case TAP_Z_D:
    HAL_UART_Transmit(&huart1, (uint8_t*)"tapzd\n", 6, 100);
    break;
  case PORTRAIT:
    HAL_UART_Transmit(&huart1, (uint8_t*)"portr\n", 6, 100);
    break;
  case LANDSCAPE:
    HAL_UART_Transmit(&huart1, (uint8_t*)"lands\n", 6, 100);
    break;
  case REV_PORTRAIT:
    HAL_UART_Transmit(&huart1, (uint8_t*)"revpo\n", 6, 100);
    break;
  case REV_LANDSCAPE:
    HAL_UART_Transmit(&huart1, (uint8_t*)"revla\n", 6, 100);
    break;
  default:
    break;
  }
}

void sensorsFloorMsg(message *rx){
  switch(rx->messageUser.type){
  case ALL_SENSORS:
    status.irFloor = rx->messageUser.data;
    break;
  default:
    break;
  }
}

void sensorsDistMsg(message *rx){
  switch(rx->messageUser.type){
  case GALLSENSORS:
    status.irDist.distDataRaw = rx->messageUser.data;
    break;
  case DIRECTION:
    status.direction = rx->messageUser.data;
  default:
  case RAWDATA:
    for(uint8_t i = 0; i < ADC_CHANELS; i++){
      status.irDistData[i] = sensorDistDataRaw[i];
    }
    //memcpy(status.irDistData, sensorDistDataRaw, ADC_CHANELS * sizeof(uint16_t));
    break;
  }
}
