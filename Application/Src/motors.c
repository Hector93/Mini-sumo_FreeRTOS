#include "motors.h"
#include "cmsis_os.h" // para retardos
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "stdint.h"

//F3 3 20 2 orgien destino type speed direction

motorInternalData motorProcessMessage(message msg,motorInternalData data);
motorInternalData motorUpdate(motorInternalData newData);
void motorDirectionInternal(motorInternalData data);
void motorSpeedInternal(motorInternalData data);

void motorR(const void* argument){
  UNUSED(argument);
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  motorInternalData status;
  //  message tx;
  message rx;
  //rx = createMessage(15,2,5,0);

  status.motorOpt.speed = 250;
  status.motorOpt.direction = FORWARD;
  status.motorOpt.channel = motorRID;

  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

  //HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
  //status = motorUpdate(motorProcessMessage(rx,status),status);
  //status = motorUpdate(status,status);

  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      if(pdPASS ==(xQueueReceive(motorRQueueHandle,&rx,10))){

	status = motorUpdate(motorProcessMessage(rx,status));
      }
    }
}

void motorL(const void* argument){
  UNUSED(argument);
  //  HAL_UART_Transmit(&huart1,"motorD process active\r\n",22,100);
  motorInternalData status;
  //message tx;
  message rx;
  //rx = createMessage(15,3,5,0);

  status.motorOpt.speed = 100;
  status.motorOpt.direction = FORWARD;
  status.motorOpt.channel = motorLID;


  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  //HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_RESET);
  //status = motorUpdate(motorProcessMessage(rx,status),status);
  //status = motorUpdate(status,status);

  for(;;)
    {
      //xQueueSend(serialQueueHandle,&tx,100);
      //taskYIELD();
      //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
      if(pdPASS ==(xQueueReceive(motorLQueueHandle,&rx,10))){
	status = motorUpdate(motorProcessMessage(rx,status));
      }


    }
}


motorInternalData motorProcessMessage(message msg,motorInternalData data){
  motorInternalData aux;
  message response;
  //     HAL_UART_Transmit(&huart1,"hola motors\r\n",13,10);
  switch(msg.messageUser.type){
  case motorError:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,0,0);
    xQueueSend(serialQueueHandle,&response, 10);
    return data;
  case stopHard:     // poner pwm de canal a 100% y poner en cero los dos pines del motor
    data.motorOpt.direction = STOPEDHARD;
    data.motorOpt.speed = 255;
    return data;
  case stopFree:     //poner pwm de canal en 0;
    data.motorOpt.speed = STOPED;
    return data;
  case startMotor:
    data.motorData = msg.messageUser.data;
    return data;
  case setDirection:
    aux.motorData = msg.messageUser.data;
    data.motorOpt.direction = aux.motorOpt.direction;
    data.motorOpt.speed = aux.motorOpt.speed;
    motorSpeedInternal(aux);
    return data;
  case setSpeed:
    aux.motorData = msg.messageUser.data;
    data.motorOpt.speed = aux.motorOpt.speed;
    return data;
  case getStatus:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getStatus,data.motorData);
    xQueueSend(serialQueueHandle,&response,10);
    return data;
  case getSpeed:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getSpeed,data.motorData & 0xFF00);
    xQueueSend(serialQueueHandle,&response,10);
    return data;
  case getDirection:
    response = createMessage(data.motorOpt.channel,msg.messageUser.IdpO,getDirection,data.motorData & 0x00FF);
    xQueueSend(serialQueueHandle,&response,10);
    return data;
  case test:
    //llamar a funcion de prueba
    return data;
  default: return data;
  }
}

motorInternalData motorUpdate(motorInternalData newData){
  motorDirectionInternal(newData);
  return newData;
}

void motorSpeedInternal(motorInternalData data){

  HAL_TIM_SetPWM(data.motorOpt.speed,data.motorOpt.channel);
}


void motorDirectionInternal(motorInternalData data){
  switch(data.motorOpt.direction){
  case FORWARD:
    if(data.motorOpt.channel == motorRID){
      if(data.motorOpt.direction == BACKWARDS){
	motorInternalData aux;
	aux.motorOpt.direction = STOPEDHARD;
	aux.motorOpt.channel = motorRID;
	motorDirectionInternal(aux);
	//vTaskDelay(10);
      }
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    }else if(data.motorOpt.channel == motorLID){
      if(data.motorOpt.direction == BACKWARDS){
	motorInternalData aux;
	aux.motorOpt.direction = STOPEDHARD;
	aux.motorOpt.channel = motorLID;
	motorDirectionInternal(aux);
	//vTaskDelay(10);
      }
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_RESET);
    }
    motorSpeedInternal(data);
    break;
  case BACKWARDS:
    if(data.motorOpt.channel == motorRID){
      if(data.motorOpt.direction == FORWARD){
	motorInternalData aux;
	aux.motorOpt.direction = STOPEDHARD;
	aux.motorOpt.channel = motorRID;
	motorDirectionInternal(aux);
	//vTaskDelay(10);
      }
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_SET);
    }else if(data.motorOpt.channel == motorLID){
      if(data.motorOpt.direction == FORWARD){
	motorInternalData aux;
	aux.motorOpt.direction = STOPEDHARD;
	aux.motorOpt.channel = motorLID;
	motorDirectionInternal(aux);
	//vTaskDelay(10);
      }
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_SET);
    }
    motorSpeedInternal(data);
    break;
  case STOPED:
    data.motorOpt.speed = 0;
    motorSpeedInternal(data);
    break;
  case STOPEDHARD:    // poner pwm de canal a 100% y poner en cero los dos pines del motor
    if(data.motorOpt.channel == motorRID){
      HAL_GPIO_WritePin(ph_1A_GPIO_Port,ph_1A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_1B_GPIO_Port,ph_1B_Pin,GPIO_PIN_RESET);
    }else if(data.motorOpt.channel == motorLID){
      HAL_GPIO_WritePin(ph_2A_GPIO_Port,ph_2A_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ph_2B_GPIO_Port,ph_2B_Pin,GPIO_PIN_RESET);
    }
    data.motorOpt.speed = 0;
    motorSpeedInternal(data);
    break;
  }
}

uint16_t createMotorData(uint8_t speed,uint8_t direction, uint8_t channel){
  motorInternalData aux;
  aux.motorOpt.speed = speed;
  aux.motorOpt.direction = direction;
  aux.motorOpt.channel = channel;
  return aux.motorData;
}
uint16_t motorSpeed(uint8_t speed, uint8_t channel){
  return createMotorData(speed,0, channel);
}
uint16_t motorDirection(uint8_t direction, uint8_t channel){
  return createMotorData(0,direction, channel);
}
