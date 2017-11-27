#include "oneWire.h"

void ds18b20Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  static uint8_t sensState[3] = {0};
  sensState[0] = sensState[1] = sensState[2] = 0;
  
  /*Configure GPIO pins : OutTemp_Pin MotorTemp_Pin StrTemp_Pin */
  GPIO_InitStruct.Pin = OutTemp_Pin|MotorTemp_Pin|StrTemp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  
  HAL_GPIO_WritePin(OutTemp_GPIO_Port, OutTemp_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorTemp_GPIO_Port, MotorTemp_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(StrTemp_GPIO_Port, StrTemp_Pin, GPIO_PIN_SET);

  delayMicroseconds(10);
  sensState[0] = HAL_GPIO_ReadPin(OutTemp_GPIO_Port, OutTemp_Pin);
  sensState[1] = HAL_GPIO_ReadPin(MotorTemp_GPIO_Port, MotorTemp_Pin);
  sensState[2] = HAL_GPIO_ReadPin(StrTemp_GPIO_Port, StrTemp_Pin);
  
  HAL_GPIO_WritePin(OutTemp_GPIO_Port, OutTemp_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorTemp_GPIO_Port, MotorTemp_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(StrTemp_GPIO_Port, StrTemp_Pin, GPIO_PIN_RESET);
  delayMicroseconds(720);
  HAL_GPIO_WritePin(OutTemp_GPIO_Port, OutTemp_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorTemp_GPIO_Port, MotorTemp_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(StrTemp_GPIO_Port, StrTemp_Pin, GPIO_PIN_SET);
  delayMicroseconds(10);
  sensState[0] = HAL_GPIO_ReadPin(OutTemp_GPIO_Port, OutTemp_Pin);
  sensState[1] = HAL_GPIO_ReadPin(MotorTemp_GPIO_Port, MotorTemp_Pin);
  sensState[2] = HAL_GPIO_ReadPin(StrTemp_GPIO_Port, StrTemp_Pin);

  delayMicroseconds(60);
  sensState[0] = HAL_GPIO_ReadPin(OutTemp_GPIO_Port, OutTemp_Pin);
  sensState[1] = HAL_GPIO_ReadPin(MotorTemp_GPIO_Port, MotorTemp_Pin);
  sensState[2] = HAL_GPIO_ReadPin(StrTemp_GPIO_Port, StrTemp_Pin);               // 0 - if there is a presense

  delayMicroseconds(400);
  sensState[0] = HAL_GPIO_ReadPin(OutTemp_GPIO_Port, OutTemp_Pin);
  sensState[1] = HAL_GPIO_ReadPin(MotorTemp_GPIO_Port, MotorTemp_Pin);
  sensState[2] = HAL_GPIO_ReadPin(StrTemp_GPIO_Port, StrTemp_Pin);               // should be 1 again
}

