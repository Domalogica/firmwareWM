#include "oneWire.h"

void swiSendBit(bool );
void swiSendByte(uint8_t );
void swiReadByte(uint8_t *);
void swiReset();

inline void setupInputPullUps() {
  HAL_GPIO_WritePin(GPIOB, OutTpPin|MotorTpPin|StreetTpPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(cntWaterTpPort, cntWaterTpPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(inWaterTpPort, inWaterTpPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NINT_TEMP1_GPIO_Port, NINT_TEMP1_Pin, GPIO_PIN_SET);
}

inline void setupOutputLowOpenDrains() {
  HAL_GPIO_WritePin(GPIOB, OutTpPin|MotorTpPin|StreetTpPin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(cntWaterTpPort, cntWaterTpPin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(inWaterTpPort, inWaterTpPin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_TEMP1_GPIO_Port, NINT_TEMP1_Pin, GPIO_PIN_RESET);
}

void readTempSensorsData (uint8_t * array) {
  array[0] = HAL_GPIO_ReadPin(OutTpPort, OutTpPin);
  array[1] = HAL_GPIO_ReadPin(MotorTpPort, MotorTpPin);
  array[2] = HAL_GPIO_ReadPin(StreetTpPort, StreetTpPin);
  array[3] = HAL_GPIO_ReadPin(cntWaterTpPort, cntWaterTpPin);
  array[4] = HAL_GPIO_ReadPin(inWaterTpPort, inWaterTpPin);
  array[5] = HAL_GPIO_ReadPin(NINT_TEMP1_GPIO_Port, NINT_TEMP1_Pin);
}

void swiSendBit(bool oneZero) {
  __disable_interrupt();
  if (oneZero) {
    setupOutputLowOpenDrains();
    delayMicroseconds(10);
    setupInputPullUps();
    delayMicroseconds(55);
  }
  else {
    setupOutputLowOpenDrains();
    delayMicroseconds(65);
    setupInputPullUps();
    delayMicroseconds(5);
  }
  __enable_interrupt();
}

void swiSendByte(uint8_t byte) {
  for(uint8_t i = 0; i < 8; i++) {
    swiSendBit(byte & (0x01 << i));
  }
}

void swiStartMeasrmnt(){
  swiReset();
  swiSendByte(0xCC);
  swiSendByte(0x44);
}

void swiReadTemp(int16_t * arr) {
  swiReset();
  swiSendByte(0xCC);
  swiSendByte(0xBE);

  static uint8_t arr8[6];
  swiReadByte(&arr8[0]);
  for (uint8_t i = 0; i < 6; i++) {
    arr[i] = 0;
    arr[i] |= arr8[i];
  }
  swiReadByte(&arr8[0]);
  for (uint8_t i = 0; i < 6; i++) arr[i] |= (arr8[i] << 8);
}

void swiReadBits (uint8_t * arr) {
  __disable_interrupt();
  setupOutputLowOpenDrains();
  delayMicroseconds(3);
  setupInputPullUps();
  delayMicroseconds(10);
  readTempSensorsData(&arr[0]);
  delayMicroseconds(53);
  __enable_interrupt();
}

void swiReadByte (uint8_t * arr) {
  uint8_t bitMask;
  arr[0] = arr[1] = arr[2] = arr[3] = arr[4] = arr[5] = 0;
  for (bitMask = 0x01; bitMask; bitMask <<= 1) {
    uint8_t dataArr[6];
    swiReadBits(&dataArr[0]);
    if (dataArr[0]) arr[0] |= bitMask;
    if (dataArr[1]) arr[1] |= bitMask;
    if (dataArr[2]) arr[2] |= bitMask;
    if (dataArr[3]) arr[3] |= bitMask;
    if (dataArr[4]) arr[4] |= bitMask;
    if (dataArr[5]) arr[5] |= bitMask; 
  }
}

void initTempPins() {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = OutTpPin|MotorTpPin|StreetTpPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(GPIOB, OutTpPin|MotorTpPin|StreetTpPin, GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = inWaterTpPin;
  HAL_GPIO_Init(inWaterTpPort, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(inWaterTpPort, inWaterTpPin, GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = cntWaterTpPin;
  HAL_GPIO_Init(cntWaterTpPort, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(cntWaterTpPort, cntWaterTpPin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = NINT_TEMP1_Pin;
  HAL_GPIO_Init(NINT_TEMP1_GPIO_Port, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(NINT_TEMP1_GPIO_Port, NINT_TEMP1_Pin, GPIO_PIN_SET);
  
  
}

void ds18b20Init(void) {
  uint8_t sensData[6] = {0};
  
  initTempPins();
  
  // input pull up
  setupInputPullUps();
  delayMicroseconds(10);
  readTempSensorsData(&sensData[0]);            // should be 1s - line pulled up
  
  // output OD output low
  setupOutputLowOpenDrains();
  delayMicroseconds(720);
  setupInputPullUps();
  delayMicroseconds(10);
  readTempSensorsData(&sensData[0]);            // should be 1s - line pulled up
  
  delayMicroseconds(60);
  readTempSensorsData(&sensData[0]);            // should be 0s - presense signal
  delayMicroseconds(400);
  readTempSensorsData(&sensData[0]);            // should be 1s again
}

void swiReset(){
  __disable_interrupt();
  setupInputPullUps();
  delayMicroseconds(10);
  
  setupOutputLowOpenDrains();
  delayMicroseconds(720);
  
  setupInputPullUps();
  delayMicroseconds(470);
  __enable_interrupt();
}