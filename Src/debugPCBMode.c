#include "stm32f1xx_hal.h"
#include "debugPCBMode.h"
#include "timeMgmnt.h"
#include "portsMgmnt.h"
#include "tm_stm32_hd44780.h"
#include "uartDataExchMgmnt.h"

void setUpCheckGPIO();
void clrAll();

void setupIn();
void setupGPIOOut();
void setupULN2003Out();
void setupMOSFET();
void setupPwrOut();
void setupW26();

void initCheckLoop() {
  setUpCheckGPIO();
  clrAll();
}

void checkLoop(){
  while(1) {
    uartDataExchMgmnt();
    if (isAdminUpButtonPressed()) {
      printAdminUp();
      setupIn();
      setupPwrOut();
      disableButtonsForTime();  
      clrServUpButton();
    }
    if (isAdminDownButtonPressed()) {
      printAdminDown();
      setupGPIOOut();
      disableButtonsForTime();  
      clrServDownButton();
    }
    if (isAdminLeftButtonPressed()) {
      printAdminLeft();
      setupULN2003Out();
      disableButtonsForTime();  
      clrServLeftButton();
    }
    if (isAdminRightButtonPressed()) {
      printAdminRight();
      setupMOSFET();
      setupW26();
      disableButtonsForTime();  
      clrServRightButton();
    }
  }
}


void setUpCheckGPIO() {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = INT_IN4_Pin|INT_IN3_Pin|INT_IN2_Pin|INT_IN1_Pin|OUT8_Pin|
                        OUT9_Pin|OUT10_Pin|OUT11_Pin|OUT12_Pin|OUT13_Pin|OUT14_Pin|
                        MOSFET1_Pin|MOSFET2_Pin|INT_IN6_Pin|INT_IN5_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = W26_DI_Pin|INT_TEMP2_Pin|INT_TEMP1_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|
                        LCD_D4_Pin|OUT3_Pin|OUT4_Pin|PWR5_Pin|PWR4_Pin|PWR3_Pin|PWR2_Pin|
                        INT_TEMP2_Pin|INT_TEMP1_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_RS_Pin|OUT1_Pin|OUT2_Pin|PWR1_Pin|W26_DO_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OUT5_Pin|OUT6_Pin|OUT7_Pin|NINT_TEMP3_Pin|NINT_IN13_Pin|NINT_IN12_Pin|
                        NINT_IN11_Pin|NINT_IN10_Pin|INT_IN9_Pin|INT_IN8_Pin|INT_IN7_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO7_Pin|GPIO6_Pin|GPIO5_Pin|GPIO4_Pin|GPIO3_Pin|GPIO2_Pin|
                        GPIO1_Pin|PWR6_Pin|NINT_IN21_Pin|NINT_IN20_Pin|NINT_IN19_Pin|
                        NINT_IN18_Pin|NINT_IN17_Pin|NINT_IN16_Pin|NINT_IN15_Pin|NINT_IN14_Pin;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = SERV_BUT4_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SERV_BUT3_Pin|SERV_BUT2_Pin|SERV_BUT1_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void setupGPIOOut() {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, state);
              break;
    case 2:   HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, state);
              break;
    case 3:   HAL_GPIO_WritePin(GPIO4_GPIO_Port, GPIO4_Pin, state);
              break;
    case 4:   HAL_GPIO_WritePin(GPIO5_GPIO_Port, GPIO5_Pin, state);
              break;
    case 5:   HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, state);
              break;
    case 6:   HAL_GPIO_WritePin(GPIO7_GPIO_Port, GPIO7_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 6) num = 0;
    state = GPIO_PIN_SET;
  }
}


void setupMOSFET() {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(MOSFET1_GPIO_Port, MOSFET1_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(MOSFET2_GPIO_Port, MOSFET2_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 1) num = 0;
    state = GPIO_PIN_SET;
  }
}

void setupW26() {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(W26_DO_GPIO_Port, W26_DO_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(W26_DI_GPIO_Port, W26_DI_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 1) num = 0;
    state = GPIO_PIN_SET;
  }
}

void setupIn() {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(INT_IN1_GPIO_Port, INT_IN1_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(INT_IN2_GPIO_Port, INT_IN2_Pin, state);
              break;
    case 2:   HAL_GPIO_WritePin(INT_IN3_GPIO_Port, INT_IN3_Pin, state);
              break;
    case 3:   HAL_GPIO_WritePin(INT_IN4_GPIO_Port, INT_IN4_Pin, state);
              break;
    case 4:   HAL_GPIO_WritePin(INT_IN5_GPIO_Port, INT_IN5_Pin, state);
              break;
    case 5:   HAL_GPIO_WritePin(INT_IN6_GPIO_Port, INT_IN6_Pin, state);
              break;
    case 6:   HAL_GPIO_WritePin(INT_IN7_GPIO_Port, INT_IN7_Pin, state);
              break;
    case 7:   HAL_GPIO_WritePin(INT_IN8_GPIO_Port, INT_IN8_Pin, state);
              break;
    case 8:   HAL_GPIO_WritePin(INT_IN9_GPIO_Port, INT_IN9_Pin, state);
              break;
    case 9:   HAL_GPIO_WritePin(NINT_IN10_GPIO_Port, NINT_IN10_Pin, state);
              break;
    case 10:  HAL_GPIO_WritePin(NINT_IN11_GPIO_Port, NINT_IN11_Pin, state);
              break;
    case 11:  HAL_GPIO_WritePin(NINT_IN12_GPIO_Port, NINT_IN12_Pin, state);
              break;
    case 12:  HAL_GPIO_WritePin(NINT_IN13_GPIO_Port, NINT_IN13_Pin, state);
              break;
    case 13:  HAL_GPIO_WritePin(NINT_IN14_GPIO_Port, NINT_IN14_Pin, state);
              break;
    case 14:  HAL_GPIO_WritePin(NINT_IN15_GPIO_Port, NINT_IN15_Pin, state);
              break;
    case 15:  HAL_GPIO_WritePin(NINT_IN16_GPIO_Port, NINT_IN16_Pin, state);
              break;
    case 16:  HAL_GPIO_WritePin(NINT_IN17_GPIO_Port, NINT_IN17_Pin, state);
              break;
    case 17:  HAL_GPIO_WritePin(NINT_IN18_GPIO_Port, NINT_IN18_Pin, state);
              break;
    case 18:  HAL_GPIO_WritePin(NINT_IN19_GPIO_Port, NINT_IN19_Pin, state);
              break;
    case 19:  HAL_GPIO_WritePin(NINT_IN20_GPIO_Port, NINT_IN20_Pin, state);
              break;
    case 20:  HAL_GPIO_WritePin(NINT_IN21_GPIO_Port, NINT_IN21_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 20) num = 0;
    state = GPIO_PIN_SET;
  }
}

void setupULN2003Out () {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, state);
              break;
    case 2:   HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, state);
              break;
    case 3:   HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, state);
              break;
    case 4:   HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, state);
              break;
    case 5:   HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, state);
              break;
    case 6:   HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, state);
              break;
    case 7:   HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, state);
              break;
    case 8:   HAL_GPIO_WritePin(OUT9_GPIO_Port, OUT9_Pin, state);
              break;
    case 9:   HAL_GPIO_WritePin(OUT10_GPIO_Port, OUT10_Pin, state);
              break;
    case 10:  HAL_GPIO_WritePin(OUT11_GPIO_Port, OUT11_Pin, state);
              break;
    case 11:  HAL_GPIO_WritePin(OUT12_GPIO_Port, OUT12_Pin, state);
              break;
    case 12:  HAL_GPIO_WritePin(OUT13_GPIO_Port, OUT13_Pin, state);
              break;
    case 13:  HAL_GPIO_WritePin(OUT14_GPIO_Port, OUT14_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 13) num = 0;
    state = GPIO_PIN_SET;
  }
}

void setupPwrOut() {
  static uint8_t num = 0;
  static GPIO_PinState state = GPIO_PIN_SET;
  switch (num) {
    case 0:   HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, state);
              break;
    case 1:   HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, state);
              break;
    case 2:   HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, state);
              break;
    case 3:   HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, state);
              break;
    case 4:   HAL_GPIO_WritePin(PWR5_GPIO_Port, PWR5_Pin, state);
              break;
    case 5:   HAL_GPIO_WritePin(PWR6_GPIO_Port, PWR6_Pin, state);
              break;
  }
  if (state == GPIO_PIN_SET) {
    state = GPIO_PIN_RESET;
  }
  else {
    if (++num > 5) num = 0;
    state = GPIO_PIN_SET;
  }
}

void clrAll() {
  HAL_GPIO_WritePin(GPIOC, OUT3_Pin|OUT4_Pin|PWR5_Pin|PWR4_Pin|PWR3_Pin|PWR2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, OUT1_Pin|OUT2_Pin|PWR1_Pin|W26_DO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, OUT5_Pin|OUT6_Pin|OUT7_Pin|NINT_TEMP3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, OUT8_Pin|OUT9_Pin|OUT10_Pin|OUT11_Pin|OUT12_Pin|OUT13_Pin|OUT14_Pin|MOSFET1_Pin|MOSFET2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO7_Pin|GPIO6_Pin|GPIO5_Pin|GPIO4_Pin|GPIO3_Pin|GPIO2_Pin|GPIO1_Pin|PWR6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(W26_DI_GPIO_Port, W26_DI_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN1_GPIO_Port, INT_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN2_GPIO_Port, INT_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN3_GPIO_Port, INT_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN4_GPIO_Port, INT_IN4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN5_GPIO_Port, INT_IN5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN6_GPIO_Port, INT_IN6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN7_GPIO_Port, INT_IN7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN8_GPIO_Port, INT_IN8_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INT_IN9_GPIO_Port, INT_IN9_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN10_GPIO_Port, NINT_IN10_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN11_GPIO_Port, NINT_IN11_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN12_GPIO_Port, NINT_IN12_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN13_GPIO_Port, NINT_IN13_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN14_GPIO_Port, NINT_IN14_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN15_GPIO_Port, NINT_IN15_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN16_GPIO_Port, NINT_IN16_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN17_GPIO_Port, NINT_IN17_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN18_GPIO_Port, NINT_IN18_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN19_GPIO_Port, NINT_IN19_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN20_GPIO_Port, NINT_IN20_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NINT_IN21_GPIO_Port, NINT_IN21_Pin, GPIO_PIN_RESET);
}
