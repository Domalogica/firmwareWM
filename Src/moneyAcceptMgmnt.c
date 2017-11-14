#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "moneyAcceptMgmnt.h"
#include "timeMgmnt.h"
#include "main.h"

extern moneyStats money;

void checkCoinMoney(void) {
  static timeStr trueTime;
  static bool trueFlag = false;
  
  if (trueFlag == true) {
    if (HAL_GPIO_ReadPin(COIN_PORT, COIN_PIN)) {
      if (getTimeDiff(trueTime) > 5) {
        money.sessionPaid += 100;
        money.leftFromPaid += 100.0;            // в копейках
      }
      trueFlag = false;
    }
  }
  else {
    if (!HAL_GPIO_ReadPin(COIN_PORT, COIN_PIN)) {
      writeTime(&trueTime);
      trueFlag = true;
    }
  }
} 
  
void checkMoney(void) {
  static timeStr trueTime;
  static bool trueFlag = false;
  
  if (trueFlag == true) {
    if (HAL_GPIO_ReadPin(MONEY_PORT, MONEY_PIN)) {
      if (getTimeDiff(trueTime) > 5) {
        money.sessionPaid += 1000;
        money.leftFromPaid += 1000.0;           // в копейках
      }
      trueFlag = false;
    }
  }
  else {
    if (!HAL_GPIO_ReadPin(MONEY_PORT, MONEY_PIN)) {
      writeTime(&trueTime);
      trueFlag = true;
    }
  }
}