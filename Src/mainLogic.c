#include "main.h"
#include "stm32f1xx_hal.h"

#include "mainLogic.h"
#include "timeMgmnt.h"
#include "portsMgmnt.h"
#include "uartDataExchMgmnt.h"
#include "tm_stm32_hd44780.h"
#include "oneWire.h"
#include "intrinsics.h"

void prepareToTransition ();


extern volatile uint32_t counterContIn;

timeStr timeConsPumpStarted;
static noTareStageEnum noTareStage = THREE;
timeStr waterMissDetectedTime;

// показания выходного миллитрового счетчика, когда было завершено последнее обслуживание 
static uint32_t lastMilLitWentOut = 0;

// состояние автомата
machineParameters wa;

// показания счетчиков (приходят по UART)
filtersStr filters;

// оплачено за все время, оплачено сейчас, осталось отработать
moneyStats money;

counters cnt = {0};

uint32_t valFor10LitInCalibr = 4296;    // количество импульсов расходомера на 10Л. используется для расчетов объема поступившей воды, перелива и воду вне тары
uint32_t valFor10LitOutCalibr = 4810;   // количество импульсов расходомера на 10Л. используется для расчетов объема выдачи воды

// цена литра, в копейках
float waterPrice = 400.0;
// секунд до остановки выходного насоса, если нет воды
uint8_t outPumpNoWaterStopTime = 30;
// минимальный объем воды в контейнере
uint8_t startContVolume = 15;
// минимальный объем воды в контейнере
uint8_t containerMinVolume = 3;
// объем контейнера с водой
uint8_t maxContainerVolume = 95;
// порог включения обогревателя в градусах цельсия
int8_t warmerOnTrshld = 15;

void containerMgmnt() {
  wa.currentContainerVolume = (cnt.milLitContIn - cnt.milLitWentOut - cnt.milLitloseCounter) / 10;
  if (cnt.milLitWentOut + (maxContainerVolume-5)*1000 > cnt.milLitContIn) wa.container = NOT_FULL;
  checkMagistralPressure();
  
  if (wa.magistralPressure == HI_PRESSURE && wa.container != FULL) {
    if (wa.mainPump != WORKING) {
      delayMilliseconds(100);
      if (wa.magistralPressure == HI_PRESSURE) {
        MAINV_ON();
        MAINP_ON();
//        FILTV_ON();
      }
    }
  }
  if (wa.magistralPressure == NO_PRESSURE || wa.container == FULL) {
    if (wa.mainPump != STOPPED) {
      MAINP_OFF();
      MAINV_OFF();
//      FILTV_OFF();
    }
  }
}

void buttonMgmnt(){
  timeStr time = getCurTime();
  if (wa.machineState == NOT_READY) TURN_BUT_LED_OFF();
  if (wa.machineState == WAIT) {
    uint8_t secTenFrac = time.sec % 20;
    if (secTenFrac > 10) TURN_BUT_LED_ON();
    else {
      secTenFrac /= 2;
      if (secTenFrac % 2 == 1) TURN_BUT_LED_OFF();
      else {
        if (time.msec / 5 < 100) TURN_BUT_LED_OFF();
        else TURN_BUT_LED_ON();
      }
    }
  }  
  if (wa.machineState == NO_TARE)     {
    if (time.msec %250 > 125) TURN_BUT_LED_ON();
    else TURN_BUT_LED_OFF();
  }
  if (wa.machineState == CONFIG)      TURN_BUT_LED_OFF();
  if (wa.machineState == WASH_FILTER) TURN_BUT_LED_OFF();
  if (wa.machineState == SERVICE)     TURN_BUT_LED_OFF();
  if (wa.machineState == FREE)        TURN_BUT_LED_OFF();
  
  // if (wa.machineState == JUST_PAID) реализовано в обработчике прерывания 1мс
  // if (wa.machineState == WORK) реализовано в обработчике прерывания 1мс
}

void outPumpMgmnt() {
  static timeStr timeCheck = {0};
  static uint32_t lastMillilit = 0;
  if (!(wa.machineState == NO_TARE || wa.machineState == WORK)) return;

  // NO_TARE condition detected, let's turn off out pump
  if (wa.waterMissDetected == true) {
    if (wa.consumerPump == WORKING) CONSUMP_OFF();
    if (getTimeDiff(waterMissDetectedTime) > 500) {
      wa.waterMissDetected = false;
    }
  }
  
  // check if there is no water in container
  static uint8_t noWaterOut = 0;
  if (getTimeDiff(timeCheck) > 500 && wa.consumerPump == WORKING) {
    writeTime(&timeCheck);
    if (cnt.milLitWentOut > lastMillilit + 10) noWaterOut = 0;
    else if (noWaterOut < 255) noWaterOut++;
    lastMillilit = cnt.milLitWentOut;
  }
  
  // turn off out pump if there is no water in container
  if (noWaterOut > outPumpNoWaterStopTime) {
    if (wa.consumerPump == WORKING) CONSUMP_OFF();
    noWaterOut = 0;
    setContainerValToZero(maxContainerVolume);
  }
}

void lcdMgmnt() {
  static timeStr refreshLCDtime = {0};
  if (getTimeDiff(refreshLCDtime) > 250) {
    writeTime(&refreshLCDtime);
    if (wa.machineState == NOT_READY)   printNotReady(wa.currentContainerVolume);
    if (wa.machineState == WAIT)        printWait(wa.currentContainerVolume);
    if (wa.machineState == JUST_PAID)   {
      uint32_t temp = money.sessionPaid;        // avoid undefined behavior warning
      printPaid(temp/100, (uint16_t)(((float)temp*10.0)/waterPrice));
    }
    if (wa.machineState == WORK) {
      bool pause = !(bool)wa.consumerPump;
      wa.litersLeftFromSession = (uint32_t)((money.leftFromPaid*10.0)/waterPrice);
      printGiven((uint32_t)wa.litersLeftFromSession, (cnt.milLitWentOut - lastMilLitWentOut) / 100, (uint32_t) money.leftFromPaid/100, pause);
    }
    if (wa.machineState == NO_TARE)     printLoseDetected();
    if (wa.machineState == CONFIG)      TM_HD44780_Clear();
    if (wa.machineState == WASH_FILTER) TM_HD44780_Clear();
    if (wa.machineState == SERVICE)     TM_HD44780_Clear();
    if (wa.machineState == FREE)        TM_HD44780_Clear();
  }
} 

void lghtsMgmnt() {
  setGlobal(10);
  timeStr time = getCurTime();
  if (wa.machineState != WORK) {
    setBlue(10);
    setRed(0);
    if (time.sec % 2) {
      setGreen(10 - time.msec / 100);
    }
    else {
      setGreen(time.msec / 100);
    }
  }
  else {
    if (time.sec % 2) {      
      setGreen(time.msec / 100);
      setBlue(time.msec / 100);
      setRed(time.msec / 100);
    }
    else {
      setGreen(10 - time.msec / 100);
      setBlue(10 - time.msec / 100);
      setRed(10 - time.msec / 100);
    }
  }
}

void ADCMgmnt() {
  uint16_t suppVolCH4, adcCH5, adcTempMCU, intRef;
  readAdc(&suppVolCH4, &adcCH5, &adcTempMCU, &intRef);
  
  wa.suppVoltage = (uint16_t)(1.5/(float)intRef * (float)suppVolCH4 * 3.13 * 1000.0); 
  wa.tempMCU = (uint16_t)((1.43 - 1.5/(float)intRef * (float)adcTempMCU) / 4.3e-3 + 25.0); 
}

void prepareToTransition (){
  disableButtonsForTime();
  disableSensorsForTime();  
  TM_HD44780_Clear();
  
  if (wa.machineState == NOT_READY) INHIBIT_DIS();
  else INHIBIT_EN();
  
  if (wa.machineState == WAIT){    
    INHIBIT_EN();
    CONSUMP_OFF();
    wa.machineState = WAIT;
  }
  
  if (wa.machineState == WORK) {
    if (wa.lastMachineState == JUST_PAID) {
      lastMilLitWentOut = cnt.milLitWentOut;
    }
    CONSUMP_ON();
  }
  
  if (wa.machineState == NO_TARE) {
    writeTime(&waterMissDetectedTime);
    if (wa.consumerPump == WORKING) {
      CONSUMP_OFF();
    }
  }
  wa.lastMachineState = wa.machineState; 
}
              
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg) {
  NVIC_SystemReset();
}

uint32_t getNoTareProtTime (noTareStageEnum noTare) {
  switch (noTare) {
    case ZERO:    return 3000;
                  break;
    case THREE:   return 3000;
                  break;
    case FIVE:    return 5000;
                  break;
    case FIFTEEN: return 15000;
                  break;
    case THIRTY:  return 30000;
                  break;
    case SIXTY:   return 60000;
                  break;
  }
  return 0;
}

void temperatureMgmnt (void) {
  static bool started = false;
  timeStr lastTimeStarted = {0}; 
  if (started == false) {
    started = true;
    writeTime(&lastTimeStarted);
    swiStartMeasrmnt();
  }
  else {
    if (getTimeDiff(lastTimeStarted) > 1000) {
      started = false;
      int16_t temps[6] = {0};
      swiReadTemp(&temps[0]);
      
      wa.OutTp = (int8_t)(temps[0] >> 4);
      wa.MotorTp = (int8_t)(temps[1] >> 4);
      wa.StreetTp = (int8_t)(temps[2] >> 4);
      wa.inWaterTp = (int8_t)(temps[3] >> 4);
      wa.cntWaterTp = (int8_t)(temps[4] >> 4);
      wa.boardTp = (int8_t)(temps[5] >> 4);
    }
  }
  if (wa.OutTp < warmerOnTrshld) WARM_ON();     // 1 градус гистерезиса
  if (wa.OutTp > warmerOnTrshld) WARM_OFF();    
} 
  

void mainSetup() {
  initUART();
  wa.machineState = WAIT;
  wa.lastMachineState = FREE;
  prepareToTransition();
#ifdef DEBUG_PCB_MODE
  initCheckLoop();
  checkLoop();
#endif

#ifdef NON_STANDART_NO_TARE_COUNTER
  HAL_GPIO_DeInit (GPIOE, GPIO_PIN_4);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
#endif
#ifdef NON_STANDART_FULL_CONTAINER_COUNTER
  HAL_GPIO_DeInit (NINT_IN20_GPIO_Port, NINT_IN20_Pin);
  GPIO_InitTypeDef GPIO_InitStruct2;
  GPIO_InitStruct2.Pin = NINT_IN20_Pin;
  GPIO_InitStruct2.Pull = GPIO_PULLUP;
  GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(NINT_IN20_GPIO_Port, &GPIO_InitStruct2);
#endif
  COOLER_ON();
  B_ON();
  setupDefaultLitersVolume(50);
  
  refreshWatchDogs();
  ds18b20Init();  
  wa.OutTp = 25;
}


void mainLoop (void) {
////// MANAGE STUFF   
  refreshWatchDogs();
  
  temperatureMgmnt();
  ADCMgmnt();
  containerMgmnt();
  uartDataExchMgmnt();
  lcdMgmnt();
  outPumpMgmnt();
  buttonMgmnt();
  lghtsMgmnt();
    
  if (wa.machineState == WAIT);
  if (wa.machineState == NOT_READY);
  
  if (wa.machineState == WORK) {
    if (wa.waterMissDetected == true){
      wa.machineState = NO_TARE;
      if (getTimeDiff(timeConsPumpStarted) < getNoTareProtTime(noTareStage)) if (noTareStage != SIXTY) noTareStage++;
      else noTareStage = ZERO;
      prepareToTransition();
    }
    money.leftFromPaid = money.sessionPaid - (((double)cnt.milLitWentOut - (double)lastMilLitWentOut) / 1000.0) * waterPrice;      
    int32_t moneyInt = (int32_t) money.leftFromPaid;
    if (moneyInt <= 0) {
      //uint32_t temp = cnt.milLitWentOut;
      //while(cnt.milLitWentOut < temp + 20);
      wa.machineState = WAIT;
      prepareToTransition();
      money.totalPaid += money.sessionPaid - (uint32_t)money.leftFromPaid;
      money.sessionPaid = 0;
      money.leftFromPaid = 0.0;
    }
  }

////// TRANSITIONS MANAGMENT
  if (money.sessionPaid > 0 && wa.machineState == WAIT){
    wa.machineState = JUST_PAID;
    prepareToTransition();
    disableButtonsForTime();  
    clrUserButton();      
  }
  if (wa.currentContainerVolume < containerMinVolume * 100 && wa.machineState == WAIT) {
    wa.machineState = NOT_READY;
    prepareToTransition();
  }
  if (wa.currentContainerVolume >= containerMinVolume * 100 && wa.machineState == NOT_READY) {
    wa.machineState = WAIT;
    prepareToTransition();
  }
  
////// BUTTONS PROCESSING        
  if (isUserButtonPressed() && wa.machineState == JUST_PAID) {
    wa.machineState = WORK;
    writeTime(&timeConsPumpStarted);                                          // to check NO_TARE condition at the start
    noTareStage = ZERO;
    prepareToTransition();                                                     
    disableButtonsForTime();  
    clrUserButton();
  }
  if (isUserButtonPressed() && wa.machineState == WORK) {
    disableButtonsForTime();  
    disableSensorsForTime();
    clrUserButton();
    if (wa.consumerPump == WORKING) CONSUMP_OFF();
    else {
      CONSUMP_ON();
      if (noTareStage > ZERO) writeTime(&timeConsPumpStarted);                // new time for CosumPump to remember if last time 
    }
  }
    
  if (isUserButtonPressed() && wa.machineState == NO_TARE) {
    if (getTimeDiff(timeConsPumpStarted) > getNoTareProtTime(noTareStage)/2) {          // button works only after 
      if (getTimeDiff(waterMissDetectedTime) > 1000) {
        wa.machineState = WORK;
        prepareToTransition();
      }
    }
    clrUserButton();
  } 
  if (wa.machineState == NO_TARE && getTimeDiff(waterMissDetectedTime) > (getNoTareProtTime(noTareStage))) {      // return to WORK with pause mode
    wa.machineState = WORK;
  }
  
  if (isUserButtonPressed() && wa.machineState == WAIT) {
    clrUserButton();
  }
  
  if (isAdminDownButtonPressed()) {
    
    disableButtonsForTime();  
    clrServDownButton();
  }
  if (isAdminLeftButtonPressed()) {
    
    disableButtonsForTime();  
    clrServLeftButton();
  }
  if (isAdminRightButtonPressed()) {
    
    disableButtonsForTime();  
    clrServRightButton();
  }
  
}