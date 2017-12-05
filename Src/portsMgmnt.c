#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "main.h"
#include "portsMgmnt.h"
#include "timeMgmnt.h"
#include "intrinsics.h"
#include <stdlib.h>

#define FIR_TO_CALIB_SIZE  4

extern machineParameters wa;                            // состояние автомата
extern counters cnt;

// внешние настройки
extern uint32_t valFor10LitInCalibr, valFor10LitOutCalibr;
extern uint8_t maxContainerVolume;                      // объем контейнера с водой

volatile bool userButton = false;
volatile bool servUpButton = false;
volatile bool servDownButton = false;
volatile bool servLeftButton = false;
volatile bool servRightButton = false;

timeStr buttonsDisabledTime = {0}, sensorsDisabledTime = {0};
volatile uint32_t counterContIn = 0;
  
static struct waterCounters{
  uint32_t containerIn;
  uint32_t containerOut;
  uint32_t containerLose;
  uint32_t containerPause;
} waterCounters;

void clrUserButton() {
  userButton = false;
}
void clrServUpButton() {
  servUpButton = false;
}
void clrServDownButton() {
  servDownButton = false;
}
void clrServLeftButton() {
  servLeftButton = false;
}
void clrServRightButton() {
  servRightButton = false;
}

bool isUserButtonPressed() {
  return userButton;
}
bool isAdminUpButtonPressed() {
  return servUpButton;
}
bool isAdminDownButtonPressed() {
  return servDownButton;
}
bool isAdminLeftButtonPressed() {
  return servLeftButton;
}
bool isAdminRightButtonPressed() {
  return servRightButton;
}

void disableSensorsForTime (void) {
  writeTime(&sensorsDisabledTime);
}

void disableButtonsForTime (void) {
  writeTime(&buttonsDisabledTime);
}

void checkTumperDoor(){
  if (READ_DOOR_TUMPER()) wa.tumperDoor = DETECTED;
  else wa.tumperDoor = OK;
}

void checkMagistralPressure(){
  if (READ_MAG_PRESURE()) wa.magistralPressure = HI_PRESSURE;
  else wa.magistralPressure = NO_PRESSURE;
}

void checkTumperMoney(){
  if (READ_MONEY_TUMPER()) wa.tumperMoney = OK;
  else wa.tumperMoney = DETECTED;
}

void checkFreeMode(){
  if (READ_FREE_INPUT()) wa.free = NOT_FREE;
  else wa.free = FREE_MODE;
}

// Простой расходомер на входе в контейнер
void countContainerHandler(){
  counterContIn++;
  
  waterCounters.containerIn++;
  cnt.milLitContIn = (uint32_t)((double)waterCounters.containerIn * (10000.0)/(double)valFor10LitInCalibr); 
}

void setupDefaultLitersVolume(uint16_t volume) {
  waterCounters.containerIn = valFor10LitInCalibr/10 * volume;
  cnt.milLitContIn = (uint32_t)((double)waterCounters.containerIn * (10000.0)/(double)valFor10LitInCalibr); 
}

void setContainerValToZero(uint16_t maxContVolLit) {
  uint32_t substr = valFor10LitInCalibr/10 * maxContVolLit;
  if (waterCounters.containerIn > substr) waterCounters.containerIn -= substr;
  else waterCounters.containerIn = 0;
  cnt.milLitContIn = (uint32_t)((double)waterCounters.containerIn * (10000.0)/(double)valFor10LitInCalibr); 
}

// Нестандартный датчик отсутствия тары
void checkNoTare() {
#ifdef NON_STANDART_NO_TARE_COUNTER
  static timeStr timeNoTareDetected = {0};
  static timeStr noSenseTime = {0};
  static bool firstExe = false;
  
  bool inNoTareState = 0; 
  static uint8_t flterSignal[10] = {1};
  static int indexFilter = 0;
  
  flterSignal[indexFilter++] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
  if (indexFilter > 9) 
    indexFilter = 0;
  
  uint8_t temp = 0;
  for (int i = 0; i < 9; i++){
    temp += (uint8_t)flterSignal[i];
  }
  
  inNoTareState = !(temp == 0);

  
  
  
  if (getTimeDiff(noSenseTime) < NO_SENSE_TIME_AFTER_TRIGGER) return;
  
  if (inNoTareState == 0) {
    if (firstExe == false) {
      firstExe = true;
    }
  }
  else {
    writeTime(&timeNoTareDetected);
    firstExe = false;
  }
  
  if (getTimeDiff(timeNoTareDetected) > TIME_TO_NO_TARE) {
    wa.waterMissDetected = true;
    writeTime(&noSenseTime);    
  }
#endif
}

// Простой расходомер, датчик отсутствия тары
void pauseOutHandler(){
#ifdef STANDART_NO_TARE_COUNTER
  static timeStr lastTime = {0};
  static uint32_t fastPulseLoseOut = 0;
  
  waterCounters.containerPause++;
  
  if (getTimeDiff(lastTime) < 500) fastPulseLoseOut++;
    else fastPulseLoseOut = 0;
  
  if (fastPulseLoseOut > FAST_PULSES_NUM_TRESHOLD){
    wa.waterMissDetected = true;
  }
  writeTime(&lastTime);
#endif
}

// Нестандартный датчик полного контейнера
void checkContainerFull() {
#ifdef NON_STANDART_FULL_CONTAINER_COUNTER
  static timeStr timeNoTareDetected = {0};
  static timeStr noSenseTime = {0};
  static bool firstExe = false;
  
  static bool inNoTareState = 0; 
  static uint8_t flterSignal[10] = {1};
  static int indexFilter = 0;
  
  flterSignal[indexFilter++] = HAL_GPIO_ReadPin(NINT_IN20_GPIO_Port, NINT_IN20_Pin);
  if (indexFilter > 9) 
    indexFilter = 0;
  
  uint8_t temp = 0;
  for (int i = 0; i < 9; i++){
    temp += (uint8_t)flterSignal[i];
  }
  
  inNoTareState = !(temp == 0);
  
  if (getTimeDiff(noSenseTime) < NO_SENSE_TIME_AFTER_TRIGGER) return;
  
  if (inNoTareState == 0) {
    if (firstExe == false) {
      firstExe = true;
    }
  }
  else {
    writeTime(&timeNoTareDetected);
    firstExe = false;
  }
  
  if (getTimeDiff(timeNoTareDetected) > TIME_TO_NO_TARE) {
    writeTime(&noSenseTime);    
    wa.container = FULL;
    waterCounters.containerIn += valFor10LitInCalibr/10 * (maxContainerVolume - ((cnt.milLitContIn - cnt.milLitWentOut - cnt.milLitloseCounter) / 1000));
    cnt.milLitContIn = (uint32_t)((double)waterCounters.containerIn * (10000.0)/(double)valFor10LitInCalibr); 
    wa.currentContainerVolume = (cnt.milLitContIn - cnt.milLitWentOut - cnt.milLitloseCounter) / 1000;
  }
#endif
}

// Простой расходомер, датчик перелива из емкости
void countLoseHandler() {
#ifdef STANDART_FULL_CONTAINER_COUNTER

  static timeStr lastTime = {0};
  static uint32_t fastPulseLose = 0;
  
  waterCounters.containerLose++;
  cnt.milLitloseCounter = (uint32_t)((double)waterCounters.containerLose * (10000.0)/(double)valFor10LitInCalibr); 

  if (getTimeDiff(lastTime) < 1000) fastPulseLose++;
  else fastPulseLose = 0;
  writeTime(&lastTime);
  
  if (fastPulseLose > FAST_PULSES_NUM_TRESHOLD) {
    wa.container = FULL;                                                        // we know container is full
    waterCounters.containerIn += valFor10LitInCalibr/10 * (maxContainerVolume - ((cnt.milLitContIn - cnt.milLitWentOut - cnt.milLitloseCounter) / 1000));
    cnt.milLitContIn = (uint32_t)((double)waterCounters.containerIn * (10000.0)/(double)valFor10LitInCalibr); 
    wa.currentContainerVolume = (cnt.milLitContIn - cnt.milLitWentOut - cnt.milLitloseCounter) / 1000;
  }
#endif 
}

// Простой расходомер на потребителя 
void countOutHandler(){
  waterCounters.containerOut++;
  cnt.milLitWentOut = (uint32_t)((double)waterCounters.containerOut * (10000.0)/(double)valFor10LitOutCalibr); 
}


// 10 литровый расходомер на выходе у потребителя
void checkOut10Counter(){
  static uint8_t inDataOUT[10];
  static uint8_t p = 0, swchr = 0;
  
  static uint8_t temp = 0;              // to act once every 5ms
  if (temp++ < 5) return;
  temp = 0;
  
  inDataOUT[p++] = READ_10L_OUT();
  if (p > 9) p = 0;
  
  static uint32_t out101 = 0, out102 = 0;
  
  static bool firstExecute = true, newValForCalib = false;
  if ((firstExecute == true || swchr == 1) && !(inDataOUT[0]+inDataOUT[1]+inDataOUT[2]+inDataOUT[3]+inDataOUT[4]+inDataOUT[5]+inDataOUT[6]+inDataOUT[7]+inDataOUT[8]+inDataOUT[9])) {
    swchr = 0;
    if (firstExecute == false) {
      cnt.out10Counter++;
      out101 = waterCounters.containerOut;
      newValForCalib = true;
    }
    firstExecute = false;
  }
  if ((firstExecute == true || swchr == 0) && (inDataOUT[0]+inDataOUT[1]+inDataOUT[2]+inDataOUT[3]+inDataOUT[4]+inDataOUT[5]+inDataOUT[6]+inDataOUT[7]+inDataOUT[8]+inDataOUT[9]) == 10) {
    swchr = 1;
    if (firstExecute == false) {
      cnt.out10Counter++;
      out102 = waterCounters.containerOut;
      newValForCalib = true;
    }
    firstExecute = false;
  }

  static uint32_t firOut[FIR_TO_CALIB_SIZE] = {0};
  static uint8_t i = 0;  
  if (newValForCalib == true) {
    newValForCalib = false;
    if (out101 != 0 && out102 != 0) {
      firOut[i++] = (out101>out102) ? out101 - out102: out102 - out101;
      if (i >= FIR_TO_CALIB_SIZE) {
        i = 0;
        if (abs(firOut[0] - firOut[2]) < 100 && abs(firOut[1] - firOut[3]) < 100) {
          uint32_t temp = 0;
          for (uint8_t j = 0; j < FIR_TO_CALIB_SIZE; j++) temp += firOut[j];
          temp /= (FIR_TO_CALIB_SIZE / 2);
//          valFor10LitOutCalibr = temp;
        }        
      }
    }
  }
}

// 10 литровый расходомер на входе системы
void checkInput10Counter(){
  static uint8_t inDataIN[10];
  static uint8_t p = 0, swchr = 0;
  
  static uint8_t temp = 0;
  if (temp++ < 5) return;
  temp = 0;
  
  //inDataIN[p++] = READ_10L_IN();
  if (p > 9) p = 0;
  
  static uint32_t in101 = 0, in102 = 0;
  
  static bool firstExecute = true;
  static bool newValForCalib = false;
  if ((firstExecute == true || swchr == 1) && !(inDataIN[0]+inDataIN[1]+inDataIN[2]+inDataIN[3]+inDataIN[4]+inDataIN[5]+inDataIN[6]+inDataIN[7]+inDataIN[8]+inDataIN[9])) {
    swchr = 0;
    if (firstExecute == false) {
      cnt.input10Counter++;
      in101 = waterCounters.containerIn;
      newValForCalib = true;
    }
    firstExecute = false;
  }
  if ((firstExecute == true || swchr == 0) && (inDataIN[0]+inDataIN[1]+inDataIN[2]+inDataIN[3]+inDataIN[4]+inDataIN[5]+inDataIN[6]+inDataIN[7]+inDataIN[8]+inDataIN[9]) == 10) {
    swchr = 1;
    if (firstExecute == false) {
      cnt.input10Counter++;
      in102 = waterCounters.containerIn;
      newValForCalib = true;
    }
    firstExecute = false;
  }

  static int32_t firInput[FIR_TO_CALIB_SIZE] = {0};
  static uint8_t i = 0;  
  if (newValForCalib == true) {
    newValForCalib = false;
    if (in101 != 0 && in102 != 0) {
      firInput[i++] = (in101>in102) ? in101 - in102: in102 - in101;
      if (i >= FIR_TO_CALIB_SIZE) {
        i = 0;
        if (abs(firInput[0] - firInput[2]) < 100 && abs(firInput[1] - firInput[3]) < 100) {
          uint32_t temp = 0;
          for (uint8_t j = 0; j < FIR_TO_CALIB_SIZE; j++) temp += firInput[j];
          temp /= (FIR_TO_CALIB_SIZE / 2);
//          valFor10LitInCalibr = temp;
        }
      }
    }
  }
}

lghts wmLghts = {0};

void setRed(uint8_t r) {
  if (r > 10) r = 10;
  wmLghts.r = r;
}

void setBlue(uint8_t b) {
  if (b > 10) b = 10;
  wmLghts.b = b;
}

void setGreen(uint8_t g) {
  if (g > 10) g = 10;
  wmLghts.g = g;
}

void setGlobal(uint8_t l) {
  if (l > 10) l = 10;
  wmLghts.global = l;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  static uint16_t lastPinNum = 0;
  static uint16_t c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11;
  
  //static timeStr timePulseDetected = {0};
  //if (GPIO_Pin == lastPinNum && getTimeDiff(timePulseDetected) < 2) return;  
    
  if (getTimeDiff(sensorsDisabledTime) > TIME_TO_DISABLE_SENSORS) {
    
    if (GPIO_Pin == 256) {
                                                // 17
      c1++;
    }                  
    if (GPIO_Pin == 4) {
      countOutHandler();                        // 18
      c2++;                                     
    }                   
    if (GPIO_Pin == 8) {
      countLoseHandler();                       // 19
      c3++;
    }                  
    if (GPIO_Pin == 16) {
      pauseOutHandler();                        // 20
      c4++;
    }                 
    
    if (GPIO_Pin == 32) {                       
      countContainerHandler();                  // 21
      c5++;
    }
    
    if (GPIO_Pin == 1) {                        // 16 || 17, PE0, R13
      c6++;
    }
    if (GPIO_Pin == 128) {                      // PB9, R15
      c7++;
    }
    if (GPIO_Pin == 512) {                      // PB8, R19
      c8++;
    }
    
    if (GPIO_Pin == 1024) {c9++;}
    if (GPIO_Pin == 2048) {c10++;}
    if (GPIO_Pin == 4096) {c11++;}
  }
  
  if (getTimeDiff(buttonsDisabledTime) > TIME_TO_DISABLE_BUTTONS) {  
    if (GPIO_Pin == 2) {                        // R11
      static timeStr lastTimeButton = {0};
      if (getTimeDiff(lastTimeButton) > 500) {
        delayMilliseconds(5);
        if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1))return;        // debounce
        writeTime(&lastTimeButton);
        userButton = true; 
        static uint8_t usrBtCnt = 0;
        usrBtCnt++;
      }      
    }
    if (GPIO_Pin == 8192) {                     // S1
      static timeStr lastTimeButton = {0};
      if (getTimeDiff(lastTimeButton) > 200) {
        writeTime(&lastTimeButton);
        servUpButton = true; 
      }
    }
    if (GPIO_Pin == 16384) {                    // S2
      static timeStr lastTimeButton = {0};
      if (getTimeDiff(lastTimeButton) > 200) {
        writeTime(&lastTimeButton);
        servRightButton = true; 
      }
    }
    if (GPIO_Pin == 32768) {                    // S3
      static timeStr lastTimeButton = {0};
      if (getTimeDiff(lastTimeButton) > 200) {
        writeTime(&lastTimeButton);
        servDownButton = true; 
      }
    }
    if (GPIO_Pin == 64) {                       // S4
      static timeStr lastTimeButton = {0};
      if (getTimeDiff(lastTimeButton) > 200) {
        writeTime(&lastTimeButton);
        servLeftButton = true; 
      }
    }
  }
  lastPinNum = GPIO_Pin;
}


