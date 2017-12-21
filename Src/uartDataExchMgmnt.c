#include "stm32f1xx_hal.h"
#include <stdlib.h>     /* strtol */
#include <string.h>     /* strtol */

#include "main.h"
#include "uartDataExchMgmnt.h"
#include "portsMgmnt.h"

bool checkNewData(void);
void parseUART(void);

extern UART_HandleTypeDef huart1;
extern machineParameters wa;
extern filtersStr filters;
extern moneyStats money;
extern counters cnt;

extern float waterPrice ;                               // цена литра
extern uint8_t outPumpNoWaterStopTime;                  // секунд до остановки выходного насоса, если нет воды
extern uint8_t containerMinVolume;                      // минимальный объем воды в контейнере
extern uint8_t maxContainerVolume;                      // объем контейнера с водой
extern uint32_t valFor10LitOutCalibr;

static uint8_t uartDataTx[RPI_BUFFER_SIZE] = {0};
uint8_t uartDataRx[RPI_BUFFER_SIZE] = {0};
uint8_t mesRx[RPI_BUFFER_SIZE] = {0};
uint8_t entryPointer = 255;
uint8_t endPointer = 0;

void uartDataExchMgmnt(void) {
//  __disable_interrupt();        // критический участок кода
  if (checkNewData() == true) {
    parseUART();
  }
//  __enable_interrupt();
}

inline void getLastMessage () {
  uint8_t i = entryPointer+1, j = 0;
  while (i != endPointer+1) mesRx[j++] = uartDataRx[i++];          // create array with input parameters
}

inline void clrLastMessage () {
  memset(&mesRx[0], 0, RPI_BUFFER_SIZE);
  //entryPointer = endPointer;
  entryPointer = 255;
  endPointer = 0;
}


uint8_t getMessageLnhgt () {
  if (endPointer >= entryPointer) return endPointer - entryPointer;
  else return (RPI_BUFFER_SIZE - entryPointer) + endPointer;
}

uint8_t writeError (uint8_t errCode) {
  if (errCode == 0) {
    uartDataTx[0] = '0';
    uartDataTx[1] = '\n';
    return 2;
  }
  if (errCode == 1) {
    uartDataTx[0] = '-';
    uartDataTx[1] = '1';
    uartDataTx[2] = '\n';
    return 3;
  }
  if (errCode == 2) {
    uartDataTx[0] = '-';
    uartDataTx[1] = '2';
    uartDataTx[2] = '\n';
    return 3;
  }
  if (errCode == 3) {
    uartDataTx[0] = '-';
    uartDataTx[1] = '3';
    uartDataTx[2] = '\n';
    return 3;
  }
  if (errCode == 4) {
    uartDataTx[0] = '-';
    uartDataTx[1] = '4';
    uartDataTx[2] = '\n';
    return 3;
  }
  return 0;
}

// answer for 'u' response 
uint16_t insertUID (uint8_t * uartTxBuf) {
  uint16_t byteCounter = 0;
  uint32_t uid[3];
  HAL_GetUID(&uid[0]); 
  *(uartTxBuf + byteCounter) = '"';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "0x%08x", uid[0]);
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%08x", uid[1]);
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%08x", uid[2]);
  *(uartTxBuf + byteCounter) = '"';  byteCounter++;      

  *(uartTxBuf + byteCounter) = '\n'; byteCounter++;      
  return byteCounter;
}

// create answer for 'g' response
uint16_t insertStats (uint8_t * uartTxBuf) {
  uint16_t byteCounter = 0;
  *(uartTxBuf + byteCounter) = '[';
  byteCounter++;                        // 1 byte
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", cnt.input10Counter);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", cnt.out10Counter);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", cnt.milLitloseCounter);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", cnt.milLitWentOut);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", cnt.milLitContIn);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", (int)(waterPrice));
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", containerMinVolume);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", maxContainerVolume);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", money.totalPaid);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
    byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", money.tatalHardCash);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", money.hardCash);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", money.hardMoney);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", money.sessionPaid);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", (int)(money.leftFromPaid));
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.machineState);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.container);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.currentContainerVolume);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.consumerPump);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.mainPump);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.magistralPressure);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.mainValve);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.filterValve);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.washFilValve);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.warmer);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.tumperMoney);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;

  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.tumperDoor);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.waterMissDetected);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.free);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.suppVoltage);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.billAccept);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      

  uint8_t everythingOKbit = 1;
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", everythingOKbit);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", valFor10LitOutCalibr);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      

  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.tempMCU);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.boardTp);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.OutTp);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.StreetTp);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.inWaterTp);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.cntWaterTp);
  *(uartTxBuf + byteCounter) = ',';  byteCounter++;      
  byteCounter += sprintf((char*)(uartTxBuf + byteCounter), "%d", wa.MotorTp);
  
  *(uartTxBuf + byteCounter) = ']';  byteCounter++;  
  *(uartTxBuf + byteCounter) = '\n'; byteCounter++;      
  return byteCounter;
}

uint16_t returnMoney (uint8_t * uartTxBuf) {
  uint16_t byteCounter = sprintf((char*)(uartTxBuf), "%d", (uint32_t) money.leftFromPaid);  
  *(uartTxBuf + byteCounter) = '\n'; byteCounter++;      
  return byteCounter;
}

void setParameters(char const * uartRXBuf) {
  char * pEnd;
  waterPrice = (float)strtol(uartRXBuf, &pEnd, 10) / 100.0;
  pEnd++;
  containerMinVolume = strtol(pEnd, &pEnd, 10);
  pEnd++;
  maxContainerVolume = strtol(pEnd, NULL, 10);
}

void setFilterValues(char const * uartRXBuf) {
  char * pEnd;
  filters.meshFil.curValue   = strtol(uartRXBuf, &pEnd, 10);
  pEnd++;
  filters.mechFil10.curValue = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.mechFil5.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.coalFil1.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.coalFil2.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.afterFil.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.backOsm1.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.backOsm2.curValue  = strtol(pEnd, &pEnd, 10);
  pEnd++;
  filters.backOsm3.curValue  = strtol(pEnd, NULL, 10);
}

void getMoney(char const * uartRXBuf) {
  money.sessionPaid += (float)strtol(uartRXBuf, NULL, 10);
  money.leftFromPaid += strtol(uartRXBuf, NULL, 10);
}

bool checkNewData() {
  uint8_t curPos = huart1.RxXferSize - huart1.RxXferCount - 1;  
  if (uartDataRx[curPos] == '\n' && endPointer != (curPos)) {
    endPointer = curPos;
    return true;
  }
  return false;
}

void initUART(void) {
  huart1.pRxBuffPtr = &uartDataRx[0];
  huart1.pTxBuffPtr = &uartDataTx[0];
  
  HAL_UART_Receive_IT(&huart1, &uartDataRx[0], RPI_BUFFER_SIZE);
}


uint16_t parseUartData(uint8_t * uartTXBuf) {
  uint8_t txByteCounter = 0;
  getLastMessage();
  HAL_UART_AbortReceive(&huart1);
  HAL_UART_Receive_IT(&huart1, &uartDataRx[0], 256);
  if (mesRx[0] == 'g') {
    if (getMessageLnhgt() > 2) txByteCounter += writeError(3);
    if (getMessageLnhgt() < 2) txByteCounter += writeError(2);
    if (getMessageLnhgt() == 2) {      
      txByteCounter += writeError(0);
      txByteCounter += insertStats(&uartTXBuf[2]);
    }
    return txByteCounter;
  }

  if (mesRx[0] == 'u') {
    if (getMessageLnhgt() > 2) txByteCounter += writeError(3);
    if (getMessageLnhgt() < 2) txByteCounter += writeError(2);
    if (getMessageLnhgt() == 2) {      
      txByteCounter += writeError(0);
      txByteCounter += insertUID(&uartTXBuf[2]);
    }
    return txByteCounter;
  }
  
  if (mesRx[0] == 'c') {
    if (mesRx[1] == 's') {
      txByteCounter += writeError(0);
      setParameters((char const*)&mesRx[2]);
    }
    if (mesRx[1] == 'f') {
      txByteCounter += writeError(0);
      setFilterValues((char const*)&mesRx[2]);
    }
    if (mesRx[1] == 'w') {  
      if (getMessageLnhgt() > 3) txByteCounter += writeError(3);
      if (getMessageLnhgt() < 3) txByteCounter += writeError(2);
      if (getMessageLnhgt() == 3) {      
        txByteCounter += writeError(0);
        // TO DO - start filter wash
      }
    }
    if (mesRx[1] == 'r') {  
      if (getMessageLnhgt() > 3) txByteCounter += writeError(3);
      if (getMessageLnhgt() < 3) txByteCounter += writeError(2);
      if (getMessageLnhgt() == 3) {      
        txByteCounter += writeError(0);
        // TO DO - self reset
      }
    }
    if (mesRx[1] == 'n') {  
      if (getMessageLnhgt() > 3) txByteCounter += writeError(3);
      if (getMessageLnhgt() < 3) txByteCounter += writeError(2);
      if (getMessageLnhgt() == 3) {      
        txByteCounter += writeError(0);
        wa.machineState = WAIT; 
      }
    }
    if (mesRx[1] == 'i') {                 // inhibit
      if (wa.machineState == WAIT) {
        txByteCounter += writeError(0);
        INHIBIT_DIS();
      }
      else txByteCounter += writeError(4);
    }
    if (mesRx[1] == 'j') {  
      txByteCounter += writeError(0);
      INHIBIT_EN();
    }
    if (mesRx[1] == 'm') {
      if (getMessageLnhgt() > 3)
        txByteCounter += writeError(3);
      
      if (getMessageLnhgt() < 3)
        txByteCounter += writeError(2);
      if (getMessageLnhgt() == 3)
      {      
        txByteCounter += returnMoney(&uartTXBuf[0]);
        //money.totalPaid += money.sessionPaid - money.leftFromPaid
        wa.machineState = WAIT;
        prepareToTransition();
        money.totalPaid += money.sessionPaid - (uint32_t)money.leftFromPaid;
        money.sessionPaid = 0;
        money.leftFromPaid = 0.0;
      }
    }
    if (mesRx[1] == 't')
    {  
      txByteCounter += writeError(0);
      // TO DO - show data on LCD
    }
    return txByteCounter;
  }
  if (mesRx[0] == 'm') {
    txByteCounter += writeError(0);
    getMoney((char const *)&mesRx[1]);
    return txByteCounter;
  } 
  
  txByteCounter += writeError(1);
  return 0;
}
  
void parseUART() {
  HAL_UART_Transmit_IT(&huart1, &uartDataTx[0], parseUartData(&uartDataTx[0]));
  clrLastMessage();
}


