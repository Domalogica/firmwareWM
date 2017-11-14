#ifndef TIME_MGMNT_H_
#define TIME_MGMNT_H_

#include "stdint.h"

typedef struct {
  uint32_t hours;
  uint8_t min;
  uint8_t sec;
  uint16_t msec;
} timeStr; 

// возвращает  текущее время
timeStr getCurTime();

// зпускается обработчиком прерывания TIM3 для подсчета времени каждую миллисекунду
void timeMgmnt(void);

// получает разницу в миллисекундах timeVal относительно текущего времени
uint32_t getTimeDiff(timeStr timeVal);

// записать в переменную timeVal текщее время
void writeTime(timeStr * timeVal);

#endif