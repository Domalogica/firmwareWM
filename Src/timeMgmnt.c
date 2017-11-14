#include "timeMgmnt.h"

timeStr time; 

void timeMgmnt(void) {
  time.msec++;
  if (time.msec > 999) {
    time.msec = 0;
    time.sec++;
  }
  if (time.sec > 59) {
    time.sec = 0;
    time.min++;
  }
  if (time.min > 59) {
    time.min = 0;
    time.hours++;
  }
}

timeStr getCurTime() {
  return time;
}

uint32_t getTimeDiff(timeStr timeVal) {
  uint32_t mil = (((uint64_t)time.hours) * 60*60*1000 + time.min * 60*1000 + time.sec * 1000 + time.msec) - 
                  ((uint64_t)timeVal.hours * 60*60*1000 + timeVal.min * 60*1000 + timeVal.sec * 1000 + timeVal.msec);
  return mil; 
}

void writeTime(timeStr * timeVal) {
  (*timeVal).hours = time.hours;
  (*timeVal).min   = time.min;
  (*timeVal).sec   = time.sec;
  (*timeVal).msec  = time.msec;  
}
