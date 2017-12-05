#ifndef OneWire_h
#define OneWire_h

#include "main.h"
#include "stm32f1xx_hal.h"

void ds18b20Init(void);
void swiStartMeasrmnt();
void swiReadTemp(int16_t * );

//// tempsensor definitions
#define OutTpPort               NINT_IN8_GPIO_Port
#define OutTpPin                NINT_IN8_Pin
#define MotorTpPort             NINT_IN9_GPIO_Port
#define MotorTpPin              NINT_IN9_Pin
#define StreetTpPort            NINT_IN10_GPIO_Port
#define StreetTpPin             NINT_IN10_Pin
#define inWaterTpPort           NINT_TEMP2_GPIO_Port
#define inWaterTpPin            NINT_TEMP2_Pin
#define cntWaterTpPort          NINT_TEMP3_GPIO_Port
#define cntWaterTpPin           NINT_TEMP3_Pin


#endif