/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define INT_IN4_Pin GPIO_PIN_2
#define INT_IN4_GPIO_Port GPIOE
#define INT_IN4_EXTI_IRQn EXTI2_IRQn
#define INT_IN3_Pin GPIO_PIN_3
#define INT_IN3_GPIO_Port GPIOE
#define INT_IN3_EXTI_IRQn EXTI3_IRQn
#define INT_IN2_Pin GPIO_PIN_4
#define INT_IN2_GPIO_Port GPIOE
#define INT_IN2_EXTI_IRQn EXTI4_IRQn
#define INT_IN1_Pin GPIO_PIN_5
#define INT_IN1_GPIO_Port GPIOE
#define INT_IN1_EXTI_IRQn EXTI9_5_IRQn
#define SERV_BUT4_Pin GPIO_PIN_6
#define SERV_BUT4_GPIO_Port GPIOE
#define SERV_BUT4_EXTI_IRQn EXTI9_5_IRQn
#define SERV_BUT3_Pin GPIO_PIN_13
#define SERV_BUT3_GPIO_Port GPIOC
#define SERV_BUT3_EXTI_IRQn EXTI15_10_IRQn
#define SERV_BUT2_Pin GPIO_PIN_14
#define SERV_BUT2_GPIO_Port GPIOC
#define SERV_BUT2_EXTI_IRQn EXTI15_10_IRQn
#define SERV_BUT1_Pin GPIO_PIN_15
#define SERV_BUT1_GPIO_Port GPIOC
#define SERV_BUT1_EXTI_IRQn EXTI15_10_IRQn
#define LCD_D7_Pin GPIO_PIN_0
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_1
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_2
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_3
#define LCD_D4_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_0
#define LCD_E_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOA
#define AIN_10V_Pin GPIO_PIN_4
#define AIN_10V_GPIO_Port GPIOA
#define AIN_4_20mA_Pin GPIO_PIN_5
#define AIN_4_20mA_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_4
#define OUT3_GPIO_Port GPIOC
#define OUT4_Pin GPIO_PIN_5
#define OUT4_GPIO_Port GPIOC
#define OUT5_Pin GPIO_PIN_0
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_1
#define OUT6_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_2
#define OUT7_GPIO_Port GPIOB
#define OUT8_Pin GPIO_PIN_7
#define OUT8_GPIO_Port GPIOE
#define OUT9_Pin GPIO_PIN_8
#define OUT9_GPIO_Port GPIOE
#define OUT10_Pin GPIO_PIN_9
#define OUT10_GPIO_Port GPIOE
#define OUT11_Pin GPIO_PIN_10
#define OUT11_GPIO_Port GPIOE
#define OUT12_Pin GPIO_PIN_11
#define OUT12_GPIO_Port GPIOE
#define OUT13_Pin GPIO_PIN_12
#define OUT13_GPIO_Port GPIOE
#define OUT14_Pin GPIO_PIN_13
#define OUT14_GPIO_Port GPIOE
#define MOSFET1_Pin GPIO_PIN_14
#define MOSFET1_GPIO_Port GPIOE
#define MOSFET2_Pin GPIO_PIN_15
#define MOSFET2_GPIO_Port GPIOE
#define NINT_TEMP3_Pin GPIO_PIN_12
#define NINT_TEMP3_GPIO_Port GPIOB
#define GPIO7_Pin GPIO_PIN_8
#define GPIO7_GPIO_Port GPIOD
#define GPIO6_Pin GPIO_PIN_9
#define GPIO6_GPIO_Port GPIOD
#define GPIO5_Pin GPIO_PIN_10
#define GPIO5_GPIO_Port GPIOD
#define GPIO4_Pin GPIO_PIN_11
#define GPIO4_GPIO_Port GPIOD
#define GPIO3_Pin GPIO_PIN_12
#define GPIO3_GPIO_Port GPIOD
#define GPIO2_Pin GPIO_PIN_13
#define GPIO2_GPIO_Port GPIOD
#define GPIO1_Pin GPIO_PIN_14
#define GPIO1_GPIO_Port GPIOD
#define PWR6_Pin GPIO_PIN_15
#define PWR6_GPIO_Port GPIOD
#define PWR5_Pin GPIO_PIN_6
#define PWR5_GPIO_Port GPIOC
#define PWR4_Pin GPIO_PIN_7
#define PWR4_GPIO_Port GPIOC
#define PWR3_Pin GPIO_PIN_8
#define PWR3_GPIO_Port GPIOC
#define PWR2_Pin GPIO_PIN_9
#define PWR2_GPIO_Port GPIOC
#define PWR1_Pin GPIO_PIN_8
#define PWR1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define W26_DO_Pin GPIO_PIN_15
#define W26_DO_GPIO_Port GPIOA
#define W26_DI_Pin GPIO_PIN_10
#define W26_DI_GPIO_Port GPIOC
#define W26_DI_EXTI_IRQn EXTI15_10_IRQn
#define INT_TEMP2_Pin GPIO_PIN_11
#define INT_TEMP2_GPIO_Port GPIOC
#define INT_TEMP2_EXTI_IRQn EXTI15_10_IRQn
#define INT_TEMP1_Pin GPIO_PIN_12
#define INT_TEMP1_GPIO_Port GPIOC
#define INT_TEMP1_EXTI_IRQn EXTI15_10_IRQn
#define NINT_IN21_Pin GPIO_PIN_0
#define NINT_IN21_GPIO_Port GPIOD
#define NINT_IN20_Pin GPIO_PIN_1
#define NINT_IN20_GPIO_Port GPIOD
#define NINT_IN19_Pin GPIO_PIN_2
#define NINT_IN19_GPIO_Port GPIOD
#define NINT_IN18_Pin GPIO_PIN_3
#define NINT_IN18_GPIO_Port GPIOD
#define NINT_IN17_Pin GPIO_PIN_4
#define NINT_IN17_GPIO_Port GPIOD
#define NINT_IN16_Pin GPIO_PIN_5
#define NINT_IN16_GPIO_Port GPIOD
#define NINT_IN15_Pin GPIO_PIN_6
#define NINT_IN15_GPIO_Port GPIOD
#define NINT_IN14_Pin GPIO_PIN_7
#define NINT_IN14_GPIO_Port GPIOD
#define NINT_IN13_Pin GPIO_PIN_3
#define NINT_IN13_GPIO_Port GPIOB
#define NINT_IN12_Pin GPIO_PIN_4
#define NINT_IN12_GPIO_Port GPIOB
#define NINT_IN11_Pin GPIO_PIN_5
#define NINT_IN11_GPIO_Port GPIOB
#define NINT_IN10_Pin GPIO_PIN_6
#define NINT_IN10_GPIO_Port GPIOB
#define INT_IN9_Pin GPIO_PIN_7
#define INT_IN9_GPIO_Port GPIOB
#define INT_IN9_EXTI_IRQn EXTI9_5_IRQn
#define INT_IN8_Pin GPIO_PIN_8
#define INT_IN8_GPIO_Port GPIOB
#define INT_IN8_EXTI_IRQn EXTI9_5_IRQn
#define INT_IN7_Pin GPIO_PIN_9
#define INT_IN7_GPIO_Port GPIOB
#define INT_IN7_EXTI_IRQn EXTI9_5_IRQn
#define INT_IN6_Pin GPIO_PIN_0
#define INT_IN6_GPIO_Port GPIOE
#define INT_IN6_EXTI_IRQn EXTI0_IRQn
#define INT_IN5_Pin GPIO_PIN_1
#define INT_IN5_GPIO_Port GPIOE
#define INT_IN5_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

#include "stdbool.h"

//#define STANDART_FULL_CONTAINER_COUNTER                         // comment this line if non standart FULL_CONTAINER sensor is using
  #ifndef STANDART_FULL_CONTAINER_COUNTER      
  #define NON_STANDART_FULL_CONTAINER_COUNTER
#endif

#define STANDART_NO_TARE_COUNTER                                // comment this line if non standart NO_TARE sensor is using
  #ifndef STANDART_NO_TARE_COUNTER      
  #define NON_STANDART_NO_TARE_COUNTER
#endif
    
#if defined(NON_STANDART_NO_TARE_COUNTER) || defined(NON_STANDART_FULL_CONTAINER_COUNTER)
  #define TIME_TO_NO_TARE                 200             // time treshold to activate event in ms
  #define NO_SENSE_TIME_AFTER_TRIGGER     100            
#endif

//#define DEBUG_PCB_MODE
#define DEBUG_TIME_BETWEEN              500
#define TIME_BETWEEN_PORT_CHANGES       100

//#define valFor10LitCalibr               5500 
#define LOW_TH_VOL_SENS                 4000
#define HIGH_TH_VOL_SENS                7000
#define FAST_PULSES_NUM_TRESHOLD        3
//#define TIME_TO_STOP_CONSUM_PUMP_IF_NO_WATER    10*2
    
#define RPI_BUFFER_SIZE         256

typedef struct {
  uint32_t totalPaid;
  volatile uint32_t sessionPaid;
  float leftFromPaid;
} moneyStats;

typedef struct {
  uint32_t resource;
  uint32_t curValue;
} filter;

typedef enum {
  NOT_READY,
  WASH_FILTER,
  WAIT,
  JUST_PAID,
  WORK,
  NO_TARE,
  SERVICE,
  FREE,
  CONFIG
} state;

typedef enum {
  ZERO,
  THREE,
  FIVE,
  FIFTEEN,
  THIRTY,
  SIXTY
} noTareStageEnum;

typedef enum {
  HI_PRESSURE, 
  NO_PRESSURE
} waterPressureEnum;

typedef enum {
  DETECTED, 
  OK
} tumperEnum;

typedef enum {
  NORMAL_WORK, 
  SERVICING
} serviceEnum;

typedef enum {
  STOPPED,
  WORKING
} pumpEnum;

typedef enum {
  OPENED,
  CLOSED
} valveEnum;

typedef enum {
  FULL,
  NOT_FULL
} containerEnum;

typedef enum {
  FREE_MODE,
  NOT_FREE
} freeEnum;

typedef enum {
  INHIBIT_BILL,
  ENABLE_BILL
} billAccept;

typedef enum {
  TURNED_OFF,
  TURNED_ON
} tempMgmnt;

typedef struct {
  state machineState, lastMachineState;         // автомат состояний
  containerEnum container;                      // общая емкость котейнера
  uint32_t litersLeftFromSession;               // осталось литров выдать клиенту в текущей сессии
  uint32_t currentContainerVolume;              // количество воды в контейнере сейчас
  uint16_t suppVoltage;                         // напряжение питания
  uint16_t tempMCU;                             // температура МК
  billAccept billAccept;                        // запрещено/разрешено принимать деньги
  bool waterMissDetected;                       // true, если обнаружена утечка воды при выдаче
  tempMgmnt warmer;
  tempMgmnt cooler;
  
  valveEnum mainValve;                          // состояние магистрального клапана
  valveEnum filterValve;                        // состояние клапана фильтров
  valveEnum washFilValve;                       // состояние промывосного клапана
  pumpEnum consumerPump;                        // состояние выходного насоса  
  pumpEnum mainPump;                            // состояние магистрального насоса
  
  waterPressureEnum magistralPressure;          // есть ли давление в магистрали
  tumperEnum tumperMoney;                       // состояние тампера по деньгам
  tumperEnum tumperDoor;                        // состояние тампера двери
  freeEnum free;                                // находится ли автомат а режиме бесплатного обслуживания
} machineParameters;

typedef struct {
  volatile uint32_t input10Counter;             // входной 10 литровый счетчик
  volatile uint32_t out10Counter;               // выходной 10 литровой счетчик
  volatile uint32_t milLitloseCounter;          // перелив чистой воды
  volatile uint32_t milLitWentOut;              // общий расход воды с момента включения
  volatile uint32_t milLitContIn;               // общий приток воды в контейнер
} counters;

typedef struct {
  filter meshFil;                               // сетчатый фильтр 100мкр.
  filter mechFil10;                             // механический фильтр 10мкр.
  filter mechFil5;                              // механический фильтр 5мкр.
  filter coalFil1;                              // угольный фильтр 1
  filter coalFil2;                              // угольный фильтр 2
  filter afterFil;                              // постфильтр 10мкр.
  filter backOsm1;                              // обратный осмос 1
  filter backOsm2;                              // обратный осмос 2
  filter backOsm3;                              // обратный осмос 3
} filtersStr;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t global;
} lghts;

extern void delayMicroseconds(uint32_t );       // assembler delay.s
extern void delayMilliseconds(uint32_t );       // assembler delay.s
void prepareToTransition();


/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
