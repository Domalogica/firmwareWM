#ifndef PORTS_MGMNT_H_
#define PORTS_MGMNT_H_

#include "main.h"
#include "stdbool.h"

// 1. Для управления главным насосом
// 2. Для управления клапаном магистральным
// 3. Для управления насосом выдачи
// 4. Для управления клапаном промывочным
// 5. Для управления обогревом
// 6. Для управления охлаждением

#define OUT1SET()               HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET)
#define OUT1CLR()               HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET)
#define OUT2SET()               HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET)
#define OUT2CLR()               HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET)
#define OUT3SET()               HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET)
#define OUT3CLR()               HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET)
#define OUT4SET()               HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET)
#define OUT4CLR()               HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET)
#define OUT5SET()               HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET)
#define OUT5CLR()               HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET)
#define OUT6SET()               HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET)
#define OUT6CLR()               HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET)
#define OUT7SET()               HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET)
#define OUT7CLR()               HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET)
#define OUT8SET()               HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_SET)
#define OUT8CLR()               HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_RESET)
#define OUT9SET()               HAL_GPIO_WritePin(OUT9_GPIO_Port, OUT9_Pin, GPIO_PIN_SET)
#define OUT9CLR()               HAL_GPIO_WritePin(OUT9_GPIO_Port, OUT9_Pin, GPIO_PIN_RESET)
#define OUT10SET()              HAL_GPIO_WritePin(OUT10_GPIO_Port, OUT10_Pin, GPIO_PIN_SET)
#define OUT10CLR()              HAL_GPIO_WritePin(OUT10_GPIO_Port, OUT10_Pin, GPIO_PIN_RESET)
#define OUT11SET()              HAL_GPIO_WritePin(OUT11_GPIO_Port, OUT11_Pin, GPIO_PIN_SET)
#define OUT11CLR()              HAL_GPIO_WritePin(OUT11_GPIO_Port, OUT11_Pin, GPIO_PIN_RESET)
#define OUT12SET()              HAL_GPIO_WritePin(OUT12_GPIO_Port, OUT12_Pin, GPIO_PIN_SET)
#define OUT12CLR()              HAL_GPIO_WritePin(OUT12_GPIO_Port, OUT12_Pin, GPIO_PIN_RESET)
#define OUT13SET()              HAL_GPIO_WritePin(OUT13_GPIO_Port, OUT13_Pin, GPIO_PIN_SET)
#define OUT13CLR()              HAL_GPIO_WritePin(OUT13_GPIO_Port, OUT13_Pin, GPIO_PIN_RESET)
#define OUT14SET()              HAL_GPIO_WritePin(OUT14_GPIO_Port, OUT14_Pin, GPIO_PIN_SET)
#define OUT14CLR()              HAL_GPIO_WritePin(OUT14_GPIO_Port, OUT14_Pin, GPIO_PIN_RESET)
#define MOSFET2_ON()            HAL_GPIO_WritePin(MOSFET2_GPIO_Port, MOSFET2_Pin, GPIO_PIN_SET)
#define MOSFET2_OFF()           HAL_GPIO_WritePin(MOSFET2_GPIO_Port, MOSFET2_Pin, GPIO_PIN_RESET)

#define TURN_BUT_LED_ON()       OUT7SET()
#define TURN_BUT_LED_OFF()      OUT7CLR()

#define MAIN_VALVE_SET()        HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_SET)
#define MAIN_VALVE_CLR()        HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_RESET)
#define MAIN_PUMP_SET()         HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_SET)
#define MAIN_PUMP_CLR()         HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_RESET)
#define WASH_FILV_SET()         HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, GPIO_PIN_SET)
#define WASH_FILV_CLR()         HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, GPIO_PIN_RESET)
#define CONSUM_PUMP_SET()       HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_SET)
#define CONSUM_PUMP_CLR()       HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_RESET)
#define WARM_SET()              HAL_GPIO_WritePin(PWR5_GPIO_Port, PWR5_Pin, GPIO_PIN_SET)
#define WARM_CLR()              HAL_GPIO_WritePin(PWR5_GPIO_Port, PWR5_Pin, GPIO_PIN_RESET)
#define COLD_SET()              HAL_GPIO_WritePin(PWR6_GPIO_Port, PWR6_Pin, GPIO_PIN_SET)
#define COLD_CLR()              HAL_GPIO_WritePin(PWR6_GPIO_Port, PWR6_Pin, GPIO_PIN_RESET)

#define INHIBIT_C_SET()         HAL_GPIO_WritePin(GPIO7_GPIO_Port, GPIO7_Pin, GPIO_PIN_SET)
#define INHIBIT_C_CLR()         HAL_GPIO_WritePin(GPIO7_GPIO_Port, GPIO7_Pin, GPIO_PIN_RESET)
#define INHIBIT_M_SET()         HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_SET)
#define INHIBIT_M_CLR()         HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_RESET)

#define MAINP_ON()              do {wa.mainPump = WORKING; disableSensorsForTime(); MAIN_PUMP_SET();} while(0)       
#define MAINP_OFF()             do {wa.mainPump = STOPPED; disableSensorsForTime(); MAIN_PUMP_CLR();} while(0)       
#define CONSUMP_ON()            do {wa.consumerPump = WORKING; disableSensorsForTime(); } while(0)
#define CONSUMP_OFF()           do {wa.consumerPump = STOPPED; disableSensorsForTime(); } while(0)
#define MAINV_ON()              do {wa.mainValve = OPENED; disableSensorsForTime(); MAIN_VALVE_SET();} while(0)             
#define MAINV_OFF()             do {wa.mainValve = CLOSED; disableSensorsForTime(); MAIN_VALVE_CLR();} while(0)             
#define WASH_FILV_ON()          do {wa.washFilValve = OPENED; disableSensorsForTime(); WASH_FILV_SET();} while(0)             
#define WASH_FILV_OFF()         do {wa.washFilValve = CLOSED; disableSensorsForTime(); WASH_FILV_CLR();} while(0)       
#define WARM_ON()               do {wa.warmer = TURNED_ON; disableSensorsForTime(); WARM_SET();} while(0)             
#define WARM_OFF()              do {wa.warmer = TURNED_OFF; disableSensorsForTime(); WARM_CLR();} while(0)       
#define COLD_ON()               do {wa.cooler = TURNED_ON; disableSensorsForTime(); COLD_SET();} while(0)             
#define COLD_OFF()              do {wa.cooler = TURNED_OFF; disableSensorsForTime(); COLD_CLR();} while(0)       

#define INHIBIT_EN()            do {INHIBIT_C_SET(); INHIBIT_M_SET(); wa.billAccept = ENABLE_BILL;}     while(0)            
#define INHIBIT_DIS()           do {INHIBIT_C_CLR(); INHIBIT_M_CLR(); wa.billAccept = INHIBIT_BILL;}    while(0)   

#define READ_DOOR_TUMPER()      HAL_GPIO_ReadPin(NINT_IN15_GPIO_Port, NINT_IN15_Pin)            // 7, R1, PD6
#define READ_MONEY_TUMPER()     HAL_GPIO_ReadPin(NINT_IN16_GPIO_Port, NINT_IN16_Pin)            // 6, R2, PD5
#define READ_MAG_PRESURE()      HAL_GPIO_ReadPin(NINT_IN17_GPIO_Port, NINT_IN17_Pin)            // 5, R3, PD4
#define READ_FREE_INPUT()       HAL_GPIO_ReadPin(NINT_IN18_GPIO_Port, NINT_IN18_Pin)            // 4, R4, PD3
#define READ_10L_OUT()          HAL_GPIO_ReadPin(NINT_IN20_GPIO_Port, NINT_IN20_Pin)            // 3, R5, PD2
#define READ_10L_IN()           HAL_GPIO_ReadPin(NINT_IN19_GPIO_Port, NINT_IN19_Pin)            // 2, R6, PD1

#define R_ON()                  OUT9SET()
#define R_OFF()                 OUT9CLR()
#define G_ON()                  OUT8SET()
#define G_OFF()                 OUT8CLR()
#define B_ON()                  OUT10SET()
#define B_OFF()                 OUT10CLR()

#define COOLER_ON()             OUT4SET()
#define COOLER_OFF()            OUT4CLR()

#define GLOBAL_LGHT_ON()        MOSFET2_ON()
#define GLOBAL_LGHT_OFF()       MOSFET2_OFF()

void checkTumperDoor();
void checkMagistralPressure();
void checkTumperMoney();
void checkFreeMode();
void checkInput10Counter();
void checkOut10Counter();
void checkNoTare();
void checkContainerFull();

void clrUserButton();
void clrServUpButton();
void clrServDownButton();
void clrServLeftButton();
void clrServRightButton();

bool isUserButtonPressed();
bool isAdminUpButtonPressed();
bool isAdminDownButtonPressed();
bool isAdminLeftButtonPressed();
bool isAdminRightButtonPressed();

void setRed(uint8_t);
void setBlue(uint8_t);
void setGreen(uint8_t);
void setGlobal(uint8_t);

// assumption of container's water volume
void setupDefaultLitersVolume(uint16_t volume);
void setContainerValToZero(uint16_t maxContVolLit);


#define TIME_TO_DISABLE_SENSORS         50
#define TIME_TO_DISABLE_BUTTONS         50         
void disableSensorsForTime(void);
void disableButtonsForTime(void);


#endif