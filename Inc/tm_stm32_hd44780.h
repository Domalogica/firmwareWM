/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32Fxxx
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef TM_HD44780_H
#define TM_HD44780_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
 
  
// D_Pin        r61     E
// B_Pin        r59     RS
// C_pin        r60     D4
// DP0_Pin      r47     D5
// A_Pin        r58     D6
// E_Pin        r62     D7

#define D4_SET()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)
#define D4_CLR()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define D5_SET()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define D5_CLR()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define D6_SET()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define D6_CLR()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define D7_SET()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define D7_CLR()               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define E_SET()                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define E_CLR()                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define RS_SET()               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define RS_CLR()               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
  

  
/**
 * @defgroup TM_HD44780_Macros
 * @brief    Library defines
 * @{
 */

/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOB
#define HD44780_RS_PIN				GPIO_PIN_2
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOB
#define HD44780_E_PIN				GPIO_PIN_7
#endif
/* Data pins */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOC
#define HD44780_D4_PIN				GPIO_PIN_12
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOC
#define HD44780_D5_PIN				GPIO_PIN_13
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOB
#define HD44780_D6_PIN				GPIO_PIN_12
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOB
#define HD44780_D7_PIN				GPIO_PIN_13
#endif

/**
 * @}
 */

/**
 * @defgroup TM_HD44780_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes HD44780 LCD
 * @brief  cols: Width of lcd
 * @param  rows: Height of lcd
 * @retval None
 */
void TM_HD44780_Init(uint8_t cols, uint8_t rows, uint32_t timeToShowStartMes);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void TM_HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x: X location where string will start
 * @param  y; Y location where string will start
 * @param  *str: pointer to string to display
 * @retval None
 */
void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void TM_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  x: X location where character will be shown
 * @param  y: Y location where character will be shown
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
#include "main.h"

void reInit();

void printWait(uint32_t);

void printPaid(uint16_t , uint16_t); 

void printGiven(uint32_t, uint32_t, uint32_t, bool);

void printFinish();

void printService(uint8_t);

void printMeasure(uint16_t liters);

void printNotReady(uint32_t);

void printLoseDetected();

void printAdminUp();
void printAdminDown();
void printAdminLeft();
void printAdminRight();

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
