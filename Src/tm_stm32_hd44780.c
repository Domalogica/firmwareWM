/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_hd44780.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "math.h"
   
#define _D   0xE0
#define _B   0xA0
#define _I   0xA5
#define _Ib  0xAE
#define _P   0xA8
#define _F   0xAA
#define _L   0xA7
#define _R   0xB1
#define _G   0xA1
#define _Y   0xA9
#define _GG  0xA3

#define _t   0xBF
#define _d   0xE3
#define _m   0xBC
#define _l   0xBB
#define _b   0xB2  
#define _g   0xB4
#define _p   0xBE
#define _i   0xB8
#define _ib  0xC3
#define _k   0xBA

/* Private HD44780 structure */

/* Private functions */
static void TM_HD44780_InitPins(void);
static void TM_HD44780_Cmd(uint8_t cmd);
static void TM_HD44780_Cmd4bit(uint8_t cmd);
static void TM_HD44780_Data(uint8_t data);
static void TM_HD44780_CursorSet(uint8_t col, uint8_t row);

  
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
} HD44780_Options_t;


/* Private variable */
HD44780_Options_t HD44780_Opts;
/*const uint8_t let_g[8] = {
    0xf,0x8,0x8,0x8,0x8,0x8,0x8,0x0};           // Ã
  const uint8_t let_i[8] = {    
    0x11,0x13,0x15,0x15,0x15,0x19,0x11,0x0};    // Û
  const uint8_t let_l[8] = {
    0x4,0xa,0xa,0xa,0x11,0x11,0x11};            // Ë
  const uint8_t let_ii[8] = {                   
    0x11,0x11,0x19,0x15,0x15,0x15,0x19};        // È
  const uint8_t let_iii[8] = {
    0x18,0x8,0xe,0x9,0x9,0x9,0xe};              // Ú
  const uint8_t let_b[8] = {
    0x1e,0x10,0x10,0x1e,0x11,0x11,0x1e};        // B
  const uint8_t let_gg[8] = {   
    0x15,0x15,0xe,0x4,0xe,0x15,0x15};           // Æ
  const uint8_t let_d[8] = {
    0xe,0xa,0xa,0xa,0xa,0x1f,0x11};             // Ä
  // çàïèñûâàåì èõ â ÎÇÓ ÆÊÈ */
  

/* Pin definitions */
#define HD44780_RS_LOW              RS_CLR()
#define HD44780_RS_HIGH             RS_SET()
#define HD44780_E_LOW               E_CLR()
#define HD44780_E_HIGH              E_SET()

#define HD44780_E_BLINK             E_SET(); HD44780_Delay(60); E_CLR(); HD44780_Delay(60)
//#define HD44780_E_BLINK             E_SET(); HD44780_Delay(300); E_CLR(); HD44780_Delay(300)
#define HD44780_Delay(x)            delayMicroseconds(x)

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

extern void delayMicroseconds(uint32_t );
extern void delayMilliseconds(uint32_t );

void TM_HD44780_Init(uint8_t cols, uint8_t rows, uint32_t timeToShowStartMes) {
  
  /* Init pinout */
  TM_HD44780_InitPins();
  
  /* At least 40ms */
  HD44780_Delay(45000);
  
  /* Set LCD width and height */
  HD44780_Opts.Rows = rows;
  HD44780_Opts.Cols = cols;
  
  /* Set cursor pointer to beginning for LCD */
  HD44780_Opts.currentX = 0;
  HD44780_Opts.currentY = 0;
  
  HD44780_Opts.DisplayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
  if (rows > 1) {
          HD44780_Opts.DisplayFunction |= HD44780_2LINE;
  }
  
  /* Try to set 4bit mode */
  TM_HD44780_Cmd4bit(0x03);
  HD44780_Delay(5000);
  
  /* Second try */
  TM_HD44780_Cmd4bit(0x03);
  HD44780_Delay(5000);
  
  /* Third goo! */
  TM_HD44780_Cmd4bit(0x03);
  HD44780_Delay(5000);	
  
  /* Set 4-bit interface */
  TM_HD44780_Cmd4bit(0x02);
  HD44780_Delay(200);
  
  /* Set # lines, font size, etc. */
  TM_HD44780_Cmd(HD44780_FUNCTIONSET | HD44780_Opts.DisplayFunction);

  /* Turn the display on with no cursor or blinking default */
  HD44780_Opts.DisplayControl = HD44780_DISPLAYON;
  TM_HD44780_DisplayOn();

  /* Clear lcd */
  TM_HD44780_Clear();

  /* Default font directions */
  HD44780_Opts.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
  TM_HD44780_Cmd(HD44780_ENTRYMODESET | HD44780_Opts.DisplayMode);

  /* Delay */
  HD44780_Delay(5000);
  
  // ôîðìèðóåì ðóññêèå áóêâû
  // çàïèñûâàåì èõ â ÎÇÓ ÆÊÈ 
  char lit0[17] = {' ', ' ', ' ',  _D, 'o', _m, 'a', _l, 'o', _g, _i, _k, 'a', ' ', ' ', ' ', '\0'};
  TM_HD44780_Puts(0, 0, &lit0[0]);
  char lit1[17] = {' ', ' ', 'B', 'o', _d, 'o', _m, 'a', _t, ' ', 'v', '0', '.', '1', '1', ' ', '\0'};
  TM_HD44780_Puts(0, 1, &lit1[0]);
  delayMilliseconds(timeToShowStartMes);
  TM_HD44780_Clear();
}

void TM_HD44780_Clear(void) {
	TM_HD44780_Cmd(HD44780_CLEARDISPLAY);
	HD44780_Delay(3500);
}

void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str) {
	TM_HD44780_CursorSet(x, y);
	while (*str) {
		if (HD44780_Opts.currentX >= HD44780_Opts.Cols) {
			HD44780_Opts.currentX = 0;
			HD44780_Opts.currentY++;
			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		}
		if (*str == '\n') {
			HD44780_Opts.currentY++;
			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		} else if (*str == '\r') {
			TM_HD44780_CursorSet(0, HD44780_Opts.currentY);
		} else {
			TM_HD44780_Data(*str);
			HD44780_Opts.currentX++;
		}
		str++;
	}
}

void TM_HD44780_DisplayOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_DisplayOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_ScrollLeft(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void TM_HD44780_ScrollRight(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void TM_HD44780_CreateChar(uint8_t location, uint8_t *data) {
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	TM_HD44780_Cmd(HD44780_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		TM_HD44780_Data(data[i]);
	}
}

void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) {
	TM_HD44780_CursorSet(x, y);
	TM_HD44780_Data(location);
}

/* Private functions */
static void TM_HD44780_Cmd(uint8_t cmd) {
	/* Command mode */
	HD44780_RS_LOW;
	
	/* High nibble */
	TM_HD44780_Cmd4bit(cmd >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(cmd & 0x0F);
}

static void TM_HD44780_Data(uint8_t data) {
	/* Data mode */
	HD44780_RS_HIGH;
	
	/* High nibble */
	TM_HD44780_Cmd4bit(data >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(data & 0x0F);
}

static void TM_HD44780_Cmd4bit(uint8_t cmd) {
	/* Set output port */
        if (cmd & 0x08) D7_SET();
        else D7_CLR();
        if (cmd & 0x04) D6_SET();
        else D6_CLR();
        if (cmd & 0x02) D5_SET();
        else D5_CLR();
        if (cmd & 0x01) D4_SET();
        else D4_CLR();
        
	HD44780_E_BLINK;
}

static void TM_HD44780_CursorSet(uint8_t col, uint8_t row) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	
	/* Go to beginning */
	if (row >= HD44780_Opts.Rows) {
		row = 0;
	}
	
	/* Set current column and row */
	HD44780_Opts.currentX = col;
	HD44780_Opts.currentY = row;
	
	/* Set location address */
	TM_HD44780_Cmd(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}

static void TM_HD44780_InitPins(void) {
  D4_SET(); //               HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET)
  D4_CLR(); //              HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_RESET)
  D5_SET(); //           HAL_GPIO_WritePin(GPIOC, DP0_Pin, GPIO_PIN_SET)
  D5_CLR(); //           HAL_GPIO_WritePin(GPIOC, DP0_Pin, GPIO_PIN_RESET)
  D6_SET(); //           HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET)
  D6_CLR(); //           HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_RESET)
  D7_SET(); //           HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET)
  D7_CLR(); //           HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET)
  E_SET(); //            HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET)
  E_CLR(); //            HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_RESET)
  RS_SET(); //           HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET)
  RS_CLR(); //           HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_RESET)
}


/*void createRusChars(){
  TM_HD44780_CreateChar(0, (uint8_t*) let_g);   // Ã
  TM_HD44780_CreateChar(1, (uint8_t*) let_i);   // È
  TM_HD44780_CreateChar(2, (uint8_t*) let_l);   // Ë
  TM_HD44780_CreateChar(3, (uint8_t*) let_ii);  // Û
  TM_HD44780_CreateChar(4, (uint8_t*) let_iii); // Ú
  TM_HD44780_CreateChar(5, (uint8_t*) let_b);   // Á
  TM_HD44780_CreateChar(6, (uint8_t*) let_gg);  // Æ
  TM_HD44780_CreateChar(7, (uint8_t*) let_d);   // Ä
}

void reInit() {
  createRusChars();
  TM_HD44780_Cmd(0x20 | HD44780_Opts.DisplayFunction);
  HD44780_Opts.DisplayControl = 0x04;
  TM_HD44780_DisplayOn();
}*/



void printLoseDetected() {
//  createRusChars();
  TM_HD44780_Puts(0, 1, "  HET TAP !     ");
  char lit[2] = {_Ib, '\0'};
  TM_HD44780_Puts(9, 1, &lit[0]);
}

void printNotReady(uint32_t lits){
//  createRusChars();
  TM_HD44780_Puts(0, 0, "HE ");
  char lit0[3] = {_G, '\0'};
  TM_HD44780_Puts(3,0, &lit0[0]);
  TM_HD44780_Puts(4,0, "OTOB, ");
  
  if (lits > 9999) lits = 9999;
  char lit[5];
  uint32_t liters = lits / 100;
  lit[0] = liters/100 + 0x30;           if (lit[0] == '0') lit[0] = ' ';
  lit[1] = (liters%100)/10 + 0x30;      if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
  lit[2] = liters%10 + 0x30;    
  lit[3] = '\0';
  TM_HD44780_Puts(9,0, &lit[1]);
  liters = lits % 100;
  lit[0] = ',';
  lit[1] = liters/10 + 0x30;
  lit[2] = liters%10 + 0x30;
  lit[3] = '\0';
  TM_HD44780_Puts(11,0, &lit[0]);
    
  lit0[0] = _L;
  lit0[1] = '.';
  lit0[2] = '\0';  
  TM_HD44780_Puts(14,0, &lit0[0]);    
  
  char lit2[17] = {'B', 'O', _D, 'A', ' ', _F, _I, _L, 'b', 'T', 'P', _Y, 'E', 'T', 'C', _R};    
  TM_HD44780_Puts(0,1, &lit2[0]);
}

void printWait(uint32_t lits){  
//  createRusChars();
  // ÃÎÒÎÂÎ: 15Ë.
  char lit0[17] = { ' ', 'B', ' ', _B, 'A', 'K', 'E', ':', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  TM_HD44780_Puts(0, 0, &lit0[0]);
  
  if (lits > 99999) lits = 99999;
  char lit[5];
  uint32_t liters = lits / 100;
  lit[0] = liters/100 + 0x30;           if (lit[0] == '0') lit[0] = ' ';
  lit[1] = (liters%100)/10 + 0x30;      if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
  lit[2] = liters%10 + 0x30;    
  lit[3] = '\0';
  TM_HD44780_Puts(8,0, &lit[0]);
  liters = lits % 100;
  lit[0] = ',';
  lit[1] = liters/10 + 0x30;
  lit[2] = liters%10 + 0x30;
  lit[3] = '\0';
  TM_HD44780_Puts(11,0, &lit[0]);
    
  lit0[0] = _L;
  lit0[1] = '.';
  lit0[2] = '\0';  
  TM_HD44780_Puts(14,0, &lit0[0]);    
  
  // ÂÍÅÑÈÒÅ ÎÏËÀÒÓ
  char lit1[17] = {' ', 'B', 'H', 'E', 'C', _I, 'T', 'E', ' ', 'O', _P, _L, 'A', 'T', _Y, ' ', '\0'};
  TM_HD44780_Puts(0, 1, &lit1[0]);  
}

      
void printPaid(uint16_t rub, uint16_t litersPaid) {
  char lit[6];
  if (rub > 999) rub = 999;
  lit[0] = rub/100 + 0x30;              if (lit[0] == '0') lit[0] = ' ';
  lit[1] = (rub%100)/10 + 0x30;         if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
  lit[2] = rub%10 + 0x30;
  lit[3] = 0;
  TM_HD44780_Puts(0,0, &lit[0]);
  char lit1[] = {' ', 'P', _Y, _B, '.', ' ', '\0'};
  TM_HD44780_Puts(3,0, &lit1[0]);
  
  if (litersPaid > 999) litersPaid = 999;
  lit[0] = litersPaid/100 + 0x30;           if (lit[0] == '0') lit[0] = ' ';
  lit[1] = (litersPaid%100)/10 + 0x30;
  lit[2] = '.';
  lit[3] = litersPaid%10 + 0x30;    
  lit[4] = 0;
  TM_HD44780_Puts(9,0, &lit[0]);    
  
  lit[0] = _L;
  lit[1] = '.';
  lit[2] = '\0';
  TM_HD44780_Puts(14,0, &lit[0]);

  char lit2[17] = {' ', 'H', 'A', _GG, 'M', _I, 'T', 'E', ' ', 'C', 'T', 'A', 'P', 'T', ' ', ' ', '\0'};
  TM_HD44780_Puts(0,1, &lit2[0]);
}

void printGiven(uint32_t milLitOut, uint32_t liters, uint32_t rub, bool pause){
//  createRusChars();
  char let0[17] = {' ', 'B', _Ib, _D, 'A', 'H', 'O', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  char let1[17] = {' ', _P, 'A', _Y, '3', 'A', '.', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  
  if (pause) TM_HD44780_Puts(0,0, &let1[0]);
  else TM_HD44780_Puts(0,0, &let0[0]);

  char lit[6];
  if (liters > 999) liters = 999;
  if (liters > 99) {                            // >10.0..99.9 liters - 100 - 999   
    lit[0] = liters/100 + 0x30;                 if (lit[0] == '0') lit[0] = ' ';
    lit[1] = (liters%100)/10 + 0x30;            if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
    lit[2] = ',';
    lit[3] = liters%10 + 0x30;    
  }
  else {                                        // <10 liters - 99 - 9.9
    lit[0] = ' ';
    lit[1] = liters/10 + 0x30;
    lit[2] = ',';
    lit[3] = liters%10 + 0x30;
  }
  lit[4] = '\0';
  TM_HD44780_Puts(9,0, &lit[0]);
  lit[0] = _L;
  lit[1] = '.';
  lit[2] = '\0';
  TM_HD44780_Puts(13,0, &lit[0]);
  
  if (milLitOut > 999) milLitOut = 999;
  if (milLitOut > 99) {                       // >10.0..99.9 milLitOut - 100 - 999   
    lit[0] = milLitOut/100 + 0x30;            if (lit[0] == '0') lit[0] = ' ';
    lit[1] = (milLitOut%100)/10 + 0x30;       if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
    lit[2] = ',';
    lit[3] = milLitOut%10 + 0x30;    
  }
  else {                                // <10 milLitOut - 99 - 9.9
    lit[0] = ' ';
    lit[1] = milLitOut/10 + 0x30;
    lit[2] = ',';
    lit[3] = milLitOut%10 + 0x30;
  }
  lit[4] = '\0';  
  
  TM_HD44780_Puts(0,1, &lit[0]);  
  lit[0] = ' ';
  lit[1] = _L;
  lit[2] = '.';
  lit[3] = '-';
  lit[4] = '\0';
  TM_HD44780_Puts(4,1, &lit[0]);  
  
  lit[0] = rub/100 + 0x30;
  if (lit[0] == '0') lit[0] = ' ';      if (lit[0] == '0') lit[0] = ' ';
  lit[1] = (rub%100)/10 + 0x30;         if (lit[1] == '0' && lit[0] == ' ') lit[1] = ' ';
  lit[2] = rub%10 + 0x30;
  lit[3] = '\0';
  TM_HD44780_Puts(8,1, &lit[0]);
  
  char lit1[] = {' ', 'P', _Y, _B, '.','\0'};
  TM_HD44780_Puts(11,1, &lit1[0]);
}


extern filtersStr filters;

void printMeshFil(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "CET.100M. ");
  char lit[7] = {0}; 
  if (!(filters.meshFil.curValue > filters.meshFil.resource)) {    
    lit[0] = (filters.meshFil.resource - filters.meshFil.curValue)/100000 + 0x30;
    lit[1] = ((filters.meshFil.resource - filters.meshFil.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.meshFil.resource - filters.meshFil.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.meshFil.resource - filters.meshFil.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.meshFil.resource - filters.meshFil.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.meshFil.resource - filters.meshFil.curValue)%10) + 0x30;
    lit[6] = 0;
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printMechFil10(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "MEX.10MK.  ");
  char lit[7] = {0}; 
  if (!(filters.mechFil10.curValue > filters.mechFil10.resource)) {    
    lit[0] = (filters.mechFil10.resource - filters.mechFil10.curValue)/100000 + 0x30;
    lit[1] = ((filters.mechFil10.resource - filters.mechFil10.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.mechFil10.resource - filters.mechFil10.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.mechFil10.resource - filters.mechFil10.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.mechFil10.resource - filters.mechFil10.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.mechFil10.resource - filters.mechFil10.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printMechFil5(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "MEX.5MK.  ");
  char lit[7] = {0}; 
  if (!(filters.mechFil5.curValue > filters.mechFil5.resource)) {    
    lit[0] = (filters.mechFil5.resource - filters.mechFil5.curValue)/100000 + 0x30;
    lit[1] = ((filters.mechFil5.resource - filters.mechFil5.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.mechFil5.resource - filters.mechFil5.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.mechFil5.resource - filters.mechFil5.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.mechFil5.resource - filters.mechFil5.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.mechFil5.resource - filters.mechFil5.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printCoalFil1(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "Y");
  TM_HD44780_PutCustom(2, line, 0);
  TM_HD44780_Puts(3,line, "O");
  TM_HD44780_PutCustom(4, line, 2);
  TM_HD44780_Puts(5,line, ". 1  ");
  char lit[7] = {0}; 
  if (!(filters.coalFil1.curValue > filters.coalFil1.resource)) {    
    lit[0] = (filters.coalFil1.resource - filters.coalFil1.curValue)/100000 + 0x30;
    lit[1] = ((filters.coalFil1.resource - filters.coalFil1.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.coalFil1.resource - filters.coalFil1.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.coalFil1.resource - filters.coalFil1.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.coalFil1.resource - filters.coalFil1.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.coalFil1.resource - filters.coalFil1.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printCoalFil2(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "Y");
  TM_HD44780_PutCustom(2, line, 0);
  TM_HD44780_Puts(3,line, "O");
  TM_HD44780_PutCustom(4, line, 2);
  TM_HD44780_Puts(5,line, ". 2  ");
  char lit[7] = {0}; 
  if (!(filters.coalFil2.curValue > filters.coalFil2.resource)) {    
    lit[0] = (filters.coalFil2.resource - filters.coalFil2.curValue)/100000 + 0x30;
    lit[1] = ((filters.coalFil2.resource - filters.coalFil2.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.coalFil2.resource - filters.coalFil2.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.coalFil2.resource - filters.coalFil2.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.coalFil2.resource - filters.coalFil2.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.coalFil2.resource - filters.coalFil2.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printAfterFil(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "POST F.  ");
  char lit[7] = {0}; 
  if (!(filters.afterFil.curValue > filters.afterFil.resource)) {    
    lit[0] = (filters.afterFil.resource - filters.afterFil.curValue)/100000 + 0x30;
    lit[1] = ((filters.afterFil.resource - filters.afterFil.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.afterFil.resource - filters.afterFil.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.afterFil.resource - filters.afterFil.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.afterFil.resource - filters.afterFil.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.afterFil.resource - filters.afterFil.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printBackOsm1(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "OCMOC 1  ");
  char lit[7] = {0}; 
  if (!(filters.backOsm1.curValue > filters.backOsm1.resource)) {
    lit[0] = (filters.backOsm1.resource - filters.backOsm1.curValue)/100000 + 0x30;
    lit[1] = ((filters.backOsm1.resource - filters.backOsm1.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.backOsm1.resource - filters.backOsm1.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.backOsm1.resource - filters.backOsm1.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.backOsm1.resource - filters.backOsm1.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.backOsm1.resource - filters.backOsm1.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printBackOsm2(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "OCMOC 2  ");
  char lit[7] = {0};
  if (!(filters.backOsm2.curValue > filters.backOsm2.resource)) {    
    lit[0] = (filters.backOsm2.resource - filters.backOsm2.curValue)/100000 + 0x30;
    lit[1] = ((filters.backOsm2.resource - filters.backOsm2.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.backOsm2.resource - filters.backOsm2.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.backOsm2.resource - filters.backOsm2.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.backOsm2.resource - filters.backOsm2.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.backOsm2.resource - filters.backOsm2.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}

void printBackOsm3(uint8_t line) { 
  if (line == 0) TM_HD44780_Puts(0, 0, ">");
  else TM_HD44780_Puts(0, 0, " ");
  TM_HD44780_Puts(1,line, "OCMOC 3  ");
  char lit[7] = {0}; 
  if (!(filters.backOsm3.curValue > filters.backOsm3.resource)) {    
    lit[0] = (filters.backOsm3.resource - filters.backOsm3.curValue)/100000 + 0x30;
    lit[1] = ((filters.backOsm3.resource - filters.backOsm3.curValue)%100000)/10000 + 0x30;
    lit[2] = ((filters.backOsm3.resource - filters.backOsm3.curValue)%10000)/1000 + 0x30;
    lit[3] = ((filters.backOsm3.resource - filters.backOsm3.curValue)%1000)/100 + 0x30;
    lit[4] = ((filters.backOsm3.resource - filters.backOsm3.curValue)%100)/10 + 0x30;
    lit[5] = ((filters.backOsm3.resource - filters.backOsm3.curValue)%10) + 0x30;
    lit[6] = 0;  
  }
  TM_HD44780_Puts(10,line, &lit[0]);
}


void printService(uint8_t curs){
  switch (curs) {
  case 0: printMeshFil(0);
          printMechFil10(1);
          break;
  case 1: printMechFil10(0);
          printMechFil5(1);
          break;
  case 2: printMechFil5(0);
          printCoalFil1(1);
          break;
  case 3: printCoalFil1(0);
          printCoalFil2(1);
          break;
  case 4: printCoalFil2(0);
          printAfterFil(1);
          break;
  case 5: printAfterFil(0);
          printBackOsm1(1);
          break;
  case 6: printBackOsm1(0);
          printBackOsm2(1);
          break;
  case 7: printBackOsm2(0);
          printBackOsm3(1);
          break;
  case 8: printBackOsm3(0);
          printMeshFil(1);
          break;
          
  }
}


void printAdminUp() {
  static char data[2] = "1";
  TM_HD44780_Puts(0, 0, &data[0]);
  data[0]++;
  if (data[0] > '9') data[0] = '1';  
}

void printAdminDown() {
  static char data[2] = "1";
  TM_HD44780_Puts(0, 1, &data[0]);
  data[0]++;
  if (data[0] > '9') data[0] = '1';  
}
      
void printAdminLeft() {
  static char data[2] = "1";
  TM_HD44780_Puts(6, 0, &data[0]);
  data[0]++;
  if (data[0] > '9') data[0] = '1';  
}

void printAdminRight() { 
  static char data[2] = "1";
  TM_HD44780_Puts(6, 1, &data[0]);
  data[0]++;
  if (data[0] > '9') data[0] = '1';  
}
