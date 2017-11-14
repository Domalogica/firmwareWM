#ifndef MONEY_ACCEPT_MGMNT_H_
#define MONEY_ACCEPT_MGMNT_H_

void checkCoinMoney(void);
void checkMoney(void);

#define COIN_PORT       NINT_IN13_GPIO_Port             // PB3, R25, 8
#define COIN_PIN        NINT_IN13_Pin

#define MONEY_PORT      NINT_IN14_GPIO_Port             // PD7, R26, 9
#define MONEY_PIN       NINT_IN14_Pin

#endif