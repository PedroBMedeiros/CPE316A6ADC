
#ifndef USART_H_
#define USART_H_

#define DELAY 50000
#define ZEROAF 0x000F
#define SETAF7 0X0007
#define BAUDRATE 208
#define RESETTRANSMISSION 0
#define ESCASCII 0x1B
#define CLOSEBRACKETASCII 0X5B
#define REDFONT "31m"
#define BLUEFONT "34m"
#define WHITEFONT "37m"
#define GREENFONT "32m"


void USART2_init(void);
void USART2_Print(const char *string);
void USART_ESC_Print(const char *message);
void USART2_IRQHandler(void);

#endif /* USART_H_ */
