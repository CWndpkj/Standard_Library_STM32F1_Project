#ifndef _USART_HELPER_H_
#define _USART_HELPER_H_
#include <stdio.h>
u8 USART_Helper_Init();
u8 USART_Helper_SendByte(u8 Buff);
u8 USART_Helper_SendLen(uint8_t *pBuff, uint8_t length);

u8 USART_Helper_TimeOutAssert(USART_TypeDef *USARTx, uint16_t USART_FLAG);
int fputc(int ch, FILE *f);
#endif // !_USART_H_