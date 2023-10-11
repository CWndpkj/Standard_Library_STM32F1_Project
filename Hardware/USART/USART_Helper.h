#ifndef _USART_HELPER_H_
#define _USART_HELPER_H_
#include <stdio.h>
u8 USART_Helper_Init();
u8 USART_Helper_RcvLen(uint8_t *RcvBuff, uint8_t length);
u8 USART_Helper_SendLen(uint8_t *SendData, uint8_t length);
u8 USART_Helper_SendByte(u8 SendData);
u8 USART_Helper_RcvByte(u8 *RcvBuff);
uint8_t USART_Helper_GetDataReadyFlag(uint8_t length);
u8 USART_Helper_TimeOutAssert(USART_TypeDef *USARTx, uint16_t USART_FLAG);
int fputc(int ch, FILE *f);
void send_format_data(unsigned char f_code, unsigned char *data, unsigned char length);
#endif // !_USART_H_