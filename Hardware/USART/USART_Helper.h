#include <stdio.h>
#include "stm32f10x.h"
#ifndef _USART_HELPER_H_
#define _USART_HELPER_H_
#ifdef __cplusplus
extern "C" {
#endif 
u8 USART_Helper_Init();
u8 USART_Helper_SendByte(u8 Buff);
u8 USART_Helper_SendLen(uint8_t *pBuff, uint8_t length);
#ifdef __cplusplus
}
#endif 
#endif // !_USART_H_