#include <stm32f10x.h>
#include "USART_Helper.h"
#include "delay.h"
#include <stdio.h>

uint8_t USART_Helper_SendBuff[256];
uint8_t USART_Helper_RcvBuff[256]; // 循环队列
uint8_t USART_Helper_pSendBuff;
uint8_t USART_Helper_pRcvBuff;
uint8_t USART_Helper_DataReadyFlag = 0;
u8 USART_Helper_Init()
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 初始化USART
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate            = 9600;                           // 波特率
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 硬件流控制
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity              = USART_Parity_No;     // 奇偶校验
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;    // 停止位
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b; // 数据字长
    USART_Init(USART1, &USART_InitStruct);
    // 开启USART DMA
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    // 使能USART
    USART_Cmd(USART1, ENABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitSruct;
    DMA_InitSruct.DMA_BufferSize         = 256;
    DMA_InitSruct.DMA_DIR                = DMA_DIR_PeripheralSRC; // 转运方向
    DMA_InitSruct.DMA_M2M                = DMA_M2M_Disable;       // mem关闭
    DMA_InitSruct.DMA_MemoryBaseAddr     = (uint32_t)USART_Helper_Buff;
    DMA_InitSruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitSruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitSruct.DMA_Mode               = DMA_Mode_Circular; // 自动重装
    DMA_InitSruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitSruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitSruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitSruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMA1_Channel5, &DMA_InitSruct);
    // 使能DMA
    DMA_Cmd(DMA1_Channel5, ENABLE);
    return 1;
}

u8 USART_Helper_GetDataReadyFlag(uint8_t length)
{
    if (256 - DMA_GetCurrDataCounter(DMA1_Channel5) >= USART_Helper_pBuff + length) {
        return 1;
    } else
        return 0;
}
u8 USART_Helper_RcvLen(uint8_t *PackDataBuff, uint8_t length)
{
    if (!USART_Helper_GetDataReadyFlag(length))
        return 0;
    for (uint8_t i = 0; i < length; i++) {
        PackDataBuff[i] = USART_Helper_Buff[USART_Helper_pBuff++];
    }
    return 1;
}
u8 USART_Helper_RcvByte(u8 *RcvBuff)
{
    return USART_Helper_RcvLen(RcvBuff, 1);
}
u8 USART_Helper_SendLen(uint8_t *PackData, uint8_t length)
{
    uint8_t i;
    u8 flag = 1;
    for (i = 0; i < length; i++) {
        if (!USART_Helper_TimeOutAssert(USART1, USART_FLAG_TXE)) flag = 0;
        USART_SendData(USART1, PackData[i]);
    }
    return flag;
}

u8 USART_Helper_TimeOutAssert(USART_TypeDef *USARTx, uint16_t USART_FLAG)
{
    uint32_t TimeOut = 10000;
    u8 flag          = 1;
    while (USART_GetFlagStatus(USART1, USART_FLAG) != SET) {
        TimeOut--;
        if (TimeOut == 0) {
            // TODO:错误处理

            flag = 0;
            break;
        }
    }
    return flag;
}
u8 USART_Helper_SendByte(u8 SendData)
{
    return USART_Helper_SendLen(&SendData, 1);
}
int fputc(int ch, FILE *f)
{
    USART_Helper_SendByte((uint8_t)ch);
    return ch;
}