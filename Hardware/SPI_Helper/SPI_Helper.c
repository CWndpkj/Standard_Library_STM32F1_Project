#include <stm32f10x.h>
#include "SPI_Helper.h"

u8 SPI_Helper_RcvBuff[256];
u8 SPI_Helper_RcvPtr; // 接收缓存区指针

u8 SPI_Helper_Init()
{
    // 开启SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    // 开启DMA时钟
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 初始化SPI

    SPI_InitTypeDef SPI_InitStruct;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 波特率预分频系数
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;                    // SPI模式0
    SPI_InitStruct.SPI_CRCPolynomial     = 7;                               // CRC校验多项式
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;                 // 数据长度8bit
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;                // 高位在前
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;                 // 主模式
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Hard;                    // 硬件NSS

    SPI_Init(SPI1, &SPI_InitStruct);
    // 使能SPI
    SPI_Cmd(SPI1, ENABLE);
    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

    // 初始化DMA
    // 初始化SPI接收DMA通道
    SPI_Helper_DMAInit(DMA1_Channel2, 256, DMA_DIR_PeripheralSRC, SPI_Helper_RcvBuff, DMA_Mode_Circular, (u8)SPI1->DR);
}

u8 SPI_Helper_DMAInit(u8 Channelx, u8 BufferSize, u8 Diraction, u8 MemoryBaseAddr, u8 Mode, u8 PeripheralBaseAddr)
{
    // 初始化DMA
    DMA_Cmd(Channelx, DISABLE);
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitSruct.DMA_BufferSize         = BufferSize;
    DMA_InitSruct.DMA_DIR                = Diraction;       // 转运方向
    DMA_InitSruct.DMA_M2M                = DMA_M2M_Disable; // mem关闭
    DMA_InitSruct.DMA_MemoryBaseAddr     = MemoryBaseAddr;
    DMA_InitSruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitSruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitSruct.DMA_Mode               = Mode; // 不自动重装
    DMA_InitSruct.DMA_PeripheralBaseAddr = PeripheralBaseAddr;
    DMA_InitSruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitSruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitSruct.DMA_Priority           = DMA_Priority_High;

    DMA_Init(Channelx, &DMA_InitStruct);

    // 使能DMA
    DMA_Cmd(Channelx, ENABLE);
    return 0;
}

u8 SPI_Helper_GetRcvLength()
{
    int16_t length = 0;
    // if (256 - DMA_GetCurrDataCounter(DMA1_Channel2) > SPI_Helper_RcvPtr)
    //     length = 256 - DMA_GetCurrDataCounter(DMA1_Channel2) - SPI_Helper_RcvPtr;
    // else
    //     length = 256 - SPI_Helper_RcvPtr + 256 - DMA_GetCurrDataCounter(DMA1_Channel2);

    length = 256 - DMA_GetCurrDataCounter(DMA1_Channel2) - SPI_Helper_RcvPtr;
    if (length < 0) {
        length += 256;
    }
    return (u8)length;
}

u8 SPI_Helper_SendByte(u8 Byte2Send)
{
    // 拉低NSS
    SPI_SSOutputCmd(SPI1, ENABLE);
    SPI_I2S_SendData(SPI1, Byte2Send);
    SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_RXNE);
    // 释放NSS
    SPI_SSOutputCmd(SPI1, DISABLE);
    return 0;
}

u8 SPI_Helper_SendLen(u8 *pBuff, u8 length)
{
    // 重新初始化SPI的发送DMA通道
    SPI_SSOutputCmd(SPI1, ENABLE);
    SPI_Helper_DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pBuff, DMA_Mode_Normal, SPI1->DR);
    SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY);
    // 释放NSS
    SPI_SSOutputCmd(SPI1, DISABLE);

    return 0;
}

u8 SPI_Helper_RcvByte(u8 *pBuff)
{
    if (SPI_Helper_GetRcvLength() < 1)
        return 1;
    *pBuff = SPI_Helper_RcvBuff[SPI_Helper_RcvPtr++];
    return 0;
}

u8 SPI_Helper_RcvLen(u8 *pBuff, u8 Length)
{
    if (SPI_Helper_GetRcvLength() < Length)
        return 1;
    for (u8 i = 0; i < Length; i++) {
        pBuff[i] = SPI_Helper_RcvBuff[SPI_Helper_RcvPtr++];
    }
    return 0;
}

u8 SPI_Helper_SwapByte(u8 *pBuff)
{
}

u8 SPI_Helper_SwapLen(u8 *pBuffSend, u8 *pBuffRcv, u8 Length)
{
}

u8 SPI_Helper_TimeOutAssert(uint16_t SPI_I2S_FLAG)
{
    u16 TimeOut = 10000;
    u8 err      = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG) != SET) {
        TimeOut--;
        if (TimeOut == 0) {
            // TODO:错误处理
            break;
        }
    }
    return err;
}
