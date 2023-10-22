#include <stm32f10x.h>
#include "USART_Helper.h"
#include "delay.h"
#include <stdio.h>
#include "LED.h"

#include "HWInterface.h"
#include "lcd.h"

#ifdef __cplusplus
extern "C" {
#endif
#define USART_Helper_Baud 115200

uint8_t USART_Helper_RcvBuff[256]; // 循环队列
uint8_t USART_Helper_pRcvBuff;

u8 USART_Helper_GetRcvLen();
void UASRT_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 DIR, u32 MemoryBaseAddr, u32 Mode);

u8 USART_Helper_Init()
{
    LED_Init();

    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 初始化USART
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate            = USART_Helper_Baud;              // 波特率
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 硬件流控制
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity              = USART_Parity_No;     // 奇偶校验
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;    // 停止位
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b; // 数据字长
    USART_Init(USART2, &USART_InitStruct);
    // 开启USART DMA
    USART_DMACmd(USART2, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

    // 开启USART 中断
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    // 使能USART
    USART_Cmd(USART2, ENABLE);

    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);
    // 开启DMA中断
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel6_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);
    // 开启DMA中断
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    // 初始化DMA
    UASRT_Helper_DMAInit(DMA1_Channel6, 256, DMA_DIR_PeripheralSRC, (u32)USART_Helper_RcvBuff, DMA_Mode_Circular);
    // 开启DMA
    DMA_Cmd(DMA1_Channel6, ENABLE);

    return 0;
}

/**
 * @brief USART中断句柄函数
 *
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) == SET) {
        // USART主线空闲,此时对接收数据进行判定,查看是否接收到数据
        u8 length = USART_Helper_GetRcvLen();
        if (length != 0) {
            // TODO:调用自定义的数据处理函数,接收到的数据储存在USART_Helper_RcvBuff[]中,长度为length
            // 从USART_Helper_RcvBuff[USART_Helper_pRcvBuff]开始,长度为length
            // note:读取数据后必须将缓存数组的指针移动相应的位
            // 采用以下格式 : USART_Helper_RcvBuff[USART_Helper_pRcvBuff++] 读取数据
            // for (u8 i = 0; i < length; i++) {
            //     // AnoPTv8HwRecvByte(USART_Helper_RcvBuff[USART_Helper_pRcvBuff++]);
            //     printf("%c", USART_Helper_RcvBuff[USART_Helper_pRcvBuff++]);
            // }
            // LCD_SetWindows(0, 0, 32 - 1, 32 - 1); // 窗口设置
            // SPI_LCD.WriteLen(USART_Helper_RcvBuff + USART_Helper_pRcvBuff, length);
            // Gui_DrawPic(0, 0, 32, 32, USART_Helper_RcvBuff + USART_Helper_pRcvBuff);

            // LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); // 恢复显示窗口为全屏
            //  Gui_DrawPic(0, 0, 31, 31, USART_Helper_RcvBuff + USART_Helper_pRcvBuff);
            USART_Helper_pRcvBuff += length;
            printf("RcvLen:%d", length);
        }
        // note:注意查看参数! 不可通过以下方式清除IDLE,因为IDLE位为只读
        // USART_ClearITPendingBit(USART1,USART_IT_IDLE); // 清除中断标志位
        // 按照手册,读取SR寄存器,再读取DR寄存器,可清除   USART_IT_IDLE  标志位
        (void *)USART2->DR; // 清除   USART_IT_IDLE  标志位
    }
}
void DMA1_Channel6_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC6) == SET) {
        // 在成功接收256byte后的中断函数
        SPI_LCD.WriteLen(USART_Helper_RcvBuff, 256);
        DMA_ClearITPendingBit(DMA1_IT_TC6);
    }
}
void UASRT_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 DIR, u32 MemoryBaseAddr, u32 Mode)
{
    DMA_Cmd(DMAy_Channelx, DISABLE); // 关闭DMA
    DMA_InitTypeDef DMA_InitSruct;
    DMA_InitSruct.DMA_BufferSize         = BufferSize;
    DMA_InitSruct.DMA_DIR                = DIR;             // 转运方向
    DMA_InitSruct.DMA_M2M                = DMA_M2M_Disable; // mem关闭
    DMA_InitSruct.DMA_MemoryBaseAddr     = MemoryBaseAddr;
    DMA_InitSruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitSruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitSruct.DMA_Mode               = Mode;
    DMA_InitSruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
    DMA_InitSruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitSruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitSruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMAy_Channelx, &DMA_InitSruct);
    // 开启DMA
    DMA_Cmd(DMAy_Channelx, ENABLE);
}

u8 USART_Helper_SendByte(u8 Buff)
{
    int32_t TimeOutErr = 10000;
    USART_SendData(USART2, Buff);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET) {
        TimeOutErr--;
        if (TimeOutErr == 0) {
            // TODO:错误处理
            LED_On();
            return 1;
        }
    }
    return 0;
}

u8 USART_Helper_SendLen(uint8_t *pBuff, uint8_t length)
{
    // 初始化DMA
    UASRT_Helper_DMAInit(DMA1_Channel7, length, DMA_DIR_PeripheralDST, (u32)pBuff, DMA_Mode_Normal);
    return 0;
}

u8 USART_Helper_GetRcvLen()
{
    int length = 256 - DMA_GetCurrDataCounter(DMA1_Channel6) - USART_Helper_pRcvBuff;
    if (length < 0)
        return 256 + length;
    else
        return length;
}

int fputc(int ch, FILE *f)
{
    USART_Helper_SendByte((uint8_t)ch);
    return ch;
}

#ifdef __cplusplus
}
#endif