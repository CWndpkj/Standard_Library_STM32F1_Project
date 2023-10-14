#include <stm32f10x.h>
#include "SPI_Helper.h"
#include "LED.h"
#include <stdio.h>
#include "delay.h"

u8 SPI_Helper_Transfer_Finish_Flag;

int8_t SPI_Helper_Init()
{
    LED_Init();
    // 开启SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    // 开启DMA时钟
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
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
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;                    // 软件NSS
    SPI_Init(SPI1, &SPI_InitStruct);
    // 使能SPI
    SPI_Cmd(SPI1, ENABLE);
    // 配置NVIC
    SPI_Helper_NVICInit();
    return 0;
}
int8_t SPI_Helper_NVICInit()
{
    // 开启DMA传输完成中断
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);
    return 0;
}
/**
 * @brief Tx_transfer_complete_cb
 *
 */
void DMA1_Channel3_IRQnHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
        // 置标志位
        SPI_Helper_Transfer_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC3);
    }
}
/**
 * @brief Rx_transfer_complete_cb
 *
 */
void DMA1_Channel2_IRQnHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
        // 置标志位
        SPI_Helper_Transfer_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

int8_t SPI_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u16 BufferSize, u32 Diraction, u8 *MemoryBaseAddr)
{
    // 失能DMA
    DMA_Cmd(DMAy_Channelx, DISABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = BufferSize;
    DMA_InitStruct.DMA_DIR                = Diraction;       // 转运方向
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable; // mem关闭
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)MemoryBaseAddr;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)SPI1->DR;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMAy_Channelx, &DMA_InitStruct);
    // 使能DMA
    DMA_Cmd(DMAy_Channelx, ENABLE);
    return 0;
}

int8_t SPI_Helper_WriteLen(uint16_t CS_Pinx, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin   = CS_Pinx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, CS_Pinx);
    // 发送操作的寄存器地址
    SPI_I2S_SendData(SPI1, reg_addr);
    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, reg_data);
    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    // 等待DMA传输完成
    SPI_Helper_WaitforFinish();
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET);
    // 拉高片选
    GPIO_SetBits(GPIOB, CS_Pinx);
    return ErrCode;
}
int8_t SPI_Helper_ReadLen(uint16_t CS_Pinx, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin   = CS_Pinx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, CS_Pinx);
    // 发送操作的寄存器地址
    SPI_I2S_SendData(SPI1, reg_addr);
    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, reg_data);
    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
    // 等待DMA传输完成
    SPI_Helper_WaitforFinish();
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET);
    // 拉高片选
    GPIO_SetBits(GPIOB, CS_Pinx);
    return ErrCode;
}
uint8_t SPI_Helper_SwapLen(uint16_t CS_Pinx, uint8_t reg_addr, u8 *pSendBuff, u8 *pRcvBuff, u8 length)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin   = CS_Pinx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, CS_Pinx);
    // 发送操作的寄存器地址
    SPI_I2S_SendData(SPI1, reg_addr);
    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pSendBuff);
    SPI_Helper_DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    // 等待DMA传输完成
    SPI_Helper_WaitforFinish();
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET);
    // 等待最后一个数据被读取
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_RXNE, SET);
    // 拉高片选
    GPIO_SetBits(GPIOB, CS_Pinx);
    return ErrCode;
}

int8_t SPI_Helper_TimeOutAssert(uint16_t SPI_I2S_FLAG, FlagStatus Status)
{
    u16 TimeOut = 10000;
    u8 err      = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG) != Status) {
        TimeOut--;
        if (TimeOut == 0) {
            // TODO:错误处理
            break;
        }
    }
    return err;
}

int8_t SPI_Helper_WaitforFinish()
{
    u16 TimeOut = 1000;
    u8 err      = 0;
    while (SPI_Helper_Transfer_Finish_Flag != 1) {
        TimeOut--;
        delay_ms(1);
        if (TimeOut == 0) {
            err = 1;
            LED_On();
            printf("%s\t\n", "error in :SPI_Helper_WaitforFinish()");
            break;
        }
    }
    SPI_Helper_Transfer_Finish_Flag = 0;
    return err;
}

// u8 SPI_Helper_WaitBUSY()
// {
//     u32 TimeOut = 10000;
//     u8 err      = 0;
//     while (I2C_GetFlagStatus(SPI1, I2C_FLAG_BUSY) == SET) {
//         TimeOut--;
//         if (TimeOut == 0) {
//             err = 1;
//             LED_On();
//             printf("%s\t\n", "error in :I2C_DMA_WaitBUSY()");
//             I2C_DMA_SlaveDeviceReset();
//             I2C_DeInit(I2C2);
//             I2C_DMA_Init();

//             break;
//         }
//     }
//     return err;
// }
