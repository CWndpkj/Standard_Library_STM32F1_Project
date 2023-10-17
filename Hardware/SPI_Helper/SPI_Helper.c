#include <stm32f10x.h>
#include "SPI_Helper.h"
#include "LED.h"
#include <stdio.h>
#include "delay.h"

#define TimeOut_Ratio 100
uint16_t CS_Pin_x;
u8 SPI_Helper_DMA2Send_Finish_Flag;
u8 SPI_Helper_Rcv2DMA_Finish_Flag;
int8_t SPI_Helper_WaitforFinish(u8 Flag, u32 nus);
int8_t SPI_Helper_TimeOutAssert(uint16_t SPI_I2S_FLAG, FlagStatus Status, u32 nus);
int8_t SPI_Helper_NVICInit();
int8_t SPI_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u16 BufferSize, u32 Diraction, u8 *MemoryBaseAddr);

int8_t SPI_Helper_Init(uint16_t GPIO_Pin_x)
{
    LED_Init();
    CS_Pin_x = GPIO_Pin_x;
    // 开启SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    // 开启DMA时钟
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 初始化用于GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 初始化用于CS的GPIO
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin   = CS_Pin_x;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_SetBits(GPIOB, CS_Pin_x); // 初始设置为高电平
    GPIO_Init(GPIOB, &GPIO_InitStruct);

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

    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

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
void DMA1_Channel3_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
        // 置标志位
        SPI_Helper_DMA2Send_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC3);
    }
}
/**
 * @brief Rx_transfer_complete_cb
 *
 */
void DMA1_Channel2_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
        // 置标志位
        SPI_Helper_Rcv2DMA_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

int8_t SPI_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u16 BufferSize, u32 Diraction, u8 *MemoryBaseAddr)
{
    // 失能DMA
    DMA_Cmd(DMAy_Channelx, DISABLE);
    // DMA_DeInit(DMAy_Channelx);
    // DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
    // DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = BufferSize;
    DMA_InitStruct.DMA_DIR                = Diraction;       // 转运方向
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable; // mem关闭
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)MemoryBaseAddr;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR; // 一定记得要加取地址符
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMAy_Channelx, &DMA_InitStruct);
    // 使能DMA
    DMA_Cmd(DMAy_Channelx, ENABLE);
    return 0;
}

/**
 * @brief SPI写入指定长度数据
 * @param pSendBuff 要写入的数据
 * @param length   要写入数据的长度
 * @return int8_t  0表示写入成功,否则失败
 */
int8_t SPI_Helper_WriteLen(uint8_t *pSendBuff, uint16_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pSendBuff);

    // 等待DMA传输完成
    SPI_Helper_WaitforFinish(0, TimeOut_Ratio * length + 100000);
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    SPI1->DR;
    return ErrCode;
}

/**
 * @brief SPI 发送一个字节,在小数据发送上具有更高响应
 * @param pSendBuff 要发送的数据
 * @return int8_t
 */
int8_t SPI_Helper_WriteByte(uint8_t SendBuff)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // 发送数据
    SPI_I2S_SendData(SPI1, SendBuff);
    // 等待最后一个数据发送完,也接收完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    SPI1->DR;
    return ErrCode;
}

/**
 * @brief SPI读取指定长度函数
 * @param pRcvBuff 读取到的数据放在哪
 * @param length   读取的的数据的长度
 * @return int8_t  0为读取成功,否则出错
 */
int8_t SPI_Helper_ReadLen(uint8_t *pRcvBuff, uint16_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 发送缓冲数据产生时钟
    for (u16 i = 0; i < length; i++) {
        SPI_I2S_SendData(SPI1, 0xFF);
        // 等待发送完成
        ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }

    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(1, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    return ErrCode;
}

/**
 * @brief SPI 读取一个字节,读取少量数据具有更高的响应
 * @param pRcvBuff 读取到的数据存放的地址
 * @return int8_t  0为读取成功,否则失败
 */
int8_t SPI_Helper_ReadByte(uint8_t *pRcvBuff)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // SPI1->DR;

    SPI_I2S_SendData(SPI1, 0xFF);
    // 等待发送完成
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 1000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);
    // 读取数据
    *pRcvBuff = SPI_I2S_ReceiveData(SPI1);

    return ErrCode;
}

/**
 * @brief SPI 交换指定长度字节
 *
 * @param pSendBuff 要发送的数据
 * @param pRcvBuff  接收数据的地址
 * @param length    交换数据的长度
 * @return int8_t   返回0成功,否则失败
 */
int8_t SPI_Helper_SwapLen(u8 *pSendBuff, u8 *pRcvBuff, u8 length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // 初始化DMA
    SPI_Helper_DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pSendBuff);
    SPI_Helper_DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(0, TimeOut_Ratio * length + 10000);
    // 等待最后一个数据发送完,也接收完
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 10000);
    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(1, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    return ErrCode;
}

/**
 * @brief SPI 交换一个字节,读取少量数据具有更高的响应
 *
 * @param pSendBuff 要发送的字节
 * @param pRcvBuff  接收数据的地址
 * @return int8_t   返回0成功,否则失败
 */
int8_t SPI_Helper_SwapByte(u8 pSendBuff, u8 *pRcvBuff)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    SPI_I2S_SendData(SPI1, pSendBuff);
    // 等待发送完成
    ErrCode   = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);
    *pRcvBuff = SPI_I2S_ReceiveData(SPI1);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    return ErrCode;
}

int8_t SPI_Helper_WriteThenRead(u8 *pSendBuff, u16 WriteLength, u8 *pRcvBuff, u16 ReadLength)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);

    // 初始化发送DMA
    SPI_Helper_DMAInit(DMA1_Channel3, WriteLength, DMA_DIR_PeripheralDST, pSendBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(0, TimeOut_Ratio * WriteLength + 100000);
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 清除 SPI_I2S_FLAG_RXNE 标志
    SPI1->DR;

    // 初始化接收DMA
    SPI_Helper_DMAInit(DMA1_Channel2, ReadLength, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 发送填充产生时钟
    for (u16 i = 0; i < ReadLength; i++) {
        SPI_I2S_SendData(SPI1, 0xFF);
        // 等待数据发送完成
        ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }

    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(1, 1000000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    return ErrCode;
}

int8_t SPI_Helper_WriteCommand_Data(u8 *pCommandBuff, u16 CommandLength, u8 *pDataBuff, u16 DataLength)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, CS_Pin_x);
    // 发送命令(通常较短)
    for (u8 i = 0; i < CommandLength; i++) {
        SPI_I2S_SendData(SPI1, pCommandBuff[i]);
        ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }
    // 发送数据
    //  初始化发送DMA
    SPI_Helper_DMAInit(DMA1_Channel3, DataLength, DMA_DIR_PeripheralDST, pDataBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper_WaitforFinish(0, TimeOut_Ratio * DataLength + 100000);
    ErrCode = SPI_Helper_TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 1000000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, CS_Pin_x);

    // 清除 SPI_I2S_FLAG_RXNE 标志
    SPI1->DR;

    return ErrCode;
}

int8_t SPI_Helper_TimeOutAssert(uint16_t SPI_I2S_FLAG, FlagStatus Status, u32 nus)
{
    u32 TimeOut = nus;
    u8 err      = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG) != Status) {
        TimeOut--;
        if (TimeOut == 0) {
            // TODO:错误处理
            LED_On();
            printf("%s\t\n", "Error in:SPI_Helper_TimeOutAssert()");
            break;
        }
    }
    return err;
}

int8_t SPI_Helper_WaitforFinish(u8 Flag, u32 nus)
{
    u32 TimeOut = nus;
    u8 err      = 0;
    if (Flag == 0) {
        while (SPI_Helper_DMA2Send_Finish_Flag != 1) {
            TimeOut--;
            delay_ms(1);
            if (TimeOut == 0) {
                err = 1;
                LED_On();
                printf("%s\t\n", "error in :SPI_Helper_WaitforFinish()");
                break;
            }
        }
        SPI_Helper_DMA2Send_Finish_Flag = 0;
    } else {
        while (SPI_Helper_Rcv2DMA_Finish_Flag != 1) {
            TimeOut--;
            delay_ms(1);
            if (TimeOut == 0) {
                err = 1;
                LED_On();
                printf("%s\t\n", "error in :SPI_Helper_WaitforFinish()");
                break;
            }
        }
        SPI_Helper_Rcv2DMA_Finish_Flag = 0;
    }
    return err;
}