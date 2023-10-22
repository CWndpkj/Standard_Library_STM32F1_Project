/*******************************************
__CLZ(x): 计算无符号整数x的前导零位数。
__RBIT(x): 反转x的位顺序。
__REV(x): 反转x的字节顺序。
__REV16(x): 反转x的半字节顺序。
************************************************/
#include "SPI_Helper.h"
#include "LED.h"
// #include <stdio.h>
#include "delay.h"

#ifdef __cplusplus
extern "C" {
#endif

// 定义全局静态变量
volatile u8 SPI_Helper::DMA2Send_Finish_Flag = 0;
volatile u8 SPI_Helper::Rcv2DMA_Finish_Flag  = 0;
volatile u8 SPI_Helper::Initialed_Flag       = 0;
SPI_Helper::SPI_Helper(uint16_t GPIO_Pin_x)
{
    this->m_CS_Pin_x = GPIO_Pin_x;
}

SPI_Helper::~SPI_Helper()
{
}
int8_t SPI_Helper::Init()
{
    if (!SPI_Helper::Initialed_Flag) { // 仅初始化一次
        LED_Init();
        // 开启SPI时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        // 开启DMA时钟
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        // 开启GPIO时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
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

        SPIInit(SPI_DataSize_8b);

        // 开启SPI_DMA请求
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

        // 配置NVIC
        SPI_Helper::NVICInit();
        SPI_Helper::Initialed_Flag = 1;
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 初始化用于CS的GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin   = this->m_CS_Pin_x;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_SetBits(GPIOB, this->m_CS_Pin_x); // 初始设置为高电平
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    return 0;
}
int8_t SPI_Helper::SPIInit(uint16_t SPI_DataSize_x)
{
    // 关闭SPI
    SPI_Cmd(SPI1, DISABLE);
    // 初始化SPI
    SPI_InitTypeDef SPI_InitStruct;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // 波特率预分频系数
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;                    // SPI模式0
    SPI_InitStruct.SPI_CRCPolynomial     = 7;                               // CRC校验多项式
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_x;                  // 数据长度
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;                // 高位在前
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;                 // 主模式
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;                    // 软件NSS
    SPI_Init(SPI1, &SPI_InitStruct);
    // 开启SPI_DMA请求
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    // 开启SPI
    SPI_Cmd(SPI1, ENABLE);
    return 0;
}
int8_t SPI_Helper::NVICInit()
{
    // 开启DMA传输完成中断
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);
    return 0;
}

/**
 * @brief Tx_transfer_complete_cb
 */
void DMA1_Channel3_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
        // 置标志位
        SPI_Helper::DMA2Send_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC3);
    }
}

/**
 * @brief Rx_transfer_complete_cb
 */
void DMA1_Channel2_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
        // 置标志位
        SPI_Helper::Rcv2DMA_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

int8_t SPI_Helper::DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 Diraction, u8 *MemoryBaseAddr)
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
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (SPI1->DR); // 一定记得要加取地址符
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
int8_t SPI_Helper::WriteLen(uint8_t *pSendBuff, uint32_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 初始化DMA
    SPI_Helper::DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pSendBuff);

    // 等待DMA传输完成
    SPI_Helper::WaitforFinish(0, TimeOut_Ratio * length + 100000);
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)
    return ErrCode;
}

/**
 * @brief SPI 发送一个字节,在小数据发送上具有更高响应
 * @param pSendBuff 要发送的数据
 * @return int8_t
 */
int8_t SPI_Helper::WriteByte(uint8_t SendBuff)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 发送数据
    SPI_I2S_SendData(SPI1, SendBuff);
    // 等待最后一个数据发送完,也接收完
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)
    return ErrCode;
}
/**
 * @brief SPI发送半字数据
 *
 * @param SendBuff 要发送的数据
 * @return int8_t 返回0发送成功
 */
int8_t SPI_Helper::WriteHalfWord(uint16_t SendBuff)
{
    int8_t ErrCode = 0;
    SPIInit(SPI_DataSize_16b);
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);
    SPI_I2S_SendData(SPI1, SendBuff);
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);
    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)
    SPIInit(SPI_DataSize_8b);
    return ErrCode;
}

int8_t SPI_Helper::CircleWriteByte(uint8_t value, uint32_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 失能DMA
    DMA_Cmd(DMA1_Channel3, DISABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = length;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST; // 转运方向
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;       // mem关闭
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)&value;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte; // 大小Byte
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Disable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (SPI1->DR); // 一定记得要加取地址符
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);
    // 使能DMA
    DMA_Cmd(DMA1_Channel3, ENABLE);

    // 等待DMA传输完成
    SPI_Helper::WaitforFinish(0, TimeOut_Ratio * length + 100000);
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)
    return ErrCode;
}

int8_t SPI_Helper::CircleWriteHalfWord(uint16_t value, uint32_t length)
{
    int8_t ErrCode = 0;
    u16 Buff       = value;
    SPI_Helper::SPIInit(SPI_DataSize_16b);
    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);
    // 失能DMA
    DMA_Cmd(DMA1_Channel3, DISABLE);
    // 初始化DMA
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = length;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST; // 转运方向
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;       // mem关闭
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)&Buff;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Disable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (SPI1->DR); // 一定记得要加取地址符
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);
    // 使能DMA
    DMA_Cmd(DMA1_Channel3, ENABLE);

    // 等待DMA传输完成
    SPI_Helper::WaitforFinish(0, TimeOut_Ratio * length + 100000);
    // 等待最后一个数据发送完
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    // 此处必须读取一下SPI的接收寄存器以清除RXNE标志位,
    // 否则在其他函数中一启用DMA就会立马转运一个数据过去导致出错
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)
    SPI_Helper::SPIInit(SPI_DataSize_8b);
    return ErrCode;
}

/**
 * @brief SPI读取指定长度函数
 * @param pRcvBuff 读取到的数据放在哪
 * @param length   读取的的数据的长度
 * @return int8_t  0为读取成功,否则出错
 */
int8_t SPI_Helper::ReadLen(uint8_t *pRcvBuff, uint32_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 初始化DMA
    SPI_Helper::DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 发送缓冲数据产生时钟
    for (u16 i = 0; i < length; i++) {
        SPI_I2S_SendData(SPI1, 0xFF);
        // 等待发送完成
        ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }

    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(1, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    return ErrCode;
}

/**
 * @brief SPI 读取一个字节,读取少量数据具有更高的响应
 * @param pRcvBuff 读取到的数据存放的地址
 * @return int8_t  0为读取成功,否则失败
 */
int8_t SPI_Helper::ReadByte(uint8_t *pRcvBuff)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // SPI1->DR;

    SPI_I2S_SendData(SPI1, 0xFF);
    // 等待发送完成
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 1000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);
    // 读取数据
    *pRcvBuff = SPI_I2S_ReceiveData(SPI1);

    return ErrCode;
}
/**
 * @brief SPI读取一个半字数据
 *
 * @param RcvBuff 储存数据的指针
 * @return int8_t 返回表示成功
 */
int8_t SPI_Helper::ReadHalfWord(uint16_t *RcvBuff)
{
    int8_t ErrCode = 0;
    SPIInit(SPI_DataSize_16b);
    // 拉低CS
    GPIO_ResetBits(GPIOB, this->m_CS_Pin_x);
    SPI_I2S_SendData(SPI1, 0xFFFF);
    ErrCode  = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 10000);
    *RcvBuff = (uint16_t)SPI1->DR; // 读取DR
    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);
    SPIInit(SPI_DataSize_8b);
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
int8_t SPI_Helper::SwapLen(uint8_t *pSendBuff, uint8_t *pRcvBuff, uint32_t length)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 初始化DMA
    SPI_Helper::DMAInit(DMA1_Channel3, length, DMA_DIR_PeripheralDST, pSendBuff);
    SPI_Helper::DMAInit(DMA1_Channel2, length, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(0, TimeOut_Ratio * length + 10000);
    // 等待最后一个数据发送完,也接收完
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 10000);
    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(1, 100000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    return ErrCode;
}

/**
 * @brief SPI 交换一个字节,读取少量数据具有更高的响应
 *
 * @param pSendBuff 要发送的字节
 * @param pRcvBuff  接收数据的地址
 * @return int8_t   返回0成功,否则失败
 */
int8_t SPI_Helper::SwapByte(uint8_t pSendBuff, uint8_t *pRcvBuff)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    SPI_I2S_SendData(SPI1, pSendBuff);
    // 等待发送完成
    ErrCode   = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);
    *pRcvBuff = SPI_I2S_ReceiveData(SPI1);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    return ErrCode;
}

int8_t SPI_Helper::WriteThenRead(uint8_t *pSendBuff, uint32_t WriteLength, uint8_t *pRcvBuff, uint32_t ReadLength)
{
    int8_t ErrCode = 0;
    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);

    // 初始化发送DMA
    SPI_Helper::DMAInit(DMA1_Channel3, WriteLength, DMA_DIR_PeripheralDST, pSendBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(0, TimeOut_Ratio * WriteLength + 100000);
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);

    // 清除 SPI_I2S_FLAG_RXNE 标志
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)

    // 初始化接收DMA
    SPI_Helper::DMAInit(DMA1_Channel2, ReadLength, DMA_DIR_PeripheralSRC, pRcvBuff);
    // 发送填充产生时钟
    for (u32 i = 0; i < ReadLength; i++) {
        SPI_I2S_SendData(SPI1, 0xFF);
        // 等待数据发送完成
        ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }

    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(1, 10000);
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 100000);
    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    return ErrCode;
}

int8_t SPI_Helper::WriteCommand_Data(uint8_t *pCommandBuff, uint32_t CommandLength, uint8_t *pDataBuff, uint32_t DataLength)
{
    int8_t ErrCode = 0;

    // 拉低CS片选
    GPIO_ResetBits(GPIOB, m_CS_Pin_x);
    // 发送命令(通常较短)
    for (u8 i = 0; i < CommandLength; i++) {
        SPI_I2S_SendData(SPI1, pCommandBuff[i]);
        ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_TXE, SET, 10000);
    }
    // 发送数据
    //  初始化发送DMA
    SPI_Helper::DMAInit(DMA1_Channel3, DataLength, DMA_DIR_PeripheralDST, pDataBuff);
    // 等待DMA传输完成
    ErrCode = SPI_Helper::WaitforFinish(0, TimeOut_Ratio * DataLength + 100000);
    ErrCode = SPI_Helper::TimeOutAssert(SPI_I2S_FLAG_BSY, RESET, 1000000);

    // 拉高CS片选
    GPIO_SetBits(GPIOB, m_CS_Pin_x);

    // 清除 SPI_I2S_FLAG_RXNE 标志
    (void *)SPI1->DR; // 加上(void*是为了防止被编译器优化掉)

    return ErrCode;
}

int8_t SPI_Helper::TimeOutAssert(u16 SPI_I2S_FLAG, FlagStatus Status, u32 nus)
{
    u32 TimeOut = nus;
    u8 err      = 0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG) != Status) {
        TimeOut--;
        if (TimeOut == 0) {
            // TODO:错误处理
            LED_On();
            // printf("%s\t\n", "Error in:SPI_Helper_TimeOutAssert()");
            break;
        }
    }
    return err;
}

int8_t SPI_Helper::WaitforFinish(u8 Flag, u32 nus)
{
    u32 TimeOut = nus;
    u8 err      = 0;
    if (Flag == 0) {
        while (SPI_Helper::DMA2Send_Finish_Flag != 1) {
            TimeOut--;
            if (TimeOut == 0) {
                err = 1;
                LED_On();
                // printf("%s\t\n", "error in :SPI_Helper_WaitforFinish()");
                break;
            }
        }
        SPI_Helper::DMA2Send_Finish_Flag = 0;
    } else {
        while (SPI_Helper::Rcv2DMA_Finish_Flag != 1) {
            TimeOut--;
            if (TimeOut == 0) {
                err = 1;
                LED_On();
                // printf("%s\t\n", "error in :SPI_Helper_WaitforFinish()");
                break;
            }
        }
        SPI_Helper::Rcv2DMA_Finish_Flag = 0;
    }
    return err;
}

#ifdef __cplusplus
}
#endif