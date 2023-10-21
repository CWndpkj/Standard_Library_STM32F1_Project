#include <stm32f10x.h>
#include <I2C_DMA.h>

#include <stm32f10x.h>
#include "I2C_DMA.h"
#include "delay.h"
#include "OLED.h"
#include "LED.h"
#include "stdio.h"
#ifdef __cplusplus
extern "C" {
#endif // DEBUG
u8 I2C_DMA_Transfer_Finish_Flag;

void I2C_DMA_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 DIR, u32 MemoryBaseAddr, u32 Priority);

u8 I2C_DMA_Init()
{
    LED_Init();
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD; // 复用开漏输出
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // 初始化I2C
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_Ack                 = I2C_Ack_Enable;               // 默认开启应答
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 七位地址模式
    I2C_InitStruct.I2C_ClockSpeed          = 400000;                       // 856000;                       // I2C频率,<=400KHz
    I2C_InitStruct.I2C_DutyCycle           = I2C_DutyCycle_2;              // 占空比
    I2C_InitStruct.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStruct.I2C_OwnAddress1         = 0x88; // 自身作为从设备时的地址
    I2C_Init(I2C2, &I2C_InitStruct);
    // 开启I2C
    I2C_Cmd(I2C2, ENABLE);
    // 开启I2C中断
    I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
    // I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);
    // 开启I2C_DMA
    I2C_DMACmd(I2C2, ENABLE);
    // 配置中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = I2C2_ER_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);

    // NVIC_InitStruct.NVIC_IRQChannel                   = I2C2_EV_IRQn;
    // NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    // NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    // NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    // NVIC_Init(&NVIC_InitStruct);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);

    return 0;
}
/**
 * @brief I2C_Tx中断函数
 *
 */
void DMA1_Channel4_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        // 置完成标志位
        I2C_DMA_Transfer_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}
/**
 * @brief I2C_Rx中断函数
 *
 */
void DMA1_Channel5_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC5) == SET) {
        // 关闭I2C在DMA传输完成后自动产生停止条件
        I2C_DMALastTransferCmd(I2C2, DISABLE);

        // 置完成标志位
        I2C_DMA_Transfer_Finish_Flag = 1;

        DMA_ClearITPendingBit(DMA1_IT_TC5);
    }
}
// void I2C2_EV_IRQHandler(void)
// {
//     if (I2C_GetITStatus(I2C2, I2C_IT_TXE) == SET) {
//         printf("%s", "I2C_IT_TXE");
//         I2C_ClearITPendingBit(I2C2, I2C_IT_TXE);
//     }
//     if (I2C_GetITStatus(I2C2, I2C_IT_RXNE) == SET) {
//         printf("%s", "I2C_IT_RXNE");
//         I2C_ClearITPendingBit(I2C2, I2C_IT_RXNE);
//     }
// }

void I2C2_ER_IRQHandler(void)
{
    if (I2C_GetITStatus(I2C2, I2C_IT_ADDR) == SET) {
        printf("%s", "I2C_IT_ADDR");
        I2C_ClearITPendingBit(I2C2, I2C_IT_ADDR);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_TIMEOUT) == SET) {
        printf("%s", "I2C_IT_TIMEOUT");
        I2C_ClearITPendingBit(I2C2, I2C_IT_TIMEOUT);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_PECERR) == SET) {
        printf("%s", "I2C_IT_PECERR");
        I2C_ClearITPendingBit(I2C2, I2C_IT_PECERR);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_OVR) == SET) {
        printf("%s", "I2C_IT_OVR");
        I2C_ClearITPendingBit(I2C2, I2C_IT_OVR);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_AF) == SET) {
        printf("%s", "I2C_IT_AF");
        I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_ARLO) == SET) {
        printf("%s", "I2C_IT_ARLO");
        I2C_ClearITPendingBit(I2C2, I2C_IT_ARLO);
    }
    if (I2C_GetITStatus(I2C2, I2C_IT_BERR) == SET) {
        printf("%s", "I2C_IT_BERR");
        I2C_ClearITPendingBit(I2C2, I2C_IT_BERR);
    }
}

u8 I2C_DMA_WriteByte(u8 DeviceAddr, u8 RegAddr, u8 Buff)
{
    I2C_DMA_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 1;
    }

    I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Transmitter);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_SendData(I2C2, RegAddr);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    I2C_SendData(I2C2, Buff);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 4;
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

u8 I2C_DMA_Write_Len(u8 DeviceAddr, u8 RegAddr, u32 length, u8 *pBuff)
{
    I2C_DMA_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Transmitter);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    I2C_SendData(I2C2, RegAddr);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 4;
    }

    // for (u16 i = 0; i < length; i++) {
    //     I2C_SendData(I2C2, pBuff[i]);
    //     if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
    //         I2C_GenerateSTOP(I2C2, ENABLE);
    //         I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
    //         return 4;
    //     }
    // }

    // 改为DMA
    I2C_DMA_DMAInit(DMA1_Channel4, length, DMA_DIR_PeripheralDST, (u32)pBuff, DMA_Priority_VeryHigh);
    // 等待DMA传输完成
    I2C_DMA_WaitforFinish();

    // 等待数据发送完成
    //  判断传输完成标志位
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 5;
    }

    // 产生结束信号
    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

void I2C_DMA_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 DIR, u32 MemoryBaseAddr, u32 Priority)
{
    DMA_Cmd(DMAy_Channelx, DISABLE);
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = BufferSize;
    DMA_InitStruct.DMA_DIR                = DIR;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_MemoryBaseAddr     = MemoryBaseAddr;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = Priority;
    DMA_Init(DMAy_Channelx, &DMA_InitStruct);
    DMA_Cmd(DMAy_Channelx, ENABLE);
}

u8 I2C_DMA_ReadByte(u8 DeviceAddr, u8 RegAddr, u8 *pBuff)
{
    I2C_DMA_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);

        return 1;
    }
    I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Transmitter);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_SendData(I2C2, RegAddr);

    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 4;
    }
    I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Receiver);
    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 5;
    }

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);

    if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 6;
    }
    *pBuff = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 0;
}

u8 I2C_DMA_Read_Len(u8 DeviceAddr, u8 RegAddr, u32 length, u8 *pBuff)
{
    if (length == 1) {
        return I2C_DMA_ReadByte(DeviceAddr, RegAddr, pBuff);
    } else {
        I2C_DMA_WaitBUSY(); // 等待主线空闲

        I2C_GenerateSTART(I2C2, ENABLE);
        if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 2;
        }
        I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Transmitter);
        if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 3;
        }

        I2C_SendData(I2C2, RegAddr);
        if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 4;
        }

        I2C_GenerateSTART(I2C2, ENABLE);
        if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 5;
        }

        I2C_Send7bitAddress(I2C2, DeviceAddr << 1, I2C_Direction_Receiver);
        if (I2C_DMA_TimeOutAssert(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 6;
        }

        // 改为DMA
        // 开启DMA接收完成最后一个字节后自动发送非应答信号
        I2C_DMALastTransferCmd(I2C2, ENABLE);

        I2C_DMA_DMAInit(DMA1_Channel5, length, DMA_DIR_PeripheralSRC, (u32)pBuff, DMA_Priority_High);

        // 等待DMA传输完成
        if (I2C_DMA_WaitforFinish()) {
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 7;
        }

        I2C_GenerateSTOP(I2C2, ENABLE);

        return 0;
    }
}

u8 I2C_DMA_TimeOutAssert(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    u16 TimeOut = 10000;
    u8 err      = 0;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS) {
        TimeOut--;
        if (TimeOut == 0) {
            err = 1;
            LED_On();
            printf("%s\t\n", "error in :I2C_DMA_TimeOutAssert()");
            break;
        }
    }
    return err;
}

u8 I2C_DMA_WaitforFinish()
{
    u16 TimeOut = 1000;
    u8 err      = 0;
    while (I2C_DMA_Transfer_Finish_Flag != 1) {
        TimeOut--;
        delay_ms(1);
        if (TimeOut == 0) {
            err = 1;
            LED_On();
            printf("%s\t\n", "error in :I2C_DMA_WaitforFinish()");
            break;
        }
    }
    I2C_DMA_Transfer_Finish_Flag = 0;
    return err;
}

u8 I2C_DMA_WaitBUSY()
{
    u32 TimeOut = 10000;
    u8 err      = 0;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET) {
        TimeOut--;
        if (TimeOut == 0) {
            err = 1;
            LED_On();
            printf("%s\t\n", "error in :I2C_DMA_WaitBUSY()");
            I2C_DMA_SlaveDeviceReset();
            I2C_DeInit(I2C2);
            I2C_DMA_Init();

            break;
        }
    }
    return err;
}

u8 I2C_DMA_SlaveDeviceReset()
{
    I2C_Cmd(I2C2, DISABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);

    for (u8 i = 0; i < 9; i++) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
    }

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_Cmd(I2C2, ENABLE);
    return 0;
}
#ifdef __cplusplus
}
#endif // DEBUG