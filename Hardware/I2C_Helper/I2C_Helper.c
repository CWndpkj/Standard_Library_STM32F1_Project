#include <stm32f10x.h>
#include "I2C_Helper.h"
#include "delay.h"
#include "OLED.h"
#include "LED.h"
#include "stdio.h"
u8 I2C_Helper_Init()
{
    LED_Init();
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
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
    I2C_InitStruct.I2C_ClockSpeed          = 400000;                      // 856000;                       // I2C频率,<=400KHz
    I2C_InitStruct.I2C_DutyCycle           = I2C_DutyCycle_2;              // 占空比
    I2C_InitStruct.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStruct.I2C_OwnAddress1         = 0x88; // 自身作为从设备时的地址
    I2C_Init(I2C2, &I2C_InitStruct);
    // 开启I2C
    I2C_Cmd(I2C2, ENABLE);
    // 开启I2C中断
    I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
    // I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);

    // 配置中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = I2C2_ER_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = I2C2_EV_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStruct);

    return 0;
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

u8 I2C_Helper_WriteByte(u8 DeviceAddr, u8 RegAddr, u8 Buff)
{
    I2C_Helper_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 1;
    }

    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_SendData(I2C2, RegAddr);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    I2C_SendData(I2C2, Buff);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 4;
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

u8 I2C_Helper_Write_Len(u8 DeviceAddr, u8 RegAddr, u16 length, u8 *pBuff)
{

    I2C_Helper_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 1;
    }

    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_SendData(I2C2, RegAddr);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    for (u16 i = 0; i < length; i++) {
        I2C_SendData(I2C2, pBuff[i]);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 4;
        }
    }

    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 5;
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

u8 I2C_Helper_ReadByte(u8 DeviceAddr, u8 RegAddr, u8 *pBuff)
{
    I2C_Helper_WaitBUSY(); // 等待主线空闲

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);

        return 1;
    }
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 2;
    }

    I2C_SendData(I2C2, RegAddr);

    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 3;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 4;
    }
    I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Receiver);
    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 5;
    }

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);

    if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
        return 6;
    }
    *pBuff = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 0;
}

u8 I2C_Helper_Read_Len(u8 DeviceAddr, u8 RegAddr, u16 length, u8 *pBuff)
{
    if (length == 1) {
        return I2C_Helper_ReadByte(DeviceAddr, RegAddr, pBuff);
    } else {
        u8 err = 0;

        I2C_Helper_WaitBUSY(); // 等待主线空闲

        I2C_GenerateSTART(I2C2, ENABLE);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 1;
        }
        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Transmitter);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 2;
        }

        I2C_SendData(I2C2, RegAddr);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 3;
        }

        I2C_GenerateSTART(I2C2, ENABLE);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 4;
        }

        I2C_Send7bitAddress(I2C2, DeviceAddr, I2C_Direction_Receiver);
        if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
            return 5;
        }

        for (u16 i = 0; i < length; i++) {
            if (I2C_Helper_TimeOutAssert(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
                I2C_GenerateSTOP(I2C2, ENABLE);
                I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT | I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
                return 6;
            }
            pBuff[i] = I2C_ReceiveData(I2C2);
            if (i == length - 2) {
                I2C_AcknowledgeConfig(I2C2, DISABLE);
                I2C_GenerateSTOP(I2C2, ENABLE);
            }
        }

        I2C_AcknowledgeConfig(I2C2, ENABLE);
        return err;
    }
}

u8 I2C_Helper_TimeOutAssert(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    u16 TimeOut = 10000;
    u8 err      = 0;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS) {
        TimeOut--;
        if (TimeOut == 0) {
            LED_On();
            printf("%s\t\n", "error in :I2C_Helper_TimeOutAssert()");
            break;
        }
    }
    return err;
}

u8 I2C_Helper_WaitBUSY()
{
    u32 TimeOut = 10000;
    u8 err      = 0;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET) {
        TimeOut--;
        if (TimeOut == 0) {
            LED_On();
            printf("%s\t\n", "error in :I2C_Helper_WaitBUSY()");
            I2C_Helper_SlaveDeviceReset();
            I2C_DeInit(I2C2);
            I2C_Helper_Init();

            break;
        }
    }
    return err;
}

u8 I2C_Helper_SlaveDeviceReset()
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