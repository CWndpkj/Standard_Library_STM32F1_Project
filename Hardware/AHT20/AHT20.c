#include <stm32f10x.h>
#include "AHT20.h"
#include "I2C_DMA.h"
#include "delay.h"
#define AHT20_ADDR 0x38
u8 AHT20_Init()
{
    I2C_DMA_Init();
    u8 Command[2] = {0x08, 0x00}; // 初始化命令
    I2C_DMA_Write_Len(AHT20_ADDR, 0xBE, 2, Command);
    return 0;
}
u8 ATH20_Read_Status() // 读取AHT10的状态寄存器
{
    uint8_t Byte_first;
    I2C_DMA_ReadByte(AHT20_ADDR, 0x00, &Byte_first);
    return Byte_first;
}
u8 AHT20_StartAquire()
{
    u8 Command[2] = {0x33, 0x00};
    return I2C_DMA_Write_Len(AHT20_ADDR, 0xAC, 2, Command);
}

u8 AHT20_GetValue(AHT20_value *value)
{
    u8 Buff[7] = {0, 0, 0, 0, 0, 0, 0};
    u32 tmp    = 0;
    if (0 == AHT20_StartAquire()) {
        delay_ms(100);
        //  读取结果
        if (0 == I2C_DMA_Read_Len(AHT20_ADDR, 0x00, 7, Buff)) {
            tmp |= Buff[1];
            tmp <<= 8;
            tmp |= Buff[2];
            tmp <<= 8;
            tmp |= Buff[3];
            tmp >>= 4;
            value->Humidity = (float)tmp / 10485.76;

            tmp = 0;
            tmp |= (Buff[3] & 0x0F);
            tmp <<= 8;
            tmp |= Buff[4];
            tmp <<= 8;
            tmp |= Buff[5];
            value->Temperature = (float)tmp / 1048576 * 200 - 50;
            return 0;
        }
        return 1;
    }
    return 1;
}