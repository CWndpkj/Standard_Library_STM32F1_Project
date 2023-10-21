#ifndef _I2C_DMA_H_
#define _I2C_DMA_H_
#ifdef __cplusplus
extern "C" {
#endif // DEBUG
u8 I2C_DMA_Init();
u8 I2C_DMA_ReadByte(u8 DeviceAddr, u8 RegAddr, u8 *pBuff);
u8 I2C_DMA_WriteByte(u8 DeviceAddr, u8 RegAddr, u8 Buff);
u8 I2C_DMA_Write_Len(u8 DeviceAddr, u8 RegAddr, u32 length, u8 *pBuff);
u8 I2C_DMA_Read_Len(u8 DeviceAddr, u8 RegAddr, u32 length, u8 *pBuff);
u8 I2C_DMA_TimeOutAssert(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);
u8 I2C_DMA_WaitBUSY();
u8 I2C_DMA_SlaveDeviceReset();
u8 I2C_DMA_WaitforFinish();
#ifdef __cplusplus
}
#endif // DEBUG
#endif // !_I2C_DMA_H_
