#ifndef _I2C_HELPER_H_
#define _I2C_HELPER_H_
u8 I2C_Helper_Init();

u8 I2C_Helper_ReadByte(u8 DeviceAddr, u8 RegAddr, u8 *pBuff);
u8 I2C_Helper_WriteByte(u8 DeviceAddr, u8 RegAddr, u8 Buff);
u8 I2C_Helper_Write_Len(u8 DeviceAddr, u8 RegAddr, u16 length, u8 *pBuff);
u8 I2C_Helper_Read_Len(u8 DeviceAddr, u8 RegAddr, u16 length, u8 *pBuff);
u8 I2C_Helper_TimeOutAssert(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);
u8 I2C_Helper_WaitBUSY();
u8 I2C_Helper_SlaveDeviceReset();
#endif // !_I2C_HELPER_H_
