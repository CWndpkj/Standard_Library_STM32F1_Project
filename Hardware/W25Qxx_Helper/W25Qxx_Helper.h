#ifndef _W25Qxx_HELPER_H_
#define _W25Qxx_HELPER_H_
int8_t W25Qxx_Helper_Init();
u8 W25Qxx_ReadDeviceID();
int8_t W25Qxx_Helper_Write(u32 reg_addr, u8 *pBuff, int32_t length);
int8_t W25Qxx_Helper_Read(u32 reg_addr, u8 *pBuff, u16 length);

#endif // !_W25Qxx_HELPER_H_
