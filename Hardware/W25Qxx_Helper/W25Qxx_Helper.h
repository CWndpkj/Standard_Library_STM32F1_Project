#ifndef _W25Qxx_HELPER_H_
#define _W25Qxx_HELPER_H_
#include "SPI_Helper.h"
#ifdef __cplusplus
extern "C" {
#endif // DEBUG
typedef enum {
    W25Q80,
    W25Q16,
    W25Q32,
    W25Q64,
    W25Q128,
    W25Q256
} W25Qxx_TypeDef;

class W25Qxx_Helper
{
private:
    W25Qxx_TypeDef m_W25Qxx_Type;
    SPI_Helper m_SPI_W25Qxx;
    int8_t EnableWrite();
    int8_t DisableWrite();
    u8 ReadStatusReg(u8 reg_ID);
    u8 ReadDeviceID();
    int8_t Erase(u32 reg_addr);
    uint8_t WritePage(u32 reg_addr, u8 *pBuff, u16 length);
    int8_t WaitForBusy(u16 nms);

public:
    W25Qxx_Helper(const uint16_t CS_pin_x);
    ~W25Qxx_Helper();
    int8_t Init();
    u8 DeviceID();
    int8_t Write(u32 reg_addr, u8 *pBuff, int32_t length);
    int8_t Read(u32 reg_addr, u8 *pBuff, u16 length);
};

#ifdef __cplusplus
}
#endif // DEBUG
#endif // !_W25Qxx_HELPER_H_
