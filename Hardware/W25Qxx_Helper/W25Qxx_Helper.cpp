#include <stm32f10x.h>
#include "W25Qxx_Helper.h"
#include "SPI_Helper.h"
#include "delay.h"
#include "LED.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

W25Qxx_Helper::W25Qxx_Helper(const uint16_t CS_Pin_x)
    : m_SPI_W25Qxx(CS_Pin_x)
{
}

W25Qxx_Helper::~W25Qxx_Helper()
{
}

int8_t W25Qxx_Helper::Init()
{
    this->m_SPI_W25Qxx.Init();
    u8 DeviceID = this->ReadDeviceID();
    printf("%d", DeviceID);
    switch (DeviceID) {
        case 0x13:
            this->m_W25Qxx_Type = W25Q80;
            break;
        case 0x14:
            this->m_W25Qxx_Type = W25Q16;
            break;
        case 0x15:
            this->m_W25Qxx_Type = W25Q32;
            break;
        case 0x16:
            this->m_W25Qxx_Type = W25Q64;
            break;
        case 0x17:
            this->m_W25Qxx_Type = W25Q128;
            break;
        case 0x18:
            this->m_W25Qxx_Type = W25Q256;
            break;
    }
    return 0;
}

int8_t W25Qxx_Helper::DisableWrite()
{
    int8_t ErrCode = 0;
    u8 Command     = 0x04;
    ErrCode        = this->m_SPI_W25Qxx.WriteByte(Command);
    return ErrCode;
}
int8_t W25Qxx_Helper::EnableWrite()
{
    int8_t ErrCode = 0;
    u8 Command     = 0x06;
    ErrCode        = this->m_SPI_W25Qxx.WriteByte(Command);
    return ErrCode;
}
u8 W25Qxx_Helper::ReadStatusReg(u8 reg_ID)
{
    u8 Command;
    u8 RcvBuff;
    switch (reg_ID) {
        case 1:
            Command = 0x05;
            break;
        case 2:
            Command = 0x35;
            break;
        case 3:
            Command = 0x15;
            break;
        default:
            Command = 0x05;
            break;
    }
    this->m_SPI_W25Qxx.WriteThenRead(&Command, 1, &RcvBuff, 1);
    return RcvBuff;
}
int8_t W25Qxx_Helper::Read(u32 reg_addr, u8 *pBuff, u16 length)
{
    int8_t ErrCode = 0;
    u8 Command[5];

    Command[0] = 0x03;
    if (this->m_W25Qxx_Type == W25Q256) {
        Command[1] = (reg_addr >> 24) & 0xFF;
        Command[2] = (reg_addr >> 16) & 0xFF;
        Command[3] = (reg_addr >> 8) & 0xFF;
        Command[4] = reg_addr & 0xFF;
        ErrCode    = this->m_SPI_W25Qxx.WriteThenRead(Command, 5, pBuff, length);

    } else {
        Command[1] = (reg_addr >> 16) & 0xFF;
        Command[2] = (reg_addr >> 8) & 0xFF;
        Command[3] = reg_addr & 0xFF;
        ErrCode    = this->m_SPI_W25Qxx.WriteThenRead(Command, 4, pBuff, length);
    }
    return ErrCode;
}

int8_t W25Qxx_Helper::Erase(u32 reg_addr)
{
    u8 ErrCode = 0;
    u8 Command[4];
    Command[0] = 0x20;
    if (this->m_W25Qxx_Type == W25Q256)
        for (u8 i = 0; i < 4; i++) {
            Command[i + 1] = reg_addr & ((u32)0xFF000000 >> i * 8);
        }
    else
        for (u8 i = 0; i < 3; i++) {
            Command[i + 1] = reg_addr & ((u32)0xFF0000 >> i * 8);
        }

    // 使能写
    this->EnableWrite();

    if (this->m_W25Qxx_Type == W25Q256) {
        ErrCode = this->m_SPI_W25Qxx.WriteLen(Command, 5);
    } else {
        ErrCode = this->m_SPI_W25Qxx.WriteLen(Command, 4);
    }
    // 擦除1sector时间不大于400ms
    this->WaitForBusy(1000);
    // 失能写
    this->DisableWrite();
    return ErrCode;
}

int8_t W25Qxx_Helper::Write(u32 reg_addr, u8 *pBuff, int32_t length)
{
    u8 ErrCode = 0;

    // 计算偏移量
    u32 sector, sector_offset, page; // sector 0~15,block 0~127(W25Q64) 0~255(W25Q128) 0~511(W25Q256) page 0~256
    // 计算sector用于擦除
    sector        = reg_addr / 4096;
    sector_offset = reg_addr % 4096;
    // 计算page用于换页
    page = reg_addr % 256;

    u32 pReg     = reg_addr;
    int32_t pLen = length;
    // 擦除操作
    if (length < 4096) {
        // 擦除该页即可
        ErrCode = this->Erase(pReg);
    } else {
        while (pLen > 0) {
            ErrCode = this->Erase(pReg);
            pLen -= 4096;
            pReg += 4096;
        }
    }

    // 如果写入的数据没有越过该页,直接全部写入
    if (length <= 256 - page) {
        ErrCode = this->WritePage(reg_addr, pBuff, length);
    } else // 分多次
    {
        ErrCode = this->WritePage(reg_addr, pBuff, 256 - page);
        length -= 256 - page;
        reg_addr += 256 - page;
        pBuff += 256 - page;

        while (length > 0) {
            u16 write_len;
            if (length > 256)
                write_len = 256;
            else
                write_len = length;
            ErrCode = this->WritePage(reg_addr, pBuff, write_len);
            length -= 256;
            reg_addr += 256;
            pBuff += 256;
        }
    }

    return ErrCode;
}

/**
 * @brief W25Qxx 页写函数,
 * @param reg_addr 写入地址
 * @param pBuff    写入的数据
 * @param length   写入的长度
 * @return uint8_t 返回0写入成功
 * @note   该函数没有对要写入的区进行擦除,写入前必须确认已经擦除
 */
uint8_t W25Qxx_Helper::WritePage(u32 reg_addr, u8 *pBuff, u16 length)
{
    u8 ErrCode = 0;
    u8 Command[5];
    Command[0] = 0x02;
    if (this->m_W25Qxx_Type == W25Q256) {
        Command[1] = (reg_addr >> 24) & 0xFF;
        Command[2] = (reg_addr >> 16) & 0xFF;
        Command[3] = (reg_addr >> 8) & 0xFF;
        Command[4] = reg_addr & 0xFF;
    } else {
        Command[1] = (reg_addr >> 16) & 0xFF;
        Command[2] = (reg_addr >> 8) & 0xFF;
        Command[3] = reg_addr & 0xFF;
    }

    // 开启写使能
    this->EnableWrite();
    if (this->m_W25Qxx_Type == W25Q256) {
        ErrCode = this->m_SPI_W25Qxx.WriteCommand_Data(Command, 5, pBuff, length);

    } else {
        ErrCode = this->m_SPI_W25Qxx.WriteCommand_Data(Command, 4, pBuff, length);
    }
    this->WaitForBusy(1000);
    // 关闭写使能
    this->DisableWrite();
    return ErrCode;
}

u8 W25Qxx_Helper::ReadDeviceID()
{
    u8 Command[4] = {0xAB, 0xFF, 0xFF, 0xFF};
    u8 RcvBuff;
    this->m_SPI_W25Qxx.WriteThenRead(Command, 4, &RcvBuff, 1);
    return RcvBuff;
}

int8_t W25Qxx_Helper::WaitForBusy(u16 nms)
{
    u16 TimeOut = nms;
    u8 err      = 0;
    while ((this->ReadStatusReg(1) & 0x01) == 1) {
        TimeOut--;
        delay_ms(1);
        if (TimeOut == 0) {
            // TODO:错误处理
            LED_On();
            printf("%s\t\n", "Error in: W25Qxx_WaitForBusy()");
            break;
        }
    }
    return err;
}

#ifdef __cplusplus
}
#endif // DEBUG