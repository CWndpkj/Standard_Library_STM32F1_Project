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
    this->Buff = new u8[256];
}

W25Qxx_Helper::~W25Qxx_Helper()
{
    delete[] this->Buff;
}

int8_t W25Qxx_Helper::Init()
{
    this->m_SPI_W25Qxx.Init();
    u8 DeviceID = this->ReadDeviceID();
    // printf("W25QxxDeviceID:%d\t\n", DeviceID);
    switch (DeviceID) {
        case 0x13: {
            this->m_W25Qxx_Type = W25Q80;
            EndSector           = 0x000F0000;
            break;
        }
        case 0x14: {
            this->m_W25Qxx_Type = W25Q16;
            this->EndSector     = 0x001F0000;
            break;
        }
        case 0x15: {
            this->m_W25Qxx_Type = W25Q32;
            this->EndSector     = 0x003F0000;
            break;
        }
        case 0x16: {
            this->m_W25Qxx_Type = W25Q64;
            this->EndSector     = 0x007F0000;
            break;
        }
        case 0x17: {
            this->m_W25Qxx_Type = W25Q128;
            this->EndSector     = 0x00FF0000;
            break;
        }
        case 0x18: {
            this->m_W25Qxx_Type = W25Q256;
            this->EndSector     = 0x1FFF0000;
            break;
        }
        default: {
            this->m_W25Qxx_Type = W25Q64;
            this->EndSector     = 0x007F0000;
            break;
        }
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
    // 使能写
    this->EnableWrite();

    if (this->m_W25Qxx_Type == W25Q256) {
        Command[1] = (reg_addr >> 24) & 0xFF;
        Command[2] = (reg_addr >> 16) & 0xFF;
        Command[3] = (reg_addr >> 8) & 0xFF;
        Command[4] = reg_addr & 0xFF;
        ErrCode    = this->m_SPI_W25Qxx.WriteLen(Command, 5);
    } else {
        Command[1] = (reg_addr >> 16) & 0xFF;
        Command[2] = (reg_addr >> 8) & 0xFF;
        Command[3] = reg_addr & 0xFF;
        ErrCode    = this->m_SPI_W25Qxx.WriteLen(Command, 4);
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
    u16 sector, sector_offset, page_offset; // sector 0~15,block 0~127(W25Q64) 0~255(W25Q128) 0~511(W25Q256) page 0~256
    // 计算sector用于擦除
    sector        = reg_addr / 4096;
    sector_offset = reg_addr % 4096;
    // 计算page用于换页
    page_offset = reg_addr % 256;

    int32_t Len = length;
    // 计算要擦除的扇区数
    u16 sector_num_to_erase = 0;
    Len -= 4096 - sector_offset;
    sector_num_to_erase++;
    while (Len > 0) {
        Len -= 4096;
        sector_num_to_erase++;
    }

    // 计算要写入的页数
    Len                   = length;
    u16 page_num_to_write = 0;
    Len -= 256 - page_offset;
    page_num_to_write++;
    while (Len >= 256) {
        Len -= 256;
        page_num_to_write++;
    }
    // 最后小于256byte的数据长度单独写入
    u8 last_write_len;
    if (Len > 0) {
        last_write_len = Len;
        page_num_to_write++;
    } else {
        last_write_len = length;
    }

    // u32 first_sector                   = sector;                           // 第一个扇区的其起始地址
    // u32 first_sector_before_write_addr = reg_addr;                         // 第一个扇区在写入地址之前的末地址
    // u32 first_sector_after_write_addr  = reg_addr + length;                // 第一个扇区在写入地址之后的末地址
    u16 end_sector = sector + sector_num_to_erase - 1; // 最后一个扇区的起始地址
    // u32 end_sector_after_write_addr    = end_sector + Len;                 // 最后一个扇区的在写入地址之后的末地址
    u16 end_sector_write_num = (length - (4096 - sector_offset)) % 4096;
    // 开始擦除
    //  在擦除第一个扇区时,有可能起始地址并不在一个扇区的起始地址,
    //  这时擦除该扇区将会影响到前面的数据,因此需要先读取保存

    // 对第一个扇区进行特殊处理
    if (length < 4096 - sector_offset) {
        // 复制扇区数据
        ErrCode = this->MoveSectorToFinal(sector, reg_addr, reg_addr + length);
    } else {
        ErrCode = this->MoveSectorToFinal(sector, reg_addr, sector + 4096);
    }
    // 擦除第一个扇区
    ErrCode = this->Erase(sector);
    // 将数据移回来
    u32 SrcAddr = this->EndSector;
    u32 DstAddr = sector;
    for (u8 i = 0; i < 16; i++) {
        ErrCode = this->Read(SrcAddr, this->Buff, 256);
        ErrCode = this->WriteOnePage(DstAddr, this->Buff, 256);
        SrcAddr += 256;
        DstAddr += 256;
    }
    if (sector_num_to_erase > 1) {
        // 擦除中间扇区
        for (u8 i = 0; i < sector_num_to_erase - 2; i++) {
            ErrCode = this->Erase(sector + 4096 + i * 4096);
        }
        // 对最后一个扇区的处理
        // 复制扇区数据
        ErrCode = this->MoveSectorToFinal(end_sector, end_sector, end_sector + end_sector_write_num);
        // 擦除末一个扇区
        ErrCode = this->Erase(end_sector);
        // 将数据移回来
        SrcAddr = this->EndSector;
        DstAddr = end_sector;
        for (u8 i = 0; i < 16; i++) {
            ErrCode = this->Read(SrcAddr, this->Buff, 256);
            ErrCode = this->WriteOnePage(DstAddr, this->Buff, 256);
            SrcAddr += 256;
            DstAddr += 256;
        }
    }

    // 写入数据
    u32 Addr = reg_addr;
    u8 *Ptr  = pBuff;
    // 写入第一个页
    ErrCode = this->WriteOnePage(Addr, Ptr, 256 - page_offset);
    Addr += 256 - page_offset;
    Ptr += 256 - page_offset;
    if (page_num_to_write > 1) {
        // 写入中间页
        for (u16 i = 0; i < page_num_to_write - 2; i++) {
            this->WriteOnePage(Addr, Ptr, 256);
            Addr += 256;
            Ptr += 256;
        }
        // 写入最后一页
        ErrCode = this->WriteOnePage(Addr, Ptr, last_write_len);
    }
    return ErrCode;
}

/**
 * @brief W25Qxx 将一个扇区的数据转运到最后一个扇区,并在转运时将 star_addr ~ end_addr
 * 之间的数据填充为 0xFF
 * @param sector_addr 扇区起始地址
 * @param start_addr   标记起始地址
 * @param end_addr    标记结束地址
 * @return int8_t
 */
int8_t W25Qxx_Helper::MoveSectorToFinal(u32 sector_addr, u32 start_addr, u32 end_addr)
{
    int8_t ErrCode = 0;
    /*扇区起始   页的结束|   | start_addr                 end_addr|   |新的页的起始      |扇区尾
     * |---------------|---|-------------------------------------|---|-----------------|
     * |<--start_len---|-->|<----------------------------------->|<-|----end_len------>|
     * |<--需要保留数据-|-->|<-----------填充0xFF---------------->|<--|--需要保留数据---->|
     *                 |<->|start_len_last_page_remain           |<->|end_len_first_page_extra
     */
    u16 start_len                   = start_addr - sector_addr;
    u16 end_len                     = 4096 - end_addr;
    u16 start_len_page_num_to_write = start_len / 256; // start_len中需要写多少个页
    u16 start_len_last_page_remain  = start_len % 256; // start_len中最后一个页剩余的数据长度
    u16 end_len_page_num_to_write   = end_len / 256;   // end_len中需要写多少个页
    u16 end_len_first_page_extra    = end_len % 256;   // end_len中第一个页前面多出来长度

    this->Erase(this->EndSector);
    // 移动first_len
    u32 pSrcAddr = sector_addr;
    u32 pDstAddr = EndSector;
    for (u32 i = 0; i < start_len_page_num_to_write; i++) {
        ErrCode = this->Read(pSrcAddr, Buff, 256);
        ErrCode = this->WriteOnePage(pDstAddr, Buff, 256);
        pSrcAddr += 256;
        pDstAddr += 256;
    }
    if (start_len_last_page_remain != 0) {
        ErrCode = this->Read(pSrcAddr, Buff, start_len_last_page_remain);
        ErrCode = this->WriteOnePage(pDstAddr, Buff, start_len_last_page_remain);
        pSrcAddr += start_len_last_page_remain;
        pDstAddr += start_len_last_page_remain;
    }
    // 移动second_len
    pSrcAddr = end_addr;
    pDstAddr = EndSector + end_addr - sector_addr;
    if (end_len_first_page_extra != 0) {
        ErrCode = this->Read(pSrcAddr, Buff, end_len_first_page_extra);
        ErrCode = this->WriteOnePage(pDstAddr, Buff, end_len_first_page_extra);
        pSrcAddr += end_len_first_page_extra;
        pDstAddr += end_len_first_page_extra;
    }
    for (u16 i = 0; i < end_len_page_num_to_write; i++) {
        ErrCode = this->Read(pSrcAddr, Buff, 256);
        ErrCode = this->WriteOnePage(pDstAddr, Buff, 256);
        pSrcAddr += 256;
        pDstAddr += 256;
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
uint8_t W25Qxx_Helper::WriteOnePage(u32 reg_addr, u8 *pBuff, u16 length)
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