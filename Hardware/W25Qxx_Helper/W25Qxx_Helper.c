#include <stm32f10x.h>
#include "W25Qxx_Helper.h"
#include "SPI_Helper.h"
#include "delay.h"
#include "LED.h"
#include <stdio.h>

#define W25Qxx_Cs_Pin GPIO_Pin_6

typedef enum {
    W25Q80,
    W25Q16,
    W25Q32,
    W25Q64,
    W25Q128,
    W25Q256
} W25Qxx_TypeDef;

W25Qxx_TypeDef W25Qxx_Type;
uint8_t W25Qxx_WritePage(u32 reg_addr, u8 *pBuff, u16 length);
int8_t W25Qxx_WaitForBusy(u16 nms);

int8_t W25Qxx_Helper_Init()
{
    SPI_Helper_Init(W25Qxx_Cs_Pin);
    u8 DeviceID = W25Qxx_ReadDeviceID();
    switch (DeviceID) {
        case 0x13:
            W25Qxx_Type = W25Q80;
            break;
        case 0x14:
            W25Qxx_Type = W25Q16;
            break;
        case 0x15:
            W25Qxx_Type = W25Q32;
            break;
        case 0x16:
            W25Qxx_Type = W25Q64;
            break;
        case 0x17:
            W25Qxx_Type = W25Q128;
            break;
        case 0x18:
            W25Qxx_Type = W25Q256;
            break;
    }
    return 0;
}

int8_t W25Qxx_Helper_DisableWrite()
{
    int8_t ErrCode = 0;
    u8 Command     = 0x04;
    SPI_Helper_Start(W25Qxx_Cs_Pin);
    ErrCode = SPI_Helper_WriteLen(&Command, 1);
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    return ErrCode;
}
int8_t W25Qxx_Helper_EnableWrite()
{
    int8_t ErrCode = 0;
    u8 Command     = 0x06;
    SPI_Helper_Start(W25Qxx_Cs_Pin);
    ErrCode = SPI_Helper_WriteLen(&Command, 1);
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    return ErrCode;
}
u8 W25Qxx_ReadStatusReg(u8 reg_ID)
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
    SPI_Helper_Start(W25Qxx_Cs_Pin);
    SPI_Helper_WriteThenRead(&Command, &RcvBuff, 1, 1);
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    return RcvBuff;
}
int8_t W25Qxx_Helper_Read(u32 reg_addr, u8 *pBuff, u16 length)
{
    int8_t ErrCode = 0;
    u8 Command[5];

    // 等待Busy为0
    Command[0] = 0x03;
    if (W25Qxx_Type == W25Q256) {
        Command[1] = (reg_addr >> 24) & 0xFF;
        Command[2] = (reg_addr >> 16) & 0xFF;
        Command[3] = (reg_addr >> 8) & 0xFF;
        Command[4] = reg_addr & 0xFF;
    } else {
        Command[1] = (reg_addr >> 16) & 0xFF;
        Command[2] = (reg_addr >> 8) & 0xFF;
        Command[3] = reg_addr & 0xFF;
    }

    SPI_Helper_Start(W25Qxx_Cs_Pin);
    if (W25Qxx_Type == W25Q256) {
        if (SPI_Helper_WriteLen(Command, 5))
            ErrCode = 1;
    } else {
        if (SPI_Helper_WriteLen(Command, 4))
            ErrCode = 1;
    }
    if (SPI_Helper_ReadLen(pBuff, length))
        ErrCode = 2;
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    return ErrCode;
}

int8_t W25Qxx_Erase(u32 reg_addr)
{
    u8 ErrCode = 0;
    u8 Command[4];

    Command[0] = 0x20;

    if (W25Qxx_Type == W25Q256)
        for (u8 i = 0; i < 4; i++) {
            Command[i + 1] = reg_addr & ((u32)0xFF000000 >> i * 8);
        }
    else
        for (u8 i = 0; i < 3; i++) {
            Command[i + 1] = reg_addr & ((u32)0xFF0000 >> i * 8);
        }

    // 使能写
    W25Qxx_Helper_EnableWrite();

    SPI_Helper_Start(W25Qxx_Cs_Pin);
    if (W25Qxx_Type == W25Q256) {
        if (SPI_Helper_WriteLen(Command, 5))
            ErrCode = 1;
    } else {
        if (SPI_Helper_WriteLen(Command, 4))
            ErrCode = 1;
    }
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    //擦除时间不大于400ms
    W25Qxx_WaitForBusy(1000);
    // 失能写
    W25Qxx_Helper_DisableWrite();
    return ErrCode;
}

int8_t W25Qxx_Helper_Write(u32 reg_addr, u8 *pBuff, int32_t length)
{
    u8 ErrCode = 0;

    // 计算偏移量
    u32 sector, sector_offset, page; // sector 0~15,block 0~127(W25Q64) 0~255(W25Q128) 0~511(W25Q256) page 0~256
    // 计算sector用于擦除
    sector        = reg_addr / 4096;
    sector_offset = reg_addr % 4096;
    // 计算page用于换页
    page = reg_addr % 256;

    // 擦除操作
    if (length < 4096 - sector_offset) {
        // 擦除该页即可
        ErrCode = W25Qxx_Erase(sector);
    } else {
        ErrCode = W25Qxx_Erase(sector);
        length -= 4096 - sector_offset;
        sector++;
        while (length > 0) {
            ErrCode = W25Qxx_Erase(sector++);
            length -= 4096;
        }
    }

    // 如果写入的数据没有越过该页,直接全部写入
    if (length <= 256 - page) {
        ErrCode = W25Qxx_WritePage(reg_addr, pBuff, length);
    } else // 分多次
    {
        ErrCode = W25Qxx_WritePage(reg_addr, pBuff, 256 - page);
        length -= 256 - page;
        reg_addr += 256 - page;
        pBuff += 256 - page;

        while (length > 0) {
            u16 write_len;
            if (length > 256)
                write_len = 256;
            else
                write_len = length;
            ErrCode = W25Qxx_WritePage(reg_addr, pBuff, write_len);
            length -= 256;
            reg_addr += 256;
            pBuff += 256;
        }

        // while (length > 0) {
        //     ErrCode = W25Qxx_WritePage((block << 16 | sector << 12 | page), pBuff + (256 - page), 256);
        //     page    = 0;
        //     sector += 1;
        //     if (sector == 16) {
        //         sector = 0;
        //         block += 1;
        //     }
        //     ErrCode = W25Qxx_WritePage((block << 16 | sector << 12 | page), pBuff + (256 - page), 256);
        //     length -= 256;
        // }
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
uint8_t W25Qxx_WritePage(u32 reg_addr, u8 *pBuff, u16 length)
{
    u8 ErrCode = 0;
    u8 Command[5];
    Command[0] = 0x02;
    if (W25Qxx_Type == W25Q256) {
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
    W25Qxx_Helper_EnableWrite();
    SPI_Helper_Start(W25Qxx_Cs_Pin);
    if (W25Qxx_Type == W25Q256) {
        ErrCode = SPI_Helper_WriteLen(Command, 5);
    } else {
        ErrCode = SPI_Helper_WriteLen(Command, 4);
    }
    SPI_Helper_WriteLen(pBuff, length);
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    W25Qxx_WaitForBusy(1000);
    // 关闭写使能
    W25Qxx_Helper_DisableWrite();
    return ErrCode;
}

u8 W25Qxx_ReadDeviceID()
{
    u8 Command[4] = {0xAB, 0xFF, 0xFF, 0xFF};
    u8 RcvBuff;
    SPI_Helper_Start(W25Qxx_Cs_Pin);
    SPI_Helper_WriteThenRead(Command, &RcvBuff, 4, 1);
    SPI_Helper_Stop(W25Qxx_Cs_Pin);
    return RcvBuff;
}

int8_t W25Qxx_WaitForBusy(u16 nms)
{
    u16 TimeOut = nms;
    u8 err      = 0;
    while (W25Qxx_ReadStatusReg(1) & 0x01 == 1) {
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