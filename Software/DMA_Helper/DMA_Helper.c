#include <stm32f10x.h>
#include "DMA_Helper.h"

uint16_t DMA_Helper_Size;
void DMA_Helper_Init(uint32_t ADDR_Des,uint32_t ADDR_Src,uint16_t Size)
{
    //开启时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize=Size;
    DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;//方向
    DMA_InitStruct.DMA_M2M=DMA_M2M_Enable;//软件触发
    DMA_InitStruct.DMA_MemoryBaseAddr=ADDR_Des;//储存器地址
    DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//以字节宽度转运
    DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//储存器地址是否自增
    DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;//循环自动重装或普通模式
    DMA_InitStruct.DMA_PeripheralBaseAddr=ADDR_Src;//外设地址
    DMA_InitStruct.DMA_PeripheralDataSize=DMA_MemoryDataSize_Byte;//宽度位Byte
    DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
    DMA_InitStruct.DMA_Priority=DMA_Priority_Medium;//优先级
    DMA_Init(DMA1_Channel1,&DMA_InitStruct);
    
    DMA_Helper_Size=Size;
}

void DMA_Helper_Transfer()
{
    DMA_Cmd(DMA1_Channel1,DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1,DMA_Helper_Size);
    DMA_Cmd(DMA1_Channel1,ENABLE);
    while (DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);
    DMA_ClearFlag(DMA1_FLAG_TC1);
}