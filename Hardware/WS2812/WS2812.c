/**
 * @file WS2812.c
 * @author pkjinfinity (pkjinfinity@outlook.com)
 * @brief  WS2812驱动函数   采用 DMA+Timer+PWM产生时序

 * @version 0.1
 * @date 2023-10-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stm32f10x.h>
#include "WS2812.h"
#include "delay.h"
#include <stdio.h>

#define LED_NUM 16
#define PWM_1   59
#define PWM_0   29

u8 (*RGB_Data)[3];
u8 Buff[8 * LED_NUM * 3 + 1];
u16 WS2812_Circular_Declipse;

void WS2812_GPIOSetMode(GPIOMode_TypeDef GPIO_Mode);
void WS2812_PWM_Timer_Init();

void WS2812_Init()
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // GPIO_SetBits(GPIOA, GPIO_Pin_1); // 先向寄存器写入值再开启GPIO口,否则会产生一个低电平的波纹
    WS2812_GPIOSetMode(GPIO_Mode_AF_PP);

    WS2812_PWM_Timer_Init();
}

void WS2812_GPIOSetMode(GPIOMode_TypeDef GPIO_Mode)
{
    // 初始化首先延迟,产生一个大于280us的低电平信号用于初始化
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief 为产生PWM信号初始化时基单元
 */
void WS2812_PWM_Timer_Init()
{
    TIM_Cmd(TIM2, DISABLE);

    TIM_InternalClockConfig(TIM2);
    // 初始化Time_Base
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period            = 90 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler         = 1 - 1; // 预分频系数
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;     // 重复次数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_OCInitTypeDef TIM_OC2InitStruct;
    TIM_OCStructInit(&TIM_OC2InitStruct);
    TIM_OC2InitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OC2InitStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2InitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC2InitStruct.TIM_Pulse       = 0; // 设置占空比
    TIM_OC2Init(TIM2, &TIM_OC2InitStruct);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 开启TIM DMA
    TIM_DMAConfig(TIM2, TIM_DMABase_CCR2, TIM_DMABurstLength_1Transfer);
    TIM_SelectCCDMA(TIM2, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);

    //  初始化DMA
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_BufferSize         = 8 * LED_NUM * 3 + 1;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)Buff;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (TIM2->DMAR);
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    // 开启DMA
    DMA_Cmd(DMA1_Channel2, ENABLE);

    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Channel2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);
}
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
        DMA_Cmd(DMA1_Channel2, DISABLE);
        TIM_Cmd(TIM2, DISABLE);

        DMA_ClearITPendingBit(DMA1_IT_TC2); // 清除标志位
    }
}

/**
 * @brief 为产生灯带循环效果初始化定时器单元
 */
void WS2812_Circular_Timer_Init()
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period            = WS2812_Circular_Declipse * 10;
    TIM_TimeBaseInitStruct.TIM_Prescaler         = 7200; // 预分频系数 10000Hz
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;    // 重复次数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    // 开启定时器中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    // 配置NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);
}
void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        // 对显示的数组进行移位
        u8 temp[3];
        for (u8 i = 0; i < 3; i++) {
            temp[i] = RGB_Data[0][i];
        }
        for (u8 i = 0; i < LED_NUM - 1; i++) {
            for (u8 j = 0; j < 3; j++) {
                RGB_Data[i][j] = RGB_Data[i + 1][j];
            }
        }
        for (u8 i = 0; i < 3; i++) {
            RGB_Data[LED_NUM - 1][i] = temp[i];
        }
        WS2812_SetRGBData(RGB_Data);

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}
/**
 * @brief 将RGB数据转换成对应的占空比
 */
void WS2812_DataParse()
{
    for (u8 i = 0; i < LED_NUM; i++) {
        for (u8 j = 0; j < 3; j++) {
            //***GRB------>RGB*****  // 一个灯需要24bit , 8*3
            u8 RGB_Sequences;
            if (j == 0)
                RGB_Sequences = 1;
            else if (j == 1)
                RGB_Sequences = 0;
            else
                RGB_Sequences = 2;
            /*******************/
            for (u8 k = 0; k < 8; k++) {
                if (RGB_Data[i][RGB_Sequences] & (0x80 >> k)) {
                    Buff[i * 24 + j * 8 + k] = PWM_1;
                } else {
                    Buff[i * 24 + j * 8 + k] = PWM_0;
                }
            }
        }
    }
    Buff[8 * LED_NUM * 3] = 0xFF;
}
void WS2812_SetRGBData(u8 (*arr)[3])
{
    RGB_Data = arr;
    WS2812_DataParse();
    TIM2->CCR2 = 0x00;
    TIM_Cmd(TIM2, ENABLE);
    delay_us(300);
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

void WS2812_SetCircular(u8 NewState, int32_t Declipse)
{
    WS2812_Circular_Declipse = Declipse;
    WS2812_Circular_Timer_Init();
    TIM_Cmd(TIM3, (FunctionalState)NewState);
}