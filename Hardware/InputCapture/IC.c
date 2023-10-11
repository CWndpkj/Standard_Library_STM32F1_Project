#include <stm32f10x.h>
#include "IC.h"

void IC_Init()
{
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 初始化GPIO口
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    ////设置时钟源位内部时钟
    TIM_InternalClockConfig(TIM3);
    // 时基单元初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;       // 一分频
    TIM_TimeBaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_Period        = 65536 - 1;          // 自动计数器达到这个值后自动重装计数器加一
    TIM_TimeBaseInitStruct.TIM_Prescaler     = 72-1;                 // 分频器  @1MHz
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM3, ENABLE);

    // 配置输入捕获单元
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICInitStruct.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter    = 0x00;
    TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_PWMIConfig(TIM3, &TIM_ICInitStruct);

    // 配置从模式清零ARR
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
}

uint16_t IC_GetFreq()
{
    return 1000000 / (TIM_GetCapture1(TIM3)+1);
}

uint16_t IC_GetDuty()
{
    return (TIM_GetCapture2(TIM3)+1) * 100 /(TIM_GetCapture1(TIM3)+1);
}