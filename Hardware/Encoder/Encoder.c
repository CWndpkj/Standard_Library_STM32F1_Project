#include <stm32f10x.h>
#include "Encoder.h"

void Encoder_Init()
{
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 初始化GPIO口
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period=65536-1;
    TIM_TimeBaseInitStruct.TIM_Prescaler=1-1;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
    // 配置滤波器
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel  = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel  = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

    //使能TIM
    TIM_Cmd(TIM2,ENABLE);
}

int16_t Encoder_GetCounter()
{
    return TIM_GetCounter(TIM2);
}