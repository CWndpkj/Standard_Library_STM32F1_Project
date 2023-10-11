#include <stm32f10x.h>
#include "PWM.h"

void PMW_Init()
{
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    // 初始化GPIO口
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct); 
    ////设置时钟源位内部时钟
    TIM_InternalClockConfig(TIM2);
    // 时基单元初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;       // 一分频
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_Period            = 100-1;              // 自动计数器达到这个值后自动重装计数器加一
    TIM_TimeBaseInitStruct.TIM_Prescaler         = 72 - 1;              // 分频器  @10KHz
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;                  // 高级定时器才有的
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM2,ENABLE);

    TIM_OCInitTypeDef TIM_OC2InitStruct;
    TIM_OCStructInit(&TIM_OC2InitStruct);
    TIM_OC2InitStruct.TIM_OCMode=TIM_OCMode_PWM1;
    TIM_OC2InitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
    TIM_OC2InitStruct.TIM_OutputState=TIM_OutputState_Enable;
    TIM_OC2InitStruct.TIM_Pulse=0;
    TIM_OC2Init(TIM2,&TIM_OC2InitStruct);
}

void PWM_SetFreq(uint16_t Freq_Hz)
{
    TIM_PrescalerConfig(TIM2,720000/Freq_Hz-1,TIM_PSCReloadMode_Immediate);
}

void PWM_SetDuty(uint8_t Duty)
{
    TIM_SetCompare2(TIM2,Duty);
}