#include <stm32f10x.h>
#include "Timer.h"
#include "LED.h"
void Timer_Init()
{
    LED_Init();
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 初始化GPIO口
    // GPIO_InitTypeDef GPIO_InitStruct;
    // GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    // GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_2;
    // GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_Init(GPIOA, &GPIO_InitStruct);
    ////设置时钟源位内部时钟
    TIM_InternalClockConfig(TIM2);
    // 配置为外部时钟
    // TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0x00);
    // 时基单元初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;       // 一分频
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_Period            = 10000;               // 自动计数器达到这个值后自动重装计数器加一
    TIM_TimeBaseInitStruct.TIM_Prescaler         = 7200 - 1;           // 分频器
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;                  // 高级定时器才有的
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    // 初始化中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 开启定时器中断

    // nvic初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStruct);
}

void Timer_Start()
{
    TIM_Cmd(TIM2, ENABLE);
}

void Timer_Stop()
{
    TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_FLAG_Update) == SET) {
        LED_Turn();
        // 清楚标志位
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
    }
}