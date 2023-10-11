#include <stm32f10x.h>
#include "key.h"
#include "LED.h"
void Key_Init(void)
{
    LED_Init();
    // 开启GPIO,AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPD; // 上拉输入
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // 初始化中断
    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line    = EXTI_Line1 | EXTI_Line3;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
    // 初始化nvic
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 2,2
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = EXTI1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = EXTI3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStruct);
}

// 设置中断函数

void EXTI1_IRQHandler(void)
{
    // 判断中断标志位
    if (EXTI_GetITStatus(EXTI_Line1) == SET) {
        LED_On();
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) == SET) {
        LED_Off();
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}