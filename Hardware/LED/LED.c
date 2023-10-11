#include "stm32f10x.h"
#include "LED.h"

/**
 * @brief TODO:更改为LED连接端口
 * 
 */
#define LED_Port GPIO_Pin_13

void LED_Init(void)
{
    //使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStruct.GPIO_Pin=LED_Port;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    //初始化
    GPIO_Init(GPIOC,&GPIO_InitStruct);

    LED_Off();
}

void LED_On(void)
{
    GPIO_ResetBits(GPIOC,LED_Port);
}

void LED_Off(void)
{
    GPIO_SetBits(GPIOC,LED_Port);
}

void LED_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOC,LED_Port)){
        GPIO_ResetBits(GPIOC,LED_Port);
    }
    else 
        GPIO_SetBits(GPIOC,LED_Port);
}