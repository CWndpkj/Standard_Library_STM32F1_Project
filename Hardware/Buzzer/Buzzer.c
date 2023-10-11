#include "stm32f10x.h"
#include "Buzzer.h"

/**
 * @brief TODO:更为Buzzer连接的端口
 * 
 */
#define Buzzer_Port GPIO_Pin_1

void Buzzer_Init(void)
{
    //使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStruct.GPIO_Pin=Buzzer_Port;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    //初始化
    GPIO_Init(GPIOA,&GPIO_InitStruct);
}

void Buzzer_On(void)
{
    GPIO_SetBits(GPIOA,Buzzer_Port);
}

void Buzzer_Off(void)
{
    GPIO_ResetBits(GPIOA,Buzzer_Port);
}

void Buzzer_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOA,Buzzer_Port)){
        GPIO_ResetBits(GPIOA,Buzzer_Port);
    }
    else 
        GPIO_SetBits(GPIOA,Buzzer_Port);
}