#include "stm32f10x.h"
#include "LightSensor.h"

/**
 * @brief TODO:更改为光线传感器的连接端口
 * 
 */
#define LIGHTSENSOR_PORT GPIO_Pin_1
void LightSensor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
}

uint8_t LightSensor_GetStatus(void)
{
    return !GPIO_ReadInputDataBit(GPIOA,LIGHTSENSOR_PORT);
}