#include <stm32f10x.h>
#include "RTC_Helper.h"

u8 RTC_Helper_Init()
{
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);

    // 使能更改BKB寄存器
    PWR_BackupAccessCmd(ENABLE);

    // 选择RTC时钟源
    RCC_LSEConfig(RCC_LSE_ON); // 开启外部低速晶振
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET)
        ;
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);

    // 等待时钟同步
    RTC_WaitForSynchro();
    //设置预分频器@1ms
    RTC_SetPrescaler(72000-1);
    RTC_WaitForLastTask();
    return 0;
}

u8 RTC_Helper_Getms(unsigned long *timestamp)
{
    *timestamp=(unsigned long)RTC_GetCounter();
    return 0;
}