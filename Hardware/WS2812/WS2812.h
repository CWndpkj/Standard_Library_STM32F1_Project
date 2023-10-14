#pragma once
#include <stm32f10x.h>


void WS2812_Init();

void WS2812_SetRGBData(u8 (*arr)[3]);

void WS2812_SetCircular(u8 NewState, int32_t Declipse);
