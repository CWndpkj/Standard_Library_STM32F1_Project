#ifndef _SPI_HELPER_H_
#define _SPI_HELPER_H_
int8_t SPI_Helper_Init();
int8_t SPI_Helper_NVICInit();
int8_t SPI_Helper_DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u16 BufferSize, u32 Diraction, u8 *MemoryBaseAddr);
int8_t SPI_Helper_WriteLen(uint16_t CS_Pinx, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t SPI_Helper_ReadLen(uint16_t CS_Pinx, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint8_t SPI_Helper_SwapLen(uint16_t CS_Pinx, uint8_t reg_addr, u8 *pSendBuff, u8 *pRcvBuff, u8 length);
int8_t SPI_Helper_TimeOutAssert(uint16_t SPI_I2S_FLAG, FlagStatus Status);
int8_t SPI_Helper_WaitforFinish();

#endif // !_SPI_HELPER_H_
