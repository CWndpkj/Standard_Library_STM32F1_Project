#ifndef _SPI_HELPER_H_
#define _SPI_HELPER_H_

int8_t SPI_Helper_Init(uint16_t CS_Pinx);

int8_t SPI_Helper_WriteThenRead(u8 *pSendBuff, u8 *pRcvBuff, u16 WriteLength, u16 ReadLength);

void SPI_Helper_Start(uint16_t CS_Pinx);

void SPI_Helper_Stop(uint16_t CS_Pinx);

int8_t SPI_Helper_WriteLen(uint8_t *reg_data, uint16_t length);

int8_t SPI_Helper_ReadLen(uint8_t *reg_data, uint16_t length);

uint8_t SPI_Helper_SwapLen(u8 *pSendBuff, u8 *pRcvBuff, u8 length);

int8_t SPI_Helper_WriteThenRead(u8 *pSendBuff, u8 *pRcvBuff, u16 WriteLength, u16 ReadLength);


#endif // !_SPI_HELPER_H_
