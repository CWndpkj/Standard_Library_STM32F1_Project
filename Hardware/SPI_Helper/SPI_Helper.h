#ifndef _SPI_HELPER_H_
#define _SPI_HELPER_H_

int8_t SPI_Helper_Init(uint16_t GPIO_Pin_x);

int8_t SPI_Helper_WriteLen(uint8_t *pSendBuff, uint16_t length);

int8_t SPI_Helper_WriteByte(uint8_t pSendBuff);

int8_t SPI_Helper_ReadLen(uint8_t *pRcvBuff, uint16_t length);

int8_t SPI_Helper_ReadByte(uint8_t *pRcvBuff);

int8_t SPI_Helper_SwapLen(u8 *pSendBuff, u8 *pRcvBuff, u8 length);

int8_t SPI_Helper_SwapByte(u8 pSendBuff, u8 *pRcvBuff);

int8_t SPI_Helper_WriteThenRead(u8 *pSendBuff, u16 WriteLength, u8 *pRcvBuff, u16 ReadLength);

int8_t SPI_Helper_WriteCommand_Data(u8 *pCommandBuff, u16 CommandLength, u8 *pDataBuff, u16 DataLength);

#endif // !_SPI_HELPER_H_
