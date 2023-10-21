#include <stm32f10x.h>
#ifndef _SPI_HELPER_H_
#define _SPI_HELPER_H_

#define TimeOut_Ratio 1000
#ifdef __cplusplus
extern "C" {
#endif // DEBUG
class SPI_Helper
{
    friend void DMA1_Channel3_IRQHandler();
    friend void DMA1_Channel2_IRQHandler();

private:
    uint16_t m_CS_Pin_x;

    static volatile u8 Initialed_Flag;
    static volatile u8 DMA2Send_Finish_Flag;
    static volatile u8 Rcv2DMA_Finish_Flag;
    static int8_t DMAInit(DMA_Channel_TypeDef *DMAy_Channelx, u32 BufferSize, u32 Diraction, u8 *MemoryBaseAddr);
    static int8_t TimeOutAssert(u16 SPI_I2S_FLAG, FlagStatus Status, u32 nus);
    static int8_t WaitforFinish(u8 Flag, u32 nus);
    static int8_t SPIInit(uint16_t SPI_DataSize_x);
    static int8_t NVICInit();

public:
    SPI_Helper(const uint16_t GPIO_Pin_x);
    ~SPI_Helper();

    int8_t Init();

    int8_t WriteLen(uint8_t *pSendBuff, uint32_t length);

    int8_t WriteByte(uint8_t pSendBuff);

    int8_t WriteHalfWord(uint16_t SendBuff);

    int8_t ReadLen(uint8_t *pRcvBuff, uint32_t length);

    int8_t ReadByte(uint8_t *pRcvBuff);

    int8_t ReadHalfWord(uint16_t *RcvBuff);

    int8_t CircleWriteByte(uint8_t value, uint32_t length);

    int8_t CircleWriteHalfWord(uint16_t value, uint32_t length);

    int8_t SwapLen(uint8_t *pSendBuff, uint8_t *pRcvBuff, uint32_t length);

    int8_t SwapByte(uint8_t pSendBuff, uint8_t *pRcvBuff);

    int8_t WriteThenRead(uint8_t *pSendBuff, uint32_t WriteLength, uint8_t *pRcvBuff, uint32_t ReadLength);

    int8_t WriteCommand_Data(uint8_t *pCommandBuff, uint32_t CommandLength, uint8_t *pDataBuff, uint32_t DataLength);
};
#ifdef __cplusplus
}
#endif // DEBUG
#endif // !_SPI_HELPER_H_
