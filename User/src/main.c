#include "stm32f10x.h"
#include "delay.h"
#include "OLED.h"
#include "Timer.h"
#include "USART_Helper.h"
#include "MPU6500.h"
#include "stdio.h"
#include "AHT20.h"
#include "RTC_Helper.h"
#include "inv_mpu.h"
#include "I2C_Helper.h"
#include "HWInterface.h"
#include "AnoPTv8.h"
#include "WS2812.h"
#include "BMP280_Helper.h"
#include "W25Qxx_Helper.h"
void HardWareInit()
{
    delay_ms(500);
    //  WS2812_Init();
    LED_Init();
    LED_Off();
    USART_Helper_Init();
    OLED_Init();
    // AHT20_Init();
    //   RTC_Helper_Init();
    //   OLED_ShowNum(1,1,MPU6500_DMP_Init(),1);
    // MPU6500_Init();

    // BMP280_Init();

    // printf("%s", "hello world");
    W25Qxx_Helper_Init();
}

uint8_t databuf[14];
void AnoPTv8TxFrameF1(float pitch, float yaw, float roll)
{
    databuf[0]  = BYTE0(pitch);
    databuf[1]  = BYTE1(pitch);
    databuf[2]  = BYTE2(pitch);
    databuf[3]  = BYTE3(pitch);
    databuf[4]  = BYTE0(yaw);
    databuf[5]  = BYTE1(yaw);
    databuf[6]  = BYTE2(yaw);
    databuf[7]  = BYTE3(yaw);
    databuf[8]  = BYTE0(roll);
    databuf[9]  = BYTE1(roll);
    databuf[10] = BYTE2(roll);
    databuf[11] = BYTE3(roll);
    AnoPTv8SendBuf(ANOPTV8DEVID_ALL, 0xF1, databuf, 12);
}
u32 i;
float value[3];
int main()
{
    HardWareInit();
    u8 ReadBuff[10]  = {0};
    u8 WriteBuff[10] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};
    W25Qxx_Helper_Write(0x0, WriteBuff, 10);
    W25Qxx_Helper_Read(0x0, ReadBuff, 10);
    OLED_ShowHexNum(1, 1, ReadBuff[5], 2);

    // AHT20_value AHT_value;
    //   MPU6500_Value MPU_value;
    //   MPU6500_StartAquire();
    // OLED_ShowString(1, 1, "hello world");

    // SPI_Helper_WriteLen( 0x06, NULL, 0);
    // u8 ADDR[3] = {0x00, 0x00, 0x00};
    // SPI_Helper_WriteLen(GPIO_Pin_6, 0x02, ADDR, 3);
    // u8 Data[20] = {0x20, 0x80, 0x30};

    // u8 ReadCommend[3];
    // ReadCommend[0] = 0x00;
    // ReadCommend[1] = 0x00;
    // ReadCommend[2] = 0x00;
    // u8 Buff[20];
    // SPI_Helper_ReadLen(GPIO_Pin_6, 0x03, ReadCommend,);

    // OLED_ShowHexNum(3, 1, MPU6500_GetDeviceID(), 2);
    while (1) {
        // AnoPTv8HwTrigger1ms();
        //  OLED_ShowHexNum(2, 1, MPU6500_GetAddr(), 2);
        // OLED_ShowNum(2, 1, i++, 5);
        // MPU6500_dmp_get_euler_angle(NULL, NULL, value, value + 1, value + 2);
        // AnoPTv8TxFrameF1(value[0], value[1], value[2]);
        // AnoPTv8HwTrigger1ms();
        // printf("%s:%f\t\n", "pitch:", value[0]);
        // printf("%s:%f\t\n", "roll:", value[1]);
        // printf("%s:%f\t\n", "yaw:", value[2]);

        // AHT20_GetValue(&AHT_value);
        //    MPU6500_GetValue(&MPU_value);
        // printf("%s", "湿度:");
        // printf("%f\n", AHT_value.Humidity);
        // printf("%s", "温度:");
        // printf("%f\n", AHT_value.Temperature);
        //    printf("%s", "MPU输出:");
        //    printf("%f\t\n", MPU6500_GetTemperature());
        //   MPU6500_Send2Host(MPU_value.ACCEL_XOUT, MPU_value.ACCEL_YOUT, MPU_value.ACCEL_ZOUT,
        //                   MPU_value.GYRO_XOUT, MPU_value.GYRO_YOUT, MPU_value.GYRO_ZOUT);
        //   mpu_get_temperature(&temp, NULL);
        //   OLED_ShowNum(2, 1, temp, 10);

        // Delay_ms(100);
        // u8 i = 0 - 1;
        // while (i--) {
        // }
        LED_Turn();

        delay_ms(300);
    }
}