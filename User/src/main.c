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

void HardWareInit()
{
    Delay_ms(500);
    LED_Init();
    LED_Off();
    USART_Helper_Init();
    // OLED_Init();
    //  AHT20_Init();
    //    RTC_Helper_Init();
    //   OLED_ShowNum(1,1,MPU6500_DMP_Init(),1);
    MPU6500_Init();
}

uint8_t databuf[13];
void AnoPTv8TxFrameF1(float pitch, float yaw, float roll)
{
    databuf[0]  = BYTE0(pitch);
    databuf[1]  = BYTE1(pitch);
    databuf[2]  = BYTE2(pitch);
    databuf[3]  = BYTE3(pitch);
    databuf[4]  = BYTE1(pitch);
    databuf[5]  = BYTE0(yaw);
    databuf[6]  = BYTE1(yaw);
    databuf[7]  = BYTE2(yaw);
    databuf[8]  = BYTE3(yaw);
    databuf[9]  = BYTE0(roll);
    databuf[10] = BYTE1(roll);
    databuf[11] = BYTE2(roll);
    databuf[12] = BYTE3(roll);

    AnoPTv8SendBuf(ANOPTV8DEVID_ALL, 0xF1, databuf, 13);
}
u32 i;
float value[3];
int main()
{
    HardWareInit();
    // AHT20_value AHT_value;
    //  MPU6500_Value MPU_value;
    //  MPU6500_StartAquire();
    //  OLED_ShowString(1, 1, "hello world");

    while (1) {
        // AnoPTv8HwTrigger1ms();
        //  OLED_ShowHexNum(2, 1, MPU6500_GetAddr(), 2);
        // OLED_ShowNum(2, 1, i++, 5);
        MPU6500_dmp_get_euler_angle(NULL, NULL, value, value + 1, value + 2);
        AnoPTv8TxFrameF1(value[0], value[1], value[2]);
        AnoPTv8SendDevInfo(0x12);
        AnoPTv8HwTrigger1ms();
        // printf("%s:%f\t\n", "pitch:", value[0]);
        // printf("%s:%f\t\n", "roll:", value[1]);
        // printf("%s:%f\t\n", "yaw:", value[2]);

        // AHT20_GetValue(&AHT_value);
        //  MPU6500_GetValue(&MPU_value);
        //  printf("%s", "湿度:");
        //  printf("%f\t\n", AHT_value.Humidity);
        //  printf("%s", "温度:");
        //  printf("%f\t\n", AHT_value.Temperature);
        //   printf("%s", "MPU输出:");
        //   printf("%f\t\n", MPU6500_GetTemperature());
        //  MPU6500_Send2Host(MPU_value.ACCEL_XOUT, MPU_value.ACCEL_YOUT, MPU_value.ACCEL_ZOUT,
        //                  MPU_value.GYRO_XOUT, MPU_value.GYRO_YOUT, MPU_value.GYRO_ZOUT);
        //  mpu_get_temperature(&temp, NULL);
        //  OLED_ShowNum(2, 1, temp, 10);

        // Delay_ms(100);
        // u8 i = 0 - 1;
        // while (i--) {
        // }
        LED_Turn();

        Delay_ms(1000);
    }
}