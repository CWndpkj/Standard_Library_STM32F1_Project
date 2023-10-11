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
void HardWareInit()
{
    Delay_ms(200);
    USART_Helper_Init();
    OLED_Init();
    //  AHT20_Init();
    //  RTC_Helper_Init();
    // OLED_ShowNum(1,1,MPU6500_DMP_Init(),1);
    MPU6500_Init();
    // MPU6500_DMP_Init();
}

u32 i;
float value[3];
int main()
{
    HardWareInit();
    // AHT20_value AHT_value;
    // MPU6500_Value MPU_value;
    // MPU6500_StartAquire();
    // OLED_ShowString(1, 1, "hello world");

    while (1) {
        // OLED_ShowHexNum(2, 1, MPU6500_GetAddr(), 2);
        OLED_ShowNum(2, 1, i++, 5);
        MPU6500_dmp_get_euler_angle(NULL, NULL, value, value + 1, value + 2);

        printf("%s:%f\t\n", "pitch:", value[0]);
        printf("%s:%f\t\n", "roll:", value[1]);
        printf("%s:%f\t\n", "yaw:", value[2]);

        // AHT20_GetValue(&AHT_value);
        // MPU6500_GetValue(&MPU_value);
        // printf("%s", "湿度:");
        //  printf("%f\t\n", AHT_value.Humidity);
        //  printf("%s", "温度:");
        //  printf("%f\t\n", AHT_value.Temperature);
        //  printf("%s", "MPU输出:");
        //  printf("%f\t\n", MPU6500_GetTemperature());
        // MPU6500_Send2Host(MPU_value.ACCEL_XOUT, MPU_value.ACCEL_YOUT, MPU_value.ACCEL_ZOUT,
        //                 MPU_value.GYRO_XOUT, MPU_value.GYRO_YOUT, MPU_value.GYRO_ZOUT);
        // mpu_get_temperature(&temp, NULL);
        // OLED_ShowNum(2, 1, temp, 10);

        Delay_ms(100);
    }
}