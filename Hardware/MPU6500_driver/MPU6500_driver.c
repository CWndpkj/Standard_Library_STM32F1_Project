#include "mpu6500_driver.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math_fun.h"
#include "math.h"

u8 MPU_EXTI_flag = 0; // MPU6500引脚中断

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {

    .orientation = {0, -1, 0,
                    1, 0, 0,
                    0, 0, 1}};

/*    Axis Transformation matrix
|r11  r12  r13| |vx|     |v'x|
|r21  r22  r23| |vy|  =  |v'y|
|r31  r32  r33| |vz|     |v'z|
v'x  = {(r11  * vx) +( r12 * vy) +( r13 * vz)}
v'y  = {(r21  * vx) +( r22 * vy) +( r23 * vz)}
v'z  = {(r31  * vx) +( r32 * vy) +( r33 * vz)}
*/

void MPU6500_Port_EXIT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStrue;
    EXTI_InitTypeDef EXTI_InitStrue;
    NVIC_InitTypeDef NVIC_InitStrue;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStrue.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStrue.GPIO_Pin  = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStrue);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

    EXTI_InitStrue.EXTI_Line    = EXTI_Line14;
    EXTI_InitStrue.EXTI_LineCmd = ENABLE;
    EXTI_InitStrue.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStrue.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStrue);

    NVIC_InitStrue.NVIC_IRQChannel                   = EXTI15_10_IRQn;
    NVIC_InitStrue.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStrue.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStrue);
}

void MPU6500_I2C_PORT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
}

void MPU6500_I2C_delay(void)
{
    u16 i;
    /*　
        下面的时间是通过逻辑分析仪测试得到的。
    工作条件：CPU主频72MHz ，MDK编译环境，1级优化

        循环次数为10时，SCL频率 = 205KHz
        循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us
        循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us
    */
    for (i = 0; i < 5; i++)
        ;
}

void MPU6500_I2C_start(void)
{
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetOut_Mode();

    MPU6500_I2C_SDA_WR = 1;
    MPU6500_I2C_SCL    = 1;
    MPU6500_I2C_delay();
    MPU6500_I2C_SDA_WR = 0;
    MPU6500_I2C_SCL    = 0;
    MPU6500_I2C_delay();
}

void MPU6500_I2C_stop(void)
{
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetOut_Mode();

    MPU6500_I2C_SDA_WR = 0;
    MPU6500_I2C_SCL    = 1;
    MPU6500_I2C_delay();
    MPU6500_I2C_SDA_WR = 1;
    MPU6500_I2C_delay();
}

u8 MPU6500_I2C_check_ack(void)
{
    u16 delay_count = 0;

    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetIn_Mode();
    MPU6500_I2C_SDA_WR = 1;
    MPU6500_I2C_SCL    = 1;
    MPU6500_I2C_delay();
    while (MPU6500_I2C_SDA_RE) {
        delay_count++;
        if (delay_count > 0x0fff) {
            MPU6500_I2C_stop();
            return 1;
        }
    }

    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_delay();

    return 0;
}

void MPU6500_I2C_ack(void)
{
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetOut_Mode();

    MPU6500_I2C_SDA_WR = 0;
    MPU6500_I2C_SCL    = 1;
    MPU6500_I2C_delay();
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_delay();
    MPU6500_I2C_SDA_WR = 1;
}

void MPU6500_I2C_NoAck(void)
{
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetOut_Mode();
    MPU6500_I2C_SDA_WR = 1;
    MPU6500_I2C_delay();
    MPU6500_I2C_SCL = 1;
    MPU6500_I2C_delay();
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_delay();
}

void MPU6500_I2C_write_char(u8 dat)
{
    u8 i = 0;

    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetOut_Mode();
    for (i = 0; i < 8; i++) {

        if (dat & 0x80)
            MPU6500_I2C_SDA_WR = 1;
        else
            MPU6500_I2C_SDA_WR = 0;
        MPU6500_I2C_delay();
        MPU6500_I2C_SCL = 1;
        MPU6500_I2C_delay();
        MPU6500_I2C_SCL = 0;
        dat <<= 1;
    }
}

u8 MPU6500_I2C_read_char(void)
{
    u8 i = 0, dat = 0;
    MPU6500_I2C_SCL = 0;
    MPU6500_I2C_SetIn_Mode();
    for (i = 0; i < 8; i++) {

        dat <<= 1;
        MPU6500_I2C_SCL = 1;
        MPU6500_I2C_delay();
        if (MPU6500_I2C_SDA_RE) dat |= 0x01;
        MPU6500_I2C_SCL = 0;
        MPU6500_I2C_delay();
    }
    return dat;
}

u8 MPU6500_write_byte(u8 reg, u8 data)
{
    MPU6500_I2C_start();
    MPU6500_I2C_write_char(MPU6500_device_addr << 1 | 0x00);
    if (MPU6500_I2C_check_ack()) {
        MPU6500_I2C_stop();
        return 2;
    }

    MPU6500_I2C_write_char(reg);
    MPU6500_I2C_check_ack();
    MPU6500_I2C_write_char(data);
    MPU6500_I2C_check_ack();
    MPU6500_I2C_stop();
    return 0;
}

u8 MPU6500_read_byte(u8 reg)
{
    u8 data = 0;

    MPU6500_I2C_start();
    MPU6500_I2C_write_char(MPU6500_device_addr << 1 | 0x00);
    if (MPU6500_I2C_check_ack()) {
        MPU6500_I2C_stop();
        return 2;
    }
    MPU6500_I2C_write_char(reg);
    MPU6500_I2C_check_ack();

    MPU6500_I2C_start();
    MPU6500_I2C_write_char(MPU6500_device_addr << 1 | 0x01);
    MPU6500_I2C_check_ack();
    data = MPU6500_I2C_read_char();
    MPU6500_I2C_ack();
    MPU6500_I2C_stop();
    return data;
}

u8 MPU6500_Read_Len(u8 DeviceAddr, u8 RegAddr, u8 len, u8 *pbuff)
{
    u8 i;
    MPU6500_I2C_start();
    MPU6500_I2C_write_char(DeviceAddr << 1 | 0x00);
    if (MPU6500_I2C_check_ack()) {
        MPU6500_I2C_stop();
        return 2;
    }

    MPU6500_I2C_write_char(RegAddr);
    MPU6500_I2C_check_ack();

    MPU6500_I2C_start();
    MPU6500_I2C_write_char(DeviceAddr << 1 | 0x01);
    MPU6500_I2C_check_ack();
    for (i = 0; i < len; i++) {
        *pbuff++ = MPU6500_I2C_read_char();
        if (i + 1 >= len) {
            MPU6500_I2C_NoAck();
            break;
        }
        MPU6500_I2C_ack();
    }

    MPU6500_I2C_stop();

    return 0;
}

u8 MPU6500_Write_Len(u8 DeviceAddr, u8 RegAddr, u8 len, u8 *pbuff)
{
    u8 i;

    MPU6500_I2C_start();
    MPU6500_I2C_write_char(DeviceAddr << 1 | 0x00);
    if (MPU6500_I2C_check_ack()) {
        MPU6500_I2C_stop();
        return 2;
    }

    MPU6500_I2C_write_char(RegAddr);
    MPU6500_I2C_check_ack();
    for (i = 0; i < len; i++) {
        MPU6500_I2C_write_char(*pbuff++);
        if (MPU6500_I2C_check_ack()) {
            MPU6500_I2C_stop();
            return 3;
        }
    }

    MPU6500_I2C_stop();
    return 0;
}

u8 InitMPU6050(void)
{
    int i = 0, j = 0;
    u8 res = 0;
    MPU6500_I2C_PORT_Init();

    // 在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    for (i = 0; i < 1000; i++) {
        for (j = 0; j < 1000; j++) {
            ;
        }
    }

    MPU6500_write_byte(PWR_MGMT_1, 0x80); // 复位MPU6050
    Delay_ms(100);
    MPU6500_write_byte(PWR_MGMT_1, 0);       // 唤醒MPU6050
    MPU6500_Set_Gyro_Fsr(3);                 // 陀螺仪传感器,±2000dps
    MPU6500_Set_Accel_Fsr(0);                // 加速度传感器,±2g
    MPU6500_Set_Rate(50);                    // 设置采样率50Hz
    MPU6500_write_byte(INT_EN_REG, 0X00);    // 关闭所有中断
    MPU6500_write_byte(USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU6500_write_byte(FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU6500_write_byte(INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    res = MPU6500_read_byte(WHO_AM_I);
    if (res == MPU6500_ID) {
        MPU6500_write_byte(PWR_MGMT_1, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU6500_write_byte(PWR_MGMT_2, 0X00); // 加速度与陀螺仪都工作
        MPU6500_Set_Rate(50);                 // 设置采样率为50Hz

    } else
        return 1;

    /*
    temp=0x080;
    MPU6500_Write_Len(MPU6500_device_addr,PWR_MGMT_1,1, &temp);	//解除休眠状态
    temp=0x00;
    MPU6500_Write_Len(MPU6500_device_addr,PWR_MGMT_1,1, &temp);	//解除休眠状态

    MPU6500_Write_Len(MPU6500_device_addr,SMPLRT_DIV,1, &temp);
    temp=0x06;
    MPU6500_Write_Len(MPU6500_device_addr,CONFIG, 1,&temp);
    temp=0x00;
    MPU6500_Write_Len(MPU6500_device_addr,ACCEL_CONFIG,1, &temp);
    temp=0x18;
    MPU6500_Write_Len(MPU6500_device_addr,GYRO_CONFIG,1, &temp);
    */

    return 0;
}

short GetData(u8 REG_Address)
{
    u8 buff[2];
    MPU6500_Read_Len(MPU6500_device_addr, REG_Address, 2, buff);
    return ((buff[0] << 8) | buff[1]); // 合成数据
}

// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
u8 MPU6500_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 result;
    u8 acecel[6];
    result = MPU6500_Read_Len(MPU6500_device_addr, ACCEL_XOUT_H, 6, acecel);
    if (result == 0) {
        *ax = ((u16)acecel[0] << 8) | acecel[1];
        *ay = ((u16)acecel[2] << 8) | acecel[3];
        *az = ((u16)acecel[4] << 8) | acecel[5];
    }

    return result;
}

// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
u8 MPU6500_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 result;
    u8 gyro[6];
    result = MPU6500_Read_Len(MPU6500_device_addr, GYRO_XOUT_H, 6, gyro);
    if (result == 0) {
        *gx = ((u16)gyro[0] << 8) | gyro[1];
        *gy = ((u16)gyro[2] << 8) | gyro[3];
        *gz = ((u16)gyro[4] << 8) | gyro[5];
    }

    return result;
}

float MPU6500_GetTemperature(void)
{
    short temp;
    float dat;
    temp = GetData(TEMP_OUT_H);

    dat = ((double)(temp - 21) / 333.87) + 21;

    return dat;
}

// 设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU6500_Set_Gyro_Fsr(u8 fsr)
{
    return MPU6500_write_byte(GYRO_CONFIG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU6500_Set_Accel_Fsr(u8 fsr)
{
    return MPU6500_write_byte(ACCEL_CONFIG, fsr << 3); // 设置加速度传感器满量程范围
}

// 设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU6500_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000) rate = 1000;
    if (rate < 4) rate = 4;
    data = 1000 / rate - 1;
    data = MPU6500_write_byte(SMPLRT_DIV, data); // 设置数字低通滤波器
    return MPU6500_Set_LPF(rate / 2);            // 自动设置LPF为采样率的一半
}

// 设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU6500_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU6500_write_byte(CONFIG, data); // 设置数字低通滤波器
}

// MPU6050自测试
// 返回值:0,正常
//     其他,失败
u8 MPU6500_run_self_test(void)
{
    int result;
    // char test_packet[4] = {0};
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x07) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float gyro_sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long)(gyro[0] * gyro_sens);
        gyro[1] = (long)(gyro[1] * gyro_sens);
        gyro[2] = (long)(gyro[2] * gyro_sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);

        return 0;
    } else
        return 1;
}
u8 MPU6500_DMP_Init(void)
{
    struct int_param_s int_param;
    int result;

    result = mpu_init(&int_param);
    if (result) return 1;

    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); // 设置所需要的传感器
    if (result) return 2;

    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); // 设置FIFO
    if (result) return 3;

    result = mpu_set_sample_rate(DEFAULT_MPU_HZ); // 设置采样率
    if (result) return 4;

    result = dmp_load_motion_driver_firmware(); // 加载dmp固件
    if (result) return 5;

    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation)); // 设置陀螺仪方向
    if (result) return 6;

    result = dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
                                DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
                                DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO); // 设置dmp功能
    if (result) return 7;

    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ); // 设置DMP输出速率(最大不超过200Hz)
    if (result) return 8;

    result = MPU6500_run_self_test(); // 自检
    if (result) return 9;

    result = mpu_set_dmp_state(1); // 使能DMP
    if (result) return 10;

    result = dmp_set_interrupt_mode(DMP_INT_CONTINUOUS); // 设置中断产生方式
    if (result) return 11;

    return 0;
}
// 得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
// pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
// roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
// yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
// 返回值:0,正常
//     其他,失败
u8 MPU6500_dmp_get_euler_angle(short *accel, short *gyro, float *pitch, float *roll, float *yaw)
{

// q30格式,long转float时的除数.
#define Q30 ((1 << 30) * 1.0f)

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short sensors;
    unsigned char more;
    long quat[4];
    u8 result = 0;
    result    = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if (result) return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
     **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
     **/
    if (sensors & INV_WXYZ_QUAT) {
        q0 = quat[0] / Q30; // q30格式转换为浮点数
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;
        // 计算得到俯仰角/横滚角/航向角
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                      // pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;      // roll
        *yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    } else
        return 2;
    return 0;
}