#ifndef __MPU6500_DRIVER_H
#define __MPU6500_DRIVER_H
 
#include "stm32f10x.h"
 
#define MPU6500_I2C_SDA_RE PBin(13)	
#define MPU6500_I2C_SDA_WR PBout(13)
 
#define MPU6500_I2C_SCL PBout(12)
 
#define  MPU6500_I2C_SetOut_Mode()  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=0X00300000;}
#define  MPU6500_I2C_SetIn_Mode() 	{GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=0X00800000;GPIOB->ODR|=0X01<13;}	
 
#define MPU6500_device_addr     0x68
#define MPU6500_ID     0x70
 
 
#define DEFAULT_MPU_HZ 100		//100Hz
 
//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG				0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
 
#define FIFO_EN_REG			0X23	//FIFO使能寄存器
#define INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define INT_EN_REG			0X38	//中断使能寄存器
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define USER_CTRL_REG	0X6A	//用户控制寄存器
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define PWR_MGMT_2   	0X6C	//电源管理寄存器2 
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)
 
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取
 
 
 
extern u8  MPU_EXTI_flag;
 
 
 
 
void MPU6500_Port_EXIT_Init(void);
void MPU6500_I2C_PORT_Init(void);
void MPU6500_I2C_delay(void);
void MPU6500_I2C_start(void);
void MPU6500_I2C_stop(void);
u8 MPU6500_I2C_check_ack(void);
void MPU6500_I2C_ack(void);
void MPU6500_I2C_NoAck(void);
void MPU6500_I2C_write_char(u8 dat);
u8 MPU6500_I2C_read_char(void);
u8 MPU6500_write_byte(u8 reg,u8 data);
u8 MPU6500_read_byte(u8 reg);
u8 MPU6500_Read_Len(u8 DeviceAddr,u8 RegAddr,u8 len,u8 *pbuff);
u8 MPU6500_Write_Len(u8 DeviceAddr,u8 RegAddr,u8 len,u8 *pbuff);
u8 InitMPU6050(void);
short GetData(u8 REG_Address);
u8 MPU6500_Get_Accelerometer(short *ax,short *ay,short *az);
u8 MPU6500_Get_Gyroscope(short *gx,short *gy,short *gz);
float MPU6500_GetTemperature(void);
u8 MPU6500_Set_Gyro_Fsr(u8 fsr);
u8 MPU6500_Set_Accel_Fsr(u8 fsr);
u8 MPU6500_Set_Rate(u16 rate);
u8 MPU6500_Set_LPF(u16 lpf);
u8 MPU6500_run_self_test(void);
u8 MPU6500_DMP_Init(void);
u8 MPU6500_dmp_get_euler_angle(short *accel,short *gyro,float *pitch,float *roll,float *yaw);
 
 
 
#endif