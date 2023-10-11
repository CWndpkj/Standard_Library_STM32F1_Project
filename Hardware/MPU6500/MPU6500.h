#ifndef _MPU6500_H_
#define _MPU6500_H_
typedef struct MPU6500_VALUE {
    short ACCEL_XOUT;
    short ACCEL_YOUT;
    short ACCEL_ZOUT;
    short GYRO_XOUT;
    short GYRO_YOUT;
    short GYRO_ZOUT;
    double TEMP_OUT;
} MPU6500_Value;
void MPU6500_Init();
u8 MPU6500_Set_LPF(u16 lpf);
u8 MPU6500_run_self_test(void);
u8 MPU6500_Set_Rate(u16 rate);
u8 MPU6500_StartAquire();
void MPU6500_GetValue(MPU6500_Value *value);
u8 MPU6500_DMP_Init(void);
u8 MPU6500_dmp_get_euler_angle(short *accel, short *gyro, float *pitch, float *roll, float *yaw);
float MPU6500_GetTemperature(void);
u8 MPU6500_GetAddr();
u8 MPU6500_Set_Gyro_Fsr(u8 fsr);
u8 MPU6500_Set_Accel_Fsr(u8 fsr);
u8 MPU6500_Set_Rate(u16 rate);
u8 MPU6500_Set_LPF(u16 lpf);
u8 MPU6500_run_self_test(void);
u8 MPU6500_Send2Host(u16 ACCEL_XOUT, u16 ACCEL_YOUT, u16 ACCEL_ZOUT, u16 GYRO_XOUT, u16 GYRO_YOUT, u16 GYRO_ZOUT);
#endif // !_MPU6500_H_
