#ifndef _LIGHTSENSOR_H_
#define _LIGHTSENSOR_H_

/**
 * @brief 端口处初始化
 * 
 */
void LightSensor_Init(void);
/**
 * @brief 获取传感器的数字信号，高于阈值返回1，否则返回低电平
 * 
 * @return uint8_t 
 */
uint8_t LightSensor_GetStatus(void);
#endif // !_LIGHTSENSOR_H_
