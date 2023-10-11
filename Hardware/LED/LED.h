#ifndef _LED_H_
#define _LED_H_
/**
 * @note LED设置为低电平驱动
 */

/**
 * @brief 端口初始化函数
 * 
 */
void LED_Init(void);

/**
 * @brief 开启LED
 * 
 */
void LED_On(void);

/**
 * @brief 关闭LED
 * 
 */
void LED_Off(void);

/**
 * @brief 反转LED状态
 * 
 */
void LED_Turn(void);

#endif // !_LED_H_
