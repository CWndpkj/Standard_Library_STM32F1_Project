#ifndef _BUZZER_H_
#define _BUZZER_H_
/**
 * @brief 端口初始化函数
 * 
 */
void Buzzer_Init(void);

/**
 * @brief 开启蜂鸣器
 * 
 */
void Buzzer_On(void);

/**
 * @brief 关闭蜂鸣器
 * 
 */
void Buzzer_Off(void);

/**
 * @brief 反转蜂鸣器状态
 * 
 */
void Buzzer_Turn(void);

#endif // !_BUZZER_H_
