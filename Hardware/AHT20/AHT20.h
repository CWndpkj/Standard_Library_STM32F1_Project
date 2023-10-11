#ifndef _AHT20_H_
#define _AHT20_H_
typedef struct AHT20_Value {
    float Humidity;
    float Temperature;
} AHT20_value;
u8 AHT20_Init();
u8 AHT20_StartAquire();
u8 ATH20_Read_Status();
u8 AHT20_GetValue(AHT20_value *value);
#endif // !_AH