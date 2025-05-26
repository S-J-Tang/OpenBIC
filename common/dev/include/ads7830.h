#ifndef _ADS7830_H_
#define _ADS7830_H_

#include <stdint.h>
#include <stdbool.h>

// ADS7830 7-bit I2C address (0x96 >> 1)
#define ADS7830_I2C_ADDR 0x4B

// Command Byte 定義（單端輸入 + PD=01）
#define ADC_CH0 0x84
#define ADC_CH1 0xC4
#define ADC_CH2 0x94
#define ADC_CH3 0xD4
#define ADC_CH4 0xA4
#define ADC_CH5 0xE4
#define ADC_CH6 0xB4
#define ADC_CH7 0xF4

// 初始化與讀取函式
uint8_t ads7830_init(sensor_cfg *cfg);
uint8_t ads7830_read(sensor_cfg *cfg, int *reading);

#endif /* _ADS7830_H_ */
