#ifndef _ADS7830_H_
#define _ADS7830_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define ADC_CH0 0x84
#define ADC_CH1 0xC4
#define ADC_CH2 0x94
#define ADC_CH3 0xD4
#define ADC_CH4 0xA4
#define ADC_CH5 0xE4
#define ADC_CH6 0xB4
#define ADC_CH7 0xF4

uint8_t ads7830_init(sensor_cfg *cfg);
uint8_t ads7830_read(sensor_cfg *cfg, int *reading);

#endif /* _ADS7830_H_ */
