/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ADS7830_ADC_H
#define ADS7830_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define ADS7830_DEVICE_NAME "ADS7830"

/* Initialization arguments structure to track initialization status */
typedef struct {
    bool is_init;
    // Additional initialization parameters can be added here if needed
} ads7830_init_args;

/**
 * @brief Initialize the ADS7830 ADC device.
 *
 * @param cfg Pointer to the sensor configuration structure.
 * @return uint8_t Returns SENSOR_INIT_SUCCESS on success, or SENSOR_INIT_UNSPECIFIED_ERROR on failure.
 */
uint8_t ads7830_adc_init(sensor_cfg *cfg);

/**
 * @brief Read sensor data from ADS7830 ADC.
 *
 * @param cfg Pointer to the sensor configuration structure.
 * @param reading Pointer to store the read sensor value.
 * @return uint8_t Returns SENSOR_READ_SUCCESS on success, SENSOR_FAIL_TO_ACCESS if unable to read sensor,
 *                  or SENSOR_UNSPECIFIED_ERROR for invalid parameters.
 */
uint8_t ads7830_adc_read(sensor_cfg *cfg, int *reading);

#endif /* ADS7830_ADC_H */
