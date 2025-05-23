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

#include <stdio.h>
#include "sensor.h"
#include "plat_gpio.h"
#include "plat_def.h"
#include <zephyr.h>
#include <drivers/adc.h>
#include <logging/log.h>
#include "ads7830.h"

LOG_MODULE_REGISTER(dev_ads7830_adc);

#define ADS7830_RESOLUTION 8
#define ADS7830_CHANNEL_COUNT 8
#define ADS7830_ADC_GAIN ADC_GAIN_1
#define ADS7830_ADC_REFERENCE ADC_REF_INTERNAL
#define ADS7830_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADS7830_AVERAGE_DELAY_MSEC 1

static const struct device *dev_adc;
static int16_t sample_buffer[1];
static int is_ready = 0;

/* Initialize the ADS7830 ADC device */
static void init_ads7830_adc_dev()
{
    dev_adc = device_get_binding("ADC0");
    if (!(device_is_ready(dev_adc))) {
        LOG_WRN("ADS7830 device not ready!");
    } else {
        is_ready = 1;
    }
}

/* Read ADC value in millivolts from ADS7830 */
static bool ads7830_adc_read_mv(sensor_cfg *cfg, uint8_t sensor_num, uint32_t channel, int *adc_val)
{
    CHECK_NULL_ARG_WITH_RETURN(adc_val, false);

    if (sensor_num > SENSOR_NUM_MAX) {
        LOG_DBG("Invalid sensor number");
        return false;
    }

    if (!is_ready) {
        LOG_ERR("ADS7830 is not ready to read!");
        return false;
    }

    int retval;
    static struct adc_sequence sequence;
    sequence.channels = BIT(channel);
    sequence.buffer = sample_buffer;
    sequence.buffer_size = sizeof(sample_buffer);
    sequence.resolution = ADS7830_RESOLUTION;
    sequence.calibrate = 0;

    static struct adc_channel_cfg channel_cfg;
    channel_cfg.gain = ADS7830_ADC_GAIN;
    channel_cfg.reference = ADS7830_ADC_REFERENCE;
    channel_cfg.acquisition_time = ADS7830_ACQUISITION_TIME;
    channel_cfg.channel_id = channel;
    channel_cfg.differential = 0;

    retval = adc_channel_setup(dev_adc, &channel_cfg);
    if (retval) {
        LOG_ERR("ADS7830 channel setup failed for sensor[0x%x]", sensor_num);
        return false;
    }

    retval = adc_read(dev_adc, &sequence);
    if (retval != 0) {
        LOG_ERR("ADS7830 reading failed for sensor[0x%x] with error %d", sensor_num, retval);
        return false;
    }

    int32_t raw_value = sample_buffer[0];
    int32_t ref_mv = adc_get_ref(dev_adc);
    if (ref_mv <= 0) {
        LOG_ERR("ADS7830 reference voltage get failed for sensor[0x%x]", sensor_num);
        return false;
    }

    *adc_val = raw_value;
    adc_raw_to_millivolts(ref_mv, channel_cfg.gain, sequence.resolution, adc_val);

    return true;
}

/* Read sensor data from ADS7830 ADC, with averaging over sample_count */
uint8_t ads7830_adc_read(sensor_cfg *cfg, int *reading)
{
    CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
    CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

    if (cfg->num > SENSOR_NUM_MAX) {
        LOG_DBG("Invalid sensor number");
        return SENSOR_UNSPECIFIED_ERROR;
    }

    uint8_t number = cfg->port % ADS7830_CHANNEL_COUNT;
    int val = 1, i = 0, average_val = 0;

    for (i = 0; i < cfg->sample_count; i++) {
        val = 1;
        if (!ads7830_adc_read_mv(cfg, cfg->num, number, &val))
            return SENSOR_FAIL_TO_ACCESS;
        average_val += val;

        /* Prevent busy looping by delaying between samples */
        if (cfg->sample_count > SAMPLE_COUNT_DEFAULT) {
            k_msleep(ADS7830_AVERAGE_DELAY_MSEC);
        }
    }

    if (cfg->arg1 == 0) {
        LOG_ERR("Sensor number: 0x%x argument arg1 is zero", cfg->num);
        return SENSOR_PARAMETER_NOT_VALID;
    }

    average_val = average_val / cfg->sample_count;
    average_val = average_val * cfg->arg0 / cfg->arg1;

    sensor_val *sval = (sensor_val *)reading;
    sval->integer = (average_val / 1000) & 0xFFFF;
    sval->fraction = (average_val % 1000) & 0xFFFF;

    return SENSOR_READ_SUCCESS;
}

/* Initialize ADS7830 ADC sensor driver */
uint8_t ads7830_adc_init(sensor_cfg *cfg)
{
    CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
    CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

    if (cfg->num > SENSOR_NUM_MAX) {
        LOG_DBG("Invalid sensor number");
        return SENSOR_INIT_UNSPECIFIED_ERROR;
    }

    ads7830_init_args *init_args = (ads7830_init_args *)cfg->init_args;
    if (init_args->is_init)
        goto skip_init;

    init_ads7830_adc_dev();

    if (!is_ready) {
        LOG_ERR("ADS7830 is not ready to use!");
        return SENSOR_INIT_UNSPECIFIED_ERROR;
    }

    static struct adc_channel_cfg channel_cfg;
    channel_cfg.gain = ADS7830_ADC_GAIN;
    channel_cfg.reference = ADS7830_ADC_REFERENCE;
    channel_cfg.acquisition_time = ADS7830_ACQUISITION_TIME;
    channel_cfg.channel_id = 0;
    channel_cfg.differential = 0;

    static struct adc_sequence sequence;
    sequence.channels = 0;
    sequence.buffer = sample_buffer;
    sequence.buffer_size = sizeof(sample_buffer);
    sequence.resolution = ADS7830_RESOLUTION;
    sequence.calibrate = 0;

    for (uint8_t i = 0; i < ADS7830_CHANNEL_COUNT; i++) {
        channel_cfg.channel_id = i;
        adc_channel_setup(dev_adc, &channel_cfg);
        sequence.channels |= BIT(i);
    }

    init_args->is_init = true;

skip_init:
    cfg->read = ads7830_adc_read;
    return SENSOR_INIT_SUCCESS;
}
