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

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>

#include "plat_gpio.h"
#include "plat_cpld.h"
#include "plat_kernel_obj.h"

LOG_MODULE_REGISTER(plat_isr);

void ISR_GPIO_RST_ASTRID_PWR_ON_PLD_R1_N()
{
	if (gpio_get(RST_ASTRID_PWR_ON_PLD_R1_N)) {
		/* dc on */
		LOG_INF("DC ON");
	} else {
		/* dc off */
		LOG_INF("DC OFF");
	}
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	if (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW) {
		plat_trigger_cpld_polling();
	}
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_INF("FM_PLD_UBC_EN_R = %d\nDC ON", gpio_get(FM_PLD_UBC_EN_R));

	plat_update_ubc_status();
}