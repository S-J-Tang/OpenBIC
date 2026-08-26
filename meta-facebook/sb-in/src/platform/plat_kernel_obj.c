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

#include "plat_kernel_obj.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_kernel_obj);

/* semaphore CPLD polling semaphore */
K_TIMER_DEFINE(ragular_cpld_polling_sem_timer, plat_ragular_cpld_polling_sem_handler, NULL);
static struct k_sem cpld_polling_sem;

void plat_ragular_cpld_polling_sem_handler(struct k_timer *timer)
{
	k_sem_give(&cpld_polling_sem);
}

void plat_activate_cpld_polling_semaphore_timer(void)
{
	k_sem_init(&cpld_polling_sem, 0, 1);
	k_timer_start(&ragular_cpld_polling_sem_timer, K_MSEC(1000), K_MSEC(1000));
}

void plat_wait_for_cpld_polling_trigger(void)
{
	k_sem_take(&cpld_polling_sem, K_FOREVER);
}

void plat_trigger_cpld_polling(void)
{
	LOG_WRN("triggering CPLD polling");
	k_sem_give(&cpld_polling_sem);
}
