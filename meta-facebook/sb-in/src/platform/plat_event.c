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
#include <stdlib.h>
#include <string.h>
#include "plat_event.h"
#include "plat_log.h"

LOG_MODULE_REGISTER(plat_event);

/* ac log event*/
void plat_set_ac_on_log(void)
{
	uint16_t error_code = (AC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, LOG_ASSERT);
	LOG_INF("Generated AC on error code: 0x%x", error_code);
}

/* dc log event*/
void plat_set_dc_on_log(bool is_assert)
{
	uint16_t error_code = (DC_ON_TRIGGER_CAUSE << 13);
	error_log_event(error_code, (is_assert ? LOG_ASSERT : LOG_DEASSERT));

	if (is_assert == LOG_ASSERT) {
		LOG_INF("DC on error code asserted: 0x%x", error_code);
	} else if (is_assert == LOG_DEASSERT) {
		LOG_INF("DC on error code deasserted");
	}
}