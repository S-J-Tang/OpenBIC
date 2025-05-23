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

#ifndef PLAT_DEF_H
#define PLAT_DEF_H

#define BMC_USB_PORT "CDC_ACM_0"

#define ENABLE_MCTP_I3C
#define MCTP_I3C_PEC_ENABLE 1

#define ENABLE_PLDM
#define ENABLE_PLDM_SENSOR
#define ENABLE_CCI
#define ENABLE_VISTARA
#define ENABLE_EVENT_TO_BMC

#define ENABLE_RTQ6056

#define BIC_UPDATE_MAX_OFFSET 0xC0000
#define WORKER_STACK_SIZE 2048

#endif
