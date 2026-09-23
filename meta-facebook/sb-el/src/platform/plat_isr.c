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
#include "plat_hook.h"
#include "shell_arke_power.h"
#include "plat_kernel_obj.h"
#include "plat_log.h"
#include "plat_event.h"
#include "plat_hwmon.h"
#include "plat_ioexp.h"
#include "plat_arke_smbus.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include "plat_class.h"
#include "plat_power_capping.h"
#include "plat_user_setting.h"
#include "plat_vr_test_mode.h"
#include "plat_pldm_sensor.h"
#include "plat_clock.h"
#include "plat_adc.h"
#include "plat_mctp.h"

LOG_MODULE_REGISTER(plat_isr);

uint8_t pwr_steps_on_flag = 0;

static bool sensor_polling_delay_elapsed;

static void sensor_polling_delay_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	sensor_polling_delay_elapsed = is_mb_dc_on();
}

K_WORK_DELAYABLE_DEFINE(sensor_polling_delay_work, sensor_polling_delay_work_handler);

bool get_sensor_polling_delay_elapsed(void)
{
	return sensor_polling_delay_elapsed;
}

void set_sensor_polling_delay_elapsed(bool elapsed)
{
	sensor_polling_delay_elapsed = elapsed;
}

void set_pwr_steps_on_flag(uint8_t flag_value)
{
	pwr_steps_on_flag = flag_value;
	//check value
	if (pwr_steps_on_flag != flag_value)
		LOG_ERR("set pwr_steps_on_flag failed, now pwr_steps_on_flag = %d",
			pwr_steps_on_flag);
}

uint8_t get_pwr_steps_on_flag(void)
{
	return pwr_steps_on_flag;
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	if (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW) {
		plat_trigger_cpld_polling();
	}
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_DBG("FM_PLD_UBC_EN_R GPIO ISR, value: %d", gpio_get(FM_PLD_UBC_EN_R));
}

bool ubc_en_changed_callback(cpld_info *info, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(info, false);
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (get_pwr_steps_on_flag())
		return false;

	bool ubc_en = !!(*data & info->bit_check_mask);
	bool last_ubc_en = !!(info->last_polling_value & info->bit_check_mask);

	if (ubc_en == last_ubc_en)
		return false;

	LOG_INF("UBC_EN changed: %d -> %d", last_ubc_en, ubc_en);

	if (ubc_en) {
		plat_set_dc_on_log(LOG_ASSERT);
		plat_handle_pwr_sequence_event();
	} else {
		plat_set_dc_on_log(LOG_DEASSERT);
	}

	plat_update_ubc_status();

	return true;
}

/* Pin A12 (GPIO73 / SPIP1_CS) dynamic mux switching
 *
 * Hardware mux: SCFG DEVALTC register, SPIP1_SL bit (bit 1):
 *   DEVALTC[1] = 0 → GPIO66/67/70/73/76/77 on pads   (A12 = GPIO73)
 *   DEVALTC[1] = 1 → SPIP1 signals on pads           (A12 = SPIP1_CS)
*/
#define NPCM4XX_SCFG_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg)
#define NPCM4XX_DEVALTC 0x1C
#define NPCM4XX_SPIP1_SEL 1 /* DEVALTC bit1: 0=GPIO73, 1=SPIP1_CS    */

#define NPCM4XX_GPIO7_BASE (REG_GPIO_BASE + 0xE000) /* GPIO_7 register base  */
#define NPCM4XX_GPIO7_PDOUT 0x00 /* Port data output offset               */
#define NPCM4XX_GPIO7_PDIR 0x02 /* Port direction offset                 */
#define NPCM4XX_GPIO73_BIT 3 /* GPIO73 = port-7 bit-3                 */

void plat_switch_pin_a12(bool use_gpio73)
{
	uint8_t devaltc = sys_read8(NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);

	if (use_gpio73) {
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDOUT) &
				   ~BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDOUT);
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR) |
				   BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR);
		sys_write8(devaltc & ~BIT(NPCM4XX_SPIP1_SEL), NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);
		LOG_INF("[PIN_A12] A12 -> GPIO73 (output LOW)");
	} else {
		sys_write8(devaltc | BIT(NPCM4XX_SPIP1_SEL), NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR) &
				   ~BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR);
		LOG_INF("[PIN_A12] A12 -> SPIP1_CS");
	}
}

void ISR_GPIO_RST_ARKE_PWR_ON_PLD_R1_N()
{
	// dc on
	if (gpio_get(RST_ARKE_PWR_ON_PLD_R1_N)) {
		plat_switch_pin_a12(false); /* HIGH -> A12 = SPIP1_CS */
		ioexp_init();
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			// Ensure U200053 is initialized before initializing U200051.
			init_U200052_IO();
			init_U200053_IO();
			init_U200070_IO();
			init_U200051_IO();
		}
		for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
			clear_clock_status(NULL, i);
		}
		add_sync_oc_warn_to_work();
		// if board id == EVB , ctrl fan pwm
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			LOG_INF("dc on, set fan pwm 100");
			init_pwm_dev();
			ast_pwm_set(100, PWM_PORT2);
			ast_pwm_set(100, PWM_PORT6);
		}
		// when dc on clear cpld polling alert status
		uint8_t err_type = CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE;
		reset_error_log_states(err_type);
		// re-init adc
		set_is_adc_init(0);

		/* Refresh offsets and restore permanent VOUT settings on every DC on. */
		vr_vout_offset_get_init();
		if (!set_all_vout_command())
			LOG_ERR("set all vout command fail!");

		//check RNS vr CML status
		check_rns_vr_cml_status();
		// Allow VR/UBC polling one second after DC power becomes stable.
		set_sensor_polling_delay_elapsed(false);
		k_work_reschedule(&sensor_polling_delay_work, K_SECONDS(1));
	} else {
		k_work_cancel_delayable(&sensor_polling_delay_work);
		set_sensor_polling_delay_elapsed(false);
		plat_switch_pin_a12(true); /* LOW -> A12 = GPIO73 output low */
		gpio_conf(SPI_ADC_CS1_N, GPIO_OUTPUT);
		gpio_set(SPI_ADC_CS1_N, GPIO_LOW);
		if (get_vr_test_mode_flag()) {
			LOG_INF("dc off, exit the vr test mode");
			vr_test_mode_enable(false);
		}
		// if board id == EVB , ctrl fan pwm
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			LOG_INF("dc off, set fan pwm 0");
			init_pwm_dev();
			ast_pwm_set(0, PWM_PORT2);
			ast_pwm_set(0, PWM_PORT6);
		}
		// set I3C_ELECTRA_ALERT_R_N to default
		gpio_set(I3C_ELECTRA_ALERT_R_N, GPIO_HIGH);
	}
}

void ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N()
{
	uint8_t data[FATAL_ERROR_LEN] = { 0 };
	LOG_INF("smb hamsa mmc lvc33 alert triggered");

	if (!plat_i2c_read(I2C_BUS12, HAMSA_BOOT1_ADDR, SMBUS_ERROR, data, FATAL_ERROR_LEN)) {
		LOG_ERR("Read ASIC offset 0x%x fail", SMBUS_ERROR);
		return;
	}

	LOG_HEXDUMP_DBG(data, FATAL_ERROR_LEN, "smb hamsa mmc lvc33 alert data");

	sb_cmd_fatal_error rec = { 0 };
	memcpy(&rec, data, sizeof(rec));

	if (rec.length != ERROR_CODE_LEN) {
		LOG_ERR("Invalid event record length: %d", rec.length);
		return;
	}

	if (!plat_i2c_read(I2C_BUS12, HAMSA_BOOT1_ADDR, SMBUS_ASIC_ID, data,
			   sizeof(struct smb_cmd_id))) {
		LOG_ERR("Read ASIC offset 0x%x fail", SMBUS_ASIC_ID);
		return;
	}

	struct smb_cmd_id smb_cmd_id = { 0 };
	memcpy(&smb_cmd_id, data, sizeof(smb_cmd_id));

	plat_asic_error_event asic_event = { 0 };
	asic_event.event_id_0 = rec.event_record_data.common.event_id & 0xFF;
	asic_event.event_id_1 = (rec.event_record_data.common.event_id >> 8) & 0xFF;
	asic_event.chip_id = rec.event_record_data.common.chiplet_id;
	asic_event.module_id = rec.event_record_data.common.module_id;
	plat_asic_error_error_log(LOG_ASSERT, asic_event);

	struct pldm_addsel_data smb_hamsa_sel_msg = { 0 };
	smb_hamsa_sel_msg.assert_type = LOG_ASSERT;
	smb_hamsa_sel_msg.event_type = ARKE_FAULT;
	smb_hamsa_sel_msg.event_data_1 = HAMSA_SMB_ERR_EVENT_HEADER;
	smb_hamsa_sel_msg.event_data_2 = asic_event.event_id_0;
	smb_hamsa_sel_msg.event_data_3 = asic_event.event_id_1;
	if (send_event_log_to_bmc(smb_hamsa_sel_msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send hamsa smb error code to bmc, event data: 0x%x 0x%x 0x%x\n",
			smb_hamsa_sel_msg.event_data_1, smb_hamsa_sel_msg.event_data_2,
			smb_hamsa_sel_msg.event_data_3);
	}

	uint8_t eid = 0x08;
	uint8_t resp_buf[PLDM_MAX_DATA_SIZE] = { 0 };
	pldm_msg pmsg = { 0 };
	mctp *mctp_inst = NULL;

	pmsg.hdr.msg_type = MCTP_MSG_TYPE_PLDM;
	pmsg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	pmsg.hdr.cmd = PLDM_MONITOR_CMD_CODE_PLATFORM_EVENT_MESSAGE;
	pmsg.hdr.rq = PLDM_REQUEST;

	uint8_t event_buf[sizeof(struct pldm_platform_event_msg) +
			  sizeof(struct pldm_cper_event_data) +
			  sizeof(struct mtia_oem_cper_event)] = { 0 };

	struct pldm_platform_event_msg *evt = (struct pldm_platform_event_msg *)event_buf;
	evt->format_version = 0x01;
	evt->tid = 0x01;
	evt->event_class = PLDM_CPER_EVENT;

	struct pldm_cper_event_data *cper_evt = (struct pldm_cper_event_data *)evt->event_data;
	cper_evt->cper_format_version = CPER_FORMAT_VERSION;
	cper_evt->cper_format_type = FULL_CPER_SECTION;
	cper_evt->cper_data_length = sizeof(struct mtia_oem_cper_event);

	// clang-format off
	const uint8_t guid_mmc[16] = { 0x6c, 0x7f, 0x57, 0x2f, 0x8a, 0xdd, 0x85, 0x48,
				       0x99, 0xfd, 0x0b, 0x66, 0xe8, 0xac, 0xa0, 0x3f };
	const uint8_t guid_mtia_header[16] = { 0x7c, 0x09, 0xc0, 0xbe, 0x45, 0x55, 0x24, 0x48,
					       0x90, 0x1a, 0xd9, 0x6c, 0x8c, 0x9e, 0xcc, 0x2d };
	const uint8_t guid_mtia_section[16] = { 0xdf, 0x7d, 0xf6, 0xc8, 0x84, 0xe7, 0x1d, 0x47,
					        0xa0, 0x5f, 0x7c, 0x70, 0xa7, 0xa1, 0x1a, 0xd6 };
	// clang-format on

	struct event_record_common *asic_event_data = &rec.event_record_data.common;
	struct mtia_oem_cper_event *cper_record =
		(struct mtia_oem_cper_event *)cper_evt->cper_record;

	cper_record->record_header.signatureStart = 0x52455043;
	cper_record->record_header.Revision = 0x0101;
	cper_record->record_header.SignatureEnd = 0xFFFFFFFF;
	cper_record->record_header.SectionCount = 1;
	cper_record->record_header.ErrorSeverity = asic_event_data->severity;
	cper_record->record_header.ValidationBits = 0x02;
	cper_record->record_header.RecordLength = sizeof(struct mtia_oem_cper_event);
	cper_record->record_header.Timestamp = asic_event_data->timestamp;
	memcpy(cper_record->record_header.CreatorID, guid_mmc, sizeof(guid_mmc));
	memcpy(cper_record->record_header.NotificationType, guid_mtia_header,
	       sizeof(guid_mtia_header));

	cper_record->section_descriptor.sectionOffset =
		sizeof(struct cper_record_header) + sizeof(struct cper_section_descriptor);
	cper_record->section_descriptor.sectionLength =
		sizeof(struct mtia_oem_cper_section_header) + sizeof(event_record);
	cper_record->section_descriptor.revision = 0x0100;
	memcpy(cper_record->section_descriptor.sectionType, guid_mtia_section,
	       sizeof(guid_mtia_section));
	cper_record->section_descriptor.sectionSeverity = asic_event_data->severity;

	cper_record->section_header.version = 0x0100;
	cper_record->section_header.record_size =
		sizeof(struct mtia_oem_cper_section_header) + sizeof(event_record);
	cper_record->section_header.device_id.vendor_id = smb_cmd_id.pcie_vendor_id;
	memcpy(cper_record->section_header.device_serial_number, smb_cmd_id.asic_serial_number,
	       sizeof(smb_cmd_id.asic_serial_number));
	memcpy(&cper_record->section_data, &rec.event_record_data, sizeof(rec.event_record_data));

	pmsg.len = sizeof(event_buf);
	pmsg.buf = event_buf;
	LOG_HEXDUMP_DBG(pmsg.buf, pmsg.len, "pmsg");

	if (!get_mctp_info_by_eid(eid, &mctp_inst, &pmsg.ext_params)) {
		LOG_ERR("Failed to get mctp info by eid 0x%x", eid);
		return;
	}

	if (!mctp_pldm_read(mctp_inst, &pmsg, resp_buf, sizeof(resp_buf))) {
		LOG_ERR("Failed to send Hamsa CPER event to BMC");
	}
}

void ISR_ASIC_THERMTRIP_TRIGGER(void)
{
	plat_asic_thermtrip_error_log(LOG_ASSERT);
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	switch (gpio_num) {
	case ALL_VR_PM_ALERT_R_N:
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}
