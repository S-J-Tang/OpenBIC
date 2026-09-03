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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "plat_clock.h"
#include "plat_i2c.h"
#include "plat_util.h"
#include <logging/log.h>
#include "plat_cpld.h"
#include "plat_class.h"
#include "pldm_oem.h"
#include "plat_log.h"

LOG_MODULE_REGISTER(plat_clock);

#define CLK_312_5MHZ_REINIT_EVENT_DATA_LEN 7

static uint8_t clk_312_5_reinit_event_data[CLK_312_5MHZ_REINIT_EVENT_DATA_LEN];

static uint8_t clk_100mhz_get_lock_status(uint8_t bus, uint8_t addr)
{
/* Switch to 2-byte address mode */
#define SSI_GLOBAL_CNFG_REG 0x26
#define SET_TWO_BYTE_ADDRESS 0x05
	uint8_t write_data = SET_TWO_BYTE_ADDRESS;
	if (!plat_i2c_write(bus, addr, SSI_GLOBAL_CNFG_REG, &write_data, 1)) {
		LOG_ERR("Failed to set clock 0x%02x to 2-byte address mode", addr);
		return 0xFF;
	}

/* Read APLL lock status */
#define U86_APLL_STS_REG_HSB 0x01
#define U86_APLL_STS_REG_LSB 0x3F
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = U86_APLL_STS_REG_HSB; //offset HSB
	i2c_msg.data[1] = U86_APLL_STS_REG_LSB; //offset LSB

	if (i2c_master_read_without_error_log(&i2c_msg, retry)) {
		LOG_ERR("Failed to read clock 0x%02x APLL lock status", addr);
		return 0xFF;
	}

/* Switch back to 1-byte address mode */
#define SET_ONE_BYTE_ADDRESS 0x01
	write_data = SET_ONE_BYTE_ADDRESS;
	if (!plat_i2c_write(bus, addr, SSI_GLOBAL_CNFG_REG, &write_data, 1)) {
		LOG_ERR("Failed to set clock 0x%02x to 1-byte address mode", addr);
		return 0xFF;
	}

	return i2c_msg.data[0] & 0x01; //bit0 is the APLL lock status
}

uint8_t clk_100mhz_get_lock_status_u86(void)
{
	return clk_100mhz_get_lock_status(I2C_BUS3, CLK_GEN_100M_U86_ADDR);
}

uint8_t clk_100mhz_get_lock_status_u200045(void)
{
	return clk_100mhz_get_lock_status(I2C_BUS2, CLK_U200045_I2C_ADDR);
}

void check_clk_buf_loss_status(void)
{
	uint8_t clk_buf_loss_status = 0;
	if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &clk_buf_loss_status, 1)) {
		LOG_ERR("Failed to read 100MHz clock buffer loss status from CPLD");
		return;
	}
	/*
	Bit7: BUFF0_100M_LOSB_PLD
	Bit6: BUFF1_100M_LOSB_PLD
	Bit5: BUFF2_100M_LOSB_PLD
	if bit7 bit6 bit5 is 0 means fail
	*/
	if ((clk_buf_loss_status & 0xE0) != 0xE0) {
		if ((clk_buf_loss_status & BIT(7)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF0_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(6)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF1_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(5)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF2_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
	}
}

/* function to read APLL lock status for CLK_GEN_312_5M_U618 */
uint8_t clk_312_5mhz_get_lock_status_u618(void)
{
#define U618_APLL_STS_REG_HSB 0x00
#define U618_APLL_STS_REG_LSB 0xbd
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = CLK_GEN_312_5M_U618_ADDR; // 7-bit
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = U618_APLL_STS_REG_HSB; //offset HSB
	i2c_msg.data[1] = U618_APLL_STS_REG_LSB; //offset LSB
	if (i2c_master_read_without_error_log(&i2c_msg, retry)) {
		LOG_ERR("Failed to read 312.5MHz clock(U618) APLL lock status");
		return 0xFF;
	}
	return i2c_msg.data[0] & 0x01; //bit 0 is the APLL lock status
}

#define U618_WORKAROUND_RETRY 5
#define U618_WORKAROUND_WRITE_DELAY_MS 15
#define U618_WORKAROUND_FM_P1V80_EN_BIT 4

static bool u618_reg_read(uint16_t offset, uint8_t *data, uint8_t data_len)
{
	I2C_MSG i2c_msg = { 0 };

	k_msleep(10);
	i2c_msg.bus = CLK_U618_I2C_BUS;
	i2c_msg.target_addr = CLK_GEN_312_5M_U618_ADDR;
	i2c_msg.tx_len = 2;
	i2c_msg.rx_len = data_len;
	i2c_msg.data[0] = (offset >> 8) & 0xff;
	i2c_msg.data[1] = offset & 0xff;

	if (i2c_master_read(&i2c_msg, U618_WORKAROUND_RETRY)) {
		LOG_ERR("Failed to read CLK U618 register 0x%04x", offset);
		return false;
	}

	memcpy(data, i2c_msg.data, data_len);
	return true;
}

static bool u618_reg_write(uint16_t offset, const uint8_t *data, uint8_t data_len)
{
	I2C_MSG i2c_msg = { 0 };

	k_msleep(10);
	i2c_msg.bus = CLK_U618_I2C_BUS;
	i2c_msg.target_addr = CLK_GEN_312_5M_U618_ADDR;
	i2c_msg.tx_len = data_len + 2;
	i2c_msg.data[0] = (offset >> 8) & 0xff;
	i2c_msg.data[1] = offset & 0xff;
	memcpy(&i2c_msg.data[2], data, data_len);

	if (i2c_master_write(&i2c_msg, U618_WORKAROUND_RETRY)) {
		LOG_ERR("Failed to write CLK U618 register 0x%04x", offset);
		return false;
	}

	return true;
}

bool check_312_5MHz_init_status(void)
{
	static const uint8_t expected_00a8[] = { 0x4d, 0xc8, 0x04, 0x0b };
	static const uint8_t expected_0080[] = { 0x6d };
	static const uint8_t expected_0088[] = { 0x00, 0x20 };
	static const uint8_t apll_reinit[] = { 0x00, 0x02, 0x00 };
	uint8_t read_00a8[sizeof(expected_00a8)] = { 0 };
	uint8_t read_0080[sizeof(expected_0080)] = { 0 };
	uint8_t read_0088[sizeof(expected_0088)] = { 0 };
	uint8_t fm_p1v80_en = 0;
	uint8_t rev_id = get_board_rev_id();
	bool has_error = false;

	/* U618 workaround is required from EVT2 onward; board ID is intentionally ignored. */
	if (rev_id < REV_ID_EVT2_FAB2) {
		LOG_INF("Board rev %d does not require CLK U618 workaround", rev_id);
		return true;
	}

	if (!plat_read_cpld(VR_EN_PIN_READING_5, &fm_p1v80_en, 1)) {
		LOG_ERR("Failed to read FM_P1V80_EN before checking CLK U618 init status");
		return false;
	}
	fm_p1v80_en = !!(fm_p1v80_en & BIT(U618_WORKAROUND_FM_P1V80_EN_BIT));
	LOG_INF("Board rev: %d, FM_P1V80_EN: %d", rev_id, fm_p1v80_en);

	/* Follow the Rainbow flow: check all workaround registers first. */
	if (!u618_reg_read(0x00a8, read_00a8, sizeof(read_00a8)))
		return false;
	memcpy(&clk_312_5_reinit_event_data[0], read_00a8, sizeof(read_00a8));
	LOG_INF("Read CLK U618 reg 0x00A8: %02x%02x%02x%02x", read_00a8[0], read_00a8[1],
		read_00a8[2], read_00a8[3]);
	has_error |= memcmp(read_00a8, expected_00a8, sizeof(expected_00a8)) != 0;

	if (!u618_reg_read(0x0080, read_0080, sizeof(read_0080)))
		return false;
	memcpy(&clk_312_5_reinit_event_data[4], read_0080, sizeof(read_0080));
	LOG_INF("Read CLK U618 reg 0x0080: %02x", read_0080[0]);
	has_error |= memcmp(read_0080, expected_0080, sizeof(expected_0080)) != 0;

	if (!u618_reg_read(0x0088, read_0088, sizeof(read_0088)))
		return false;
	memcpy(&clk_312_5_reinit_event_data[5], read_0088, sizeof(read_0088));
	LOG_INF("Read CLK U618 reg 0x0088: %02x%02x", read_0088[0], read_0088[1]);
	has_error |= memcmp(read_0088, expected_0088, sizeof(expected_0088)) != 0;

	if (!has_error) {
		LOG_INF("CLK U618 workaround values are already correct");
		return true;
	}

	/* FM_P1V80_EN status is active low: bit 4 = 1 permits the workaround. */
	if (!fm_p1v80_en) {
		LOG_ERR("CLK U618 values are unexpected while FM_P1V80_EN is asserted");
		error_log_event(CLK_312_5MHZ_REINIT_ERR_CODE, LOG_ASSERT);
		return false;
	}

	LOG_WRN("FM_P1V80_EN=%d, CLK U618 registers require workaround: "
		"0x00A8=%02x%02x%02x%02x, 0x0080=%02x, 0x0088=%02x%02x; applying workaround",
		fm_p1v80_en, read_00a8[0], read_00a8[1], read_00a8[2], read_00a8[3], read_0080[0],
		read_0088[0], read_0088[1]);
	has_error = false;
	if (!u618_reg_write(0x00a8, expected_00a8, sizeof(expected_00a8))) {
		has_error = true;
	} else if (!u618_reg_read(0x00a8, read_00a8, sizeof(read_00a8))) {
		has_error = true;
	} else {
		memcpy(&clk_312_5_reinit_event_data[0], read_00a8, sizeof(read_00a8));
	}
	if (memcmp(read_00a8, expected_00a8, sizeof(expected_00a8)) != 0) {
		LOG_ERR("CLK U618 reg 0x00A8 workaround verification failed");
		has_error = true;
	}
	k_msleep(U618_WORKAROUND_WRITE_DELAY_MS);

	if (!u618_reg_write(0x0080, expected_0080, sizeof(expected_0080))) {
		has_error = true;
	} else if (!u618_reg_read(0x0080, read_0080, sizeof(read_0080))) {
		has_error = true;
	} else {
		memcpy(&clk_312_5_reinit_event_data[4], read_0080, sizeof(read_0080));
	}
	if (memcmp(read_0080, expected_0080, sizeof(expected_0080)) != 0) {
		LOG_ERR("CLK U618 reg 0x0080 workaround verification failed");
		has_error = true;
	}
	k_msleep(U618_WORKAROUND_WRITE_DELAY_MS);

	if (!u618_reg_write(0x0088, expected_0088, sizeof(expected_0088))) {
		has_error = true;
	} else if (!u618_reg_read(0x0088, read_0088, sizeof(read_0088))) {
		has_error = true;
	} else {
		memcpy(&clk_312_5_reinit_event_data[5], read_0088, sizeof(read_0088));
	}
	if (memcmp(read_0088, expected_0088, sizeof(expected_0088)) != 0) {
		LOG_ERR("CLK U618 reg 0x0088 workaround verification failed");
		has_error = true;
	}

	/* Re-initialize APLL with the intended Rainbow 0x00 -> 0x02 -> 0x00 sequence. */
	for (uint8_t i = 0; i < ARRAY_SIZE(apll_reinit); i++) {
		if (!u618_reg_write(0x0d00, &apll_reinit[i], 1))
			has_error = true;
	}

	if (!plat_read_cpld(VR_EN_PIN_READING_5, &fm_p1v80_en, 1)) {
		LOG_ERR("Failed to read FM_P1V80_EN after applying CLK U618 workaround");
		has_error = true;
	} else {
		fm_p1v80_en = !!(fm_p1v80_en & BIT(U618_WORKAROUND_FM_P1V80_EN_BIT));
	}

	/* Retain the incident if FM_P1V80_EN becomes asserted during re-init. */
	if (has_error || !fm_p1v80_en) {
		LOG_ERR("CLK U618 re-init result: error=%d, FM_P1V80_EN=%d", has_error,
			fm_p1v80_en);
		error_log_event(CLK_312_5MHZ_REINIT_ERR_CODE, LOG_ASSERT);
		return false;
	}

	LOG_INF("CLK U618 workaround applied successfully");
	return true;
}

/* get clock error data and send to bmc*/
bool clock_get_error_data(uint16_t error_code, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t clk_idx = error_code & 0xF;
	uint8_t lock_status = 0x00;
	uint8_t bmc_err_type = 0;
	bool ret = true;
	switch (clk_idx) {
	case CLK_100MHZ_ERR_IDX:
		lock_status = clk_100mhz_get_lock_status_u86();
		if (lock_status == 0xFF) {
			LOG_ERR("Failed to get 100MHz clock(U86) lock status");
			ret = false;
		}
		data[0] = lock_status;
		bmc_err_type = CLOCK_APLL_UNLOCK_EVENT;
		break;
	case CLK_312_5MHZ_ERR_IDX:
		lock_status = clk_312_5mhz_get_lock_status_u618();
		if (lock_status == 0xFF) {
			LOG_ERR("Failed to get 312.5MHz clock(U618) lock status");
			ret = false;
		}
		data[0] = lock_status;
		bmc_err_type = CLK_312_5M_APLL_UNLOCK_EVENT;
		break;
	case CLK_BUF0_100M_LOSB_PLD:
		bmc_err_type = CLK_BUF0_100M_LOSB_PLD_EVENT;
		if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
			ret = false;
		break;
	case CLK_BUF1_100M_LOSB_PLD:
		bmc_err_type = CLK_BUF1_100M_LOSB_PLD_EVENT;
		if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
			ret = false;
		break;
	case CLK_BUF2_100M_LOSB_PLD:
		bmc_err_type = CLK_BUF2_100M_LOSB_PLD_EVENT;
		if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &data[0], 1))
			ret = false;
		break;
	case CLK_312_5MHZ_REINIT_ERR_IDX:
		memcpy(data, clk_312_5_reinit_event_data, CLK_312_5MHZ_REINIT_EVENT_DATA_LEN);
		/* 0x8A06 is stored in blackbox only; do not emit another BMC event. */
		return true;
	default:
		LOG_ERR("Unsupported clock error code: 0x%04x", error_code);
		return false;
	}
	packaged_bmc_log(ARKE_FAULT, bmc_err_type, data[0], 0);

	return ret;
}
