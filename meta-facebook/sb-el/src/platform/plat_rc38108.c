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

/*
 * plat_rc38108.c
 *
 * RC38108 clock generator EEPROM firmware update driver (OpenBIC / Zephyr).
 *
 * Register references – RC38312/RC38112/RC38208/RC38108 Programming Guide
 * (R31DS0028EU0103 Rev.1.03, Aug 20 2025):
 *
 *   DEVICE_STS      0x0024  – device_ready_sts [6], eeprom_config_valid_sts [5]
 *   GPIO[1] CNFG    0x0248  – gpio_func [7:0]
 *                              0x1B = APLL lock output (default)
 *                              0x20 = General purpose input
 *   GPIO[1] PAD     0x024B  – pad_gpio_oe_b [3] (0=out enabled, 1=out disabled)
 *   EEPROM_EVENT    0x08E9  – load-error flags [3:0]
 *
 * I2C protocol used: 2-byte (2B) offset addressing (Section 3.2).
 *   Write: [DEV_ADDR|W] [ADDR_MSB] [ADDR_LSB] [DATA…]
 *   Read:  [DEV_ADDR|W] [ADDR_MSB] [ADDR_LSB]
 *          [DEV_ADDR|R] [DATA…]
 */

#include <zephyr.h>
#include <string.h>
#include <logging/log.h>

#include "hal_i2c.h"
#include "hal_gpio.h"
#include "plat_rc38108.h"

LOG_MODULE_REGISTER(rc38108_update, LOG_LEVEL_ERR);

/* ────────────────────────────────────────────────────────────────────────────
 * Internal helpers – RC38108 register read / write (2B addressing)
 * ────────────────────────────────────────────────────────────────────────────*/

/**
 * rc38108_reg_read() - Read bytes from an RC38108 register via direct I2C.
 *
 * Used for non-GPIO registers (e.g. DEVICE_STS, EEPROM_EVENT) where direct
 * HAL access is fine.
 *
 * @reg_addr: 16-bit register address (2B mode).
 * @buf:      Destination buffer.
 * @len:      Number of bytes to read (1–4 for typical registers).
 *
 * Returns 0 on success, negative errno on failure.
 */
static int rc38108_reg_read(uint16_t reg_addr, uint8_t *buf, uint8_t len)
{
	if (!buf || len == 0)
		return -EINVAL;

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_RC38108;
	msg.target_addr = RC38108_I2C_ADDR;

	/*
	 * 2B addressing: send [ADDR_MSB, ADDR_LSB] as a write-without-stop,
	 * then repeated-start read.
	 */
	msg.tx_len = 2;
	msg.data[0] = (uint8_t)(reg_addr >> 8);
	msg.data[1] = (uint8_t)(reg_addr & 0xFF);
	msg.rx_len = len;

	int ret = i2c_master_read(&msg, 3);
	if (ret) {
		LOG_ERR("RC38108 reg read 0x%04X failed (%d)", reg_addr, ret);
		return ret;
	}

	memcpy(buf, msg.data, len);
	return 0;
}

/**
 * rc38108_reg_write() - Write bytes to an RC38108 register via I2C.
 *
 * @reg_addr: 16-bit register address (2B mode).
 * @buf:      Source data.
 * @len:      Number of bytes to write.
 *
 * Returns 0 on success, negative errno on failure.
 */
static int rc38108_reg_write(uint16_t reg_addr, const uint8_t *buf, uint8_t len)
{
	if (!buf || len == 0 || (2 + len) > I2C_BUFF_SIZE)
		return -EINVAL;

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_RC38108;
	msg.target_addr = RC38108_I2C_ADDR;
	msg.tx_len = 2 + len;

	msg.data[0] = (uint8_t)(reg_addr >> 8);   /* address MSB */
	msg.data[1] = (uint8_t)(reg_addr & 0xFF); /* address LSB */
	memcpy(&msg.data[2], buf, len);

	int ret = i2c_master_write(&msg, 3);
	if (ret) {
		LOG_ERR("RC38108 reg write 0x%04X failed (%d)", reg_addr, ret);
		return ret;
	}

	return 0;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Internal helpers – external EEPROM read / write (AT24-style, 2-byte addr)
 * ────────────────────────────────────────────────────────────────────────────*/

/**
 * eeprom_page_write() - Write up to EEPROM_PAGE_SIZE bytes to the EEPROM.
 *
 * The EEPROM must have its I2C path enabled (U694_EN_R HIGH) before calling.
 * A k_msleep(EEPROM_WRITE_CYCLE_MS) delay is required after each page write.
 *
 * @mem_addr: 16-bit EEPROM memory address.
 * @src:      Source data pointer.
 * @len:      Number of bytes (≤ EEPROM_PAGE_SIZE).
 */
static int eeprom_page_write(uint16_t mem_addr, const uint8_t *src, uint8_t len)
{
	if (!src || len == 0 || len > EEPROM_PAGE_SIZE)
		return -EINVAL;
	if ((2 + len) > I2C_BUFF_SIZE)
		return -EINVAL;

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS_EEPROM;
	msg.target_addr = RC38108_EEPROM_I2C_ADDR;
	msg.tx_len = 2 + len;

	msg.data[0] = (uint8_t)(mem_addr >> 8);   /* memory address MSB */
	msg.data[1] = (uint8_t)(mem_addr & 0xFF); /* memory address LSB */
	memcpy(&msg.data[2], src, len);

	int ret = i2c_master_write(&msg, 3);
	if (ret) {
		LOG_ERR("EEPROM page write @0x%04X failed (%d)", mem_addr, ret);
		return ret;
	}

	/* Wait for internal write cycle to complete */
	k_msleep(EEPROM_WRITE_CYCLE_MS);
	return 0;
}

/**
 * eeprom_read() - Sequential read from the EEPROM.
 *
 * @mem_addr: 16-bit EEPROM memory address.
 * @dst:      Destination buffer.
 * @len:      Number of bytes to read.
 */
static int eeprom_read(uint16_t mem_addr, uint8_t *dst, uint16_t len)
{
	if (!dst || len == 0)
		return -EINVAL;

	/*
	 * Read in chunks sized to I2C_BUFF_SIZE.
	 * hal_i2c fills msg.data[] with rx bytes after the tx phase.
	 */
	uint16_t remaining = len;
	uint16_t offset = 0;

	while (remaining > 0) {
		uint8_t chunk = (remaining > I2C_BUFF_SIZE) ? I2C_BUFF_SIZE : (uint8_t)remaining;

		I2C_MSG msg = { 0 };
		msg.bus = I2C_BUS_EEPROM;
		msg.target_addr = RC38108_EEPROM_I2C_ADDR;
		msg.tx_len = 2;
		msg.data[0] = (uint8_t)((mem_addr + offset) >> 8);
		msg.data[1] = (uint8_t)((mem_addr + offset) & 0xFF);
		msg.rx_len = chunk;

		int ret = i2c_master_read(&msg, 3);
		if (ret) {
			LOG_ERR("EEPROM read @0x%04X failed (%d)", mem_addr + offset, ret);
			return ret;
		}

		memcpy(dst + offset, msg.data, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	return 0;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Step 1 – Wait for RC38108 to finish EEPROM loading
 *
 * Spec ref: DEVICE_STS register (0x0024), bit[6] = device_ready_sts
 *   "Set to 1 when the configuration load (OTP and/or EEPROM) completes
 *    during the startup sequence."
 * ────────────────────────────────────────────────────────────────────────────*/
static rc38108_update_err_t rc38108_wait_device_ready(void)
{
	for (int i = 0; i < RC38108_DEVICE_READY_POLL_MAX; i++) {
		/*
		 * DEVICE_STS is a 32-bit register; byte 0 (LSB, at offset 0x24)
		 * contains device_ready_sts [6] and eeprom_config_valid_sts [5].
		 * Read 1 byte from the LSB address.
		 */
		uint8_t sts = 0;
		int ret = rc38108_reg_read(RC38108_REG_DEVICE_STS, &sts, 1);
		if (ret)
			return RC38108_UPDATE_ERR_I2C;

		if (sts & BIT(RC38108_DEVICE_STS_READY_BIT)) {

			/* Bonus: log whether EEPROM config was valid */
			if (sts & BIT(RC38108_DEVICE_STS_EEPROM_VALID_BIT))
				LOG_INF("RC38108: eeprom_config_valid_sts=1 (EEPROM load OK)");
			else
				LOG_WRN("RC38108: eeprom_config_valid_sts=0 "
					"(no valid EEPROM config found)");

			return RC38108_UPDATE_SUCCESS;
		}

		LOG_DBG("RC38108: not ready yet (DEVICE_STS=0x%02X), poll %d/%d",
			sts, i + 1, RC38108_DEVICE_READY_POLL_MAX);
		k_msleep(RC38108_DEVICE_READY_POLL_MS);
	}

	LOG_ERR("RC38108: device_ready_sts never set after %d ms",
		RC38108_DEVICE_READY_POLL_MAX * RC38108_DEVICE_READY_POLL_MS);
	return RC38108_UPDATE_ERR_NOT_READY;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Step 2 – Reconfigure RC38108 GPIO1: APLL-lock output → GP input
 *
 * Spec ref:
 *   GPIO[1] CNFG  0x0248  bits[7:0] = gpio_func
 *     0x1B = APLL lock output (default)
 *     0x20 = General purpose input
 *   GPIO[1] PAD   0x024B  bit[3] = pad_gpio_oe_b
 *     0 = output buffer enabled  (needed for any output function)
 *     1 = output buffer disabled (REQUIRED when gpio_func is an input)
 * ────────────────────────────────────────────────────────────────────────────*/
static rc38108_update_err_t rc38108_gpio1_set_input(void)
{
	int ret;

	/*
	 * 1a. Read current GPIO_CNFG to preserve bits [15:8] before modifying
	 *     gpio_func in bits [7:0].
	 *     GPIO_CNFG is a 16-bit register; read both bytes (LSB first in
	 *     the RC38108 little-endian register layout).
	 */
	uint8_t cnfg[2] = { 0 };
	ret = rc38108_reg_read(RC38108_REG_GPIO1_CNFG, cnfg, sizeof(cnfg));
	if (ret) {
		LOG_ERR("RC38108: I2C read GPIO1_CNFG failed (%d)", ret);
		return RC38108_UPDATE_ERR_I2C;
	}

	LOG_DBG("RC38108: GPIO1_CNFG before = 0x%02X%02X", cnfg[1], cnfg[0]);

	/* 1b. Set gpio_func = 0x20 (GP input) in byte 0 (bits[7:0]) */
	cnfg[0] = RC38108_GPIO_FUNC_INPUT;
	ret = rc38108_reg_write(RC38108_REG_GPIO1_CNFG, cnfg, sizeof(cnfg));
	if (ret)
		return RC38108_UPDATE_ERR_I2C;

	/*
	 * 2a. Read current GPIO_PAD_CNFG (8-bit register at 0x024B).
	 *     Preserve bits [7:4] and [2:0]; only bit[3] (pad_gpio_oe_b)
	 *     needs to change.
	 */
	uint8_t pad = 0;
	ret = rc38108_reg_read(RC38108_REG_GPIO1_PAD_CNFG, &pad, 1);
	if (ret) {
		LOG_ERR("RC38108: I2C read GPIO1_PAD_CNFG failed (%d)", ret);
		return RC38108_UPDATE_ERR_I2C;
	}

	LOG_DBG("RC38108: GPIO1_PAD_CNFG before = 0x%02X", pad);

	/* 2b. Set pad_gpio_oe_b = 1 to disable output buffer (required for input) */
	pad |= BIT(RC38108_PAD_OE_B_BIT);
	ret = rc38108_reg_write(RC38108_REG_GPIO1_PAD_CNFG, &pad, 1);
	if (ret)
		return RC38108_UPDATE_ERR_I2C;

	LOG_INF("RC38108: GPIO1 reconfigured as input (gpio_func=0x%02X, pad=0x%02X)",
		RC38108_GPIO_FUNC_INPUT, pad);

	/*
	 * Step 2b (MMC side): stop treating GPIO54 as APLL lock indicator.
	 * Disable the interrupt so the MCU ignores the pin state while it is
	 * driven by RC38108 as input.
	 */
	LOG_INF("MMC: masking GPIO%d (APLL lock monitor) during EEPROM update",
		MMC_GPIO_APLL_LOCK);
	gpio_interrupt_conf(MMC_GPIO_APLL_LOCK, GPIO_INT_DISABLE);

	return RC38108_UPDATE_SUCCESS;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Step 7 – Restore RC38108 GPIO1: GP input → APLL-lock output
 * ────────────────────────────────────────────────────────────────────────────*/
static rc38108_update_err_t rc38108_gpio1_restore_apll_lock(void)
{
	LOG_INF("RC38108: GPIO1 general-purpose input → APLL-lock output");

	int ret;

	/*
	 * Order matters:
	 *   First re-enable the output buffer (pad_gpio_oe_b = 0),
	 *   then set gpio_func = APLL lock (0x1B).
	 * This avoids a transient where the pad is driving while gpio_func
	 * is still "input".
	 */

	/* Restore pad_gpio_oe_b = 0 (output buffer enabled) */
	uint8_t pad = 0;
	ret = rc38108_reg_read(RC38108_REG_GPIO1_PAD_CNFG, &pad, 1);
	if (ret) {
		LOG_ERR("RC38108: I2C read GPIO1_PAD_CNFG failed (%d)", ret);
		return RC38108_UPDATE_ERR_I2C;
	}

	LOG_DBG("RC38108: GPIO1_PAD_CNFG restore-read = 0x%02X", pad);
	pad &= ~BIT(RC38108_PAD_OE_B_BIT); /* clear bit[3] → enable output buffer */
	ret = rc38108_reg_write(RC38108_REG_GPIO1_PAD_CNFG, &pad, 1);
	if (ret)
		return RC38108_UPDATE_ERR_I2C;

	/* Restore gpio_func = APLL lock (0x1B) */
	uint8_t cnfg[2] = { 0 };
	ret = rc38108_reg_read(RC38108_REG_GPIO1_CNFG, cnfg, sizeof(cnfg));
	if (ret) {
		LOG_ERR("RC38108: I2C read GPIO1_CNFG failed (%d)", ret);
		return RC38108_UPDATE_ERR_I2C;
	}

	LOG_DBG("RC38108: GPIO1_CNFG restore-read = 0x%02X%02X", cnfg[1], cnfg[0]);
	cnfg[0] = RC38108_GPIO_FUNC_APLL_LOCK;
	ret = rc38108_reg_write(RC38108_REG_GPIO1_CNFG, cnfg, sizeof(cnfg));
	if (ret)
		return RC38108_UPDATE_ERR_I2C;

	LOG_INF("RC38108: GPIO1 restored to APLL lock output (gpio_func=0x%02X)",
		RC38108_GPIO_FUNC_APLL_LOCK);

	/* Re-enable GPIO54 interrupt monitoring on MMC side */
	LOG_INF("MMC: restoring GPIO%d (APLL lock monitor) interrupt", MMC_GPIO_APLL_LOCK);
	gpio_interrupt_conf(MMC_GPIO_APLL_LOCK, GPIO_INT_EDGE_BOTH);

	return RC38108_UPDATE_SUCCESS;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Steps 3 & 6 – U694_EN_R gate control (GPIO30)
 * ────────────────────────────────────────────────────────────────────────────*/
static void u694_en_r_set(uint8_t level)
{
	gpio_set(MMC_GPIO_U694_EN_R, level);
	LOG_INF("MMC: GPIO%d (U694_EN_R) → %s, MMC I2C→EEPROM path %s",
		MMC_GPIO_U694_EN_R,
		level ? "HIGH" : "LOW",
		level ? "ENABLED" : "DISABLED");

	/* Small settling delay after enabling/disabling the bus switch */
	k_msleep(1);
}

/* ────────────────────────────────────────────────────────────────────────────
 * Steps 4 & 5 – EEPROM write + verify
 * ────────────────────────────────────────────────────────────────────────────*/
static rc38108_update_err_t eeprom_write_and_verify(const uint8_t *data, uint16_t data_len,
						     uint16_t start_addr)
{
	/* ── Write phase ── */
	LOG_INF("EEPROM: writing %u bytes starting at 0x%04X ...", data_len, start_addr);

	uint16_t written = 0;
	while (written < data_len) {
		uint16_t mem_addr = start_addr + written;

		/*
		 * Limit each transaction to one EEPROM page.
		 * If the first address is not page-aligned, limit to the
		 * remaining bytes in the current page so we never cross a
		 * page boundary within a single write (AT24 requirement).
		 */
		uint16_t page_offset = mem_addr % EEPROM_PAGE_SIZE;
		uint8_t chunk = EEPROM_PAGE_SIZE - page_offset;
		if (chunk > (data_len - written))
			chunk = data_len - written;

		int ret = eeprom_page_write(mem_addr, data + written, chunk);
		if (ret) {
			LOG_ERR("EEPROM: write failed at 0x%04X", mem_addr);
			return RC38108_UPDATE_ERR_I2C;
		}

		written += chunk;
		LOG_DBG("EEPROM: wrote %u/%u bytes", written, data_len);
	}

	LOG_INF("EEPROM: write complete (%u bytes)", data_len);
	/* ── Verify phase ── */

	uint8_t read_buf[EEPROM_PAGE_SIZE];
	uint16_t verified = 0;

	while (verified < data_len) {
		uint16_t mem_addr = start_addr + verified;
		uint8_t chunk = (data_len - verified < EEPROM_PAGE_SIZE) ?
					(uint8_t)(data_len - verified) :
					EEPROM_PAGE_SIZE;

		int ret = eeprom_read(mem_addr, read_buf, chunk);
		if (ret) {
			LOG_ERR("EEPROM: read-back failed at 0x%04X", mem_addr);
			return RC38108_UPDATE_ERR_I2C;
		}

		if (memcmp(read_buf, data + verified, chunk) != 0) {
			/* Find first mismatch byte for a precise error log */
			for (uint8_t i = 0; i < chunk; i++) {
				if (read_buf[i] != data[verified + i]) {
					LOG_ERR("EEPROM: verify mismatch at 0x%04X: "
						"expected 0x%02X got 0x%02X",
						mem_addr + i,
						data[verified + i],
						read_buf[i]);
					break;
				}
			}
			return RC38108_UPDATE_ERR_VERIFY;
		}

		verified += chunk;
		LOG_DBG("EEPROM: verified %u/%u bytes", verified, data_len);
	}

	LOG_INF("EEPROM: verify OK (%u bytes match)", data_len);
	return RC38108_UPDATE_SUCCESS;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────────────────────────────*/

uint8_t pre_rc38108_eeprom_update(void *fw_update_param)
{
	rc38108_update_err_t err;

	/* ── Step 1: Wait for RC38108 to finish initial EEPROM loading ── */
	err = rc38108_wait_device_ready();
	if (err) {
		LOG_ERR("pre Step 1 FAILED: RC38108 not ready (%d)", err);
		return 1;
	}

	/*
	 * ── Step 2: Reconfigure RC38108 GPIO1 to input; mask MMC GPIO54 ──
	 *
	 * This prevents GPIO1 from being driven as APLL-lock output while
	 * the EEPROM is being written.  It also prevents the MCU from
	 * misinterpreting APLL lock changes on GPIO54 during the update.
	 */
	err = rc38108_gpio1_set_input();
	if (err) {
		LOG_ERR("pre Step 2 FAILED: could not reconfigure RC38108 GPIO1 (%d)", err);
		return 1;
	}

	/* ── Step 3: Assert U694_EN_R HIGH – enable MMC I2C → EEPROM path ── */
	u694_en_r_set(GPIO_HIGH);

	return 0;
}

rc38108_update_err_t rc38108_eeprom_update(const uint8_t *data, uint16_t data_len,
					   uint16_t eeprom_start_addr)
{
	rc38108_update_err_t err;

	if (!data || data_len == 0) {
		LOG_ERR("rc38108_eeprom_update: invalid parameters");
		return RC38108_UPDATE_ERR_BAD_PARAM;
	}

	if ((uint32_t)eeprom_start_addr + data_len > (uint32_t)EEPROM_MAX_ADDR + 1) {
		LOG_ERR("rc38108_eeprom_update: data range 0x%04X–0x%04X exceeds EEPROM size",
			eeprom_start_addr, eeprom_start_addr + data_len - 1);
		return RC38108_UPDATE_ERR_BAD_PARAM;
	}

	LOG_INF("=== RC38108 EEPROM update: %u bytes @ 0x%04X ===",
		data_len, eeprom_start_addr);

	/* ── Write then verify EEPROM ── */
	err = eeprom_write_and_verify(data, data_len, eeprom_start_addr);
	if (err) {
		LOG_ERR("EEPROM write/verify error (%d)", err);
		/*
		 * Always deassert the gate and restore GPIO1 even on failure,
		 * so the system is left in a safe state.
		 */
		goto cleanup;
	}

	LOG_INF("EEPROM written and verified successfully");

cleanup:
	/* ── Deassert U694_EN_R LOW – disable MMC I2C → EEPROM path ── */
	u694_en_r_set(GPIO_LOW);

	/*
	 * ── Restore RC38108 GPIO1 to APLL-lock output; unmask GPIO54 ──
	 *
	 * Do this even if the EEPROM write failed, so the clock monitoring
	 * function is restored regardless of the update outcome.
	 */
	rc38108_update_err_t restore_err = rc38108_gpio1_restore_apll_lock();
	if (restore_err) {
		LOG_ERR("Restore FAILED: could not restore RC38108 GPIO1 (%d)", restore_err);
		/* Prefer returning the original error if there was one */
		if (err == RC38108_UPDATE_SUCCESS)
			err = restore_err;
	}

	if (err == RC38108_UPDATE_SUCCESS)
		LOG_INF("=== RC38108 EEPROM update: COMPLETE ===");
	else
		LOG_ERR("=== RC38108 EEPROM update: FAILED (err=%d) ===", err);

	return err;
}