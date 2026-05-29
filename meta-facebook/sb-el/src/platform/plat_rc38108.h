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

#ifndef PLAT_RC38108_H
#define PLAT_RC38108_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ──────────────────────────────────────────────────────────────────────────────
 *  Hardware topology
 *
 *  MMC GPIO30  ──► U694_EN_R  ──► enables MMC I2C path to EEPROM
 *  MMC GPIO54  ◄── RC38108 GPIO1  (APLL lock status output, normally)
 *
 *  RC38108 sits on I2C_BUS_RC38108 at address RC38108_I2C_ADDR.
 *  The external AT24-style EEPROM sits on I2C_BUS_EEPROM at address
 *  RC38108_EEPROM_I2C_ADDR, gated by U694_EN_R.
 * ──────────────────────────────────────────────────────────────────────────────
 */

/* ── MMC GPIO indices (project-specific, refer to plat_gpio.h) ── */
#define MMC_GPIO_U694_EN_R   30   /* GPIO30: high = enable MMC→EEPROM I2C path */
#define MMC_GPIO_APLL_LOCK   54   /* GPIO54: input from RC38108 GPIO1 (APLL lock) */

/* ── RC38108 I2C bus / address ── */
#define I2C_BUS_RC38108          5    /* refer to board */
#define RC38108_I2C_ADDR         0x09 /* 7-bit default; left-shift done by hal_i2c */

/* ── External EEPROM I2C bus / address ── */
#define I2C_BUS_EEPROM           5    /* same bus as RC38108 (gated by U694_EN_R) */
#define RC38108_EEPROM_I2C_ADDR  0x50 /* AT24 default; matches RC38108 EEPROM_ADDR_CNFG */
#define EEPROM_PAGE_SIZE         32   /* bytes per write page (AT24C16 = 16B, AT24C32+ = 32B) */
#define EEPROM_WRITE_CYCLE_MS    5    /* max write cycle time (ms) per page */
#define EEPROM_MAX_ADDR          0x07FF /* 2 KB EEPROM */

/* ── RC38108 register addresses (16-bit, 2B addressing mode) ── */

/* GLOBAL module (base 0x0000) */
#define RC38108_REG_DEVICE_STS          0x0024  /* 32-bit: device/EEPROM ready status */

/* GLOBAL DEVICE_STS bit positions */
#define RC38108_DEVICE_STS_READY_BIT         6  /* device_ready_sts  – EEPROM load complete */
#define RC38108_DEVICE_STS_EEPROM_VALID_BIT  5  /* eeprom_config_valid_sts */

/* GPIO[1] module (base 0x0248) */
#define RC38108_REG_GPIO1_CNFG          0x0248  /* 16-bit: GPIO mode configuration */
#define RC38108_REG_GPIO1_PAD_CNFG      0x024B  /*  8-bit: GPIO pad configuration  */

/* gpio_func values (bits[7:0] of GPIO_CNFG) */
#define RC38108_GPIO_FUNC_APLL_LOCK     0x1B   /* APLL lock output (default for GPIO1) */
#define RC38108_GPIO_FUNC_INPUT         0x20   /* General purpose input */

/* pad_gpio_oe_b (bit 3 of GPIO_PAD_CNFG): 0 = output enabled, 1 = output disabled */
#define RC38108_PAD_OE_B_BIT            3

/* EEPROM module (base 0x08E0) */
#define RC38108_REG_EEPROM_EVENT        0x08E9  /*  8-bit: EEPROM event/error flags */

/* EEPROM_EVENT bit positions */
#define RC38108_EEPROM_EVT_BAD_BIT       3  /* eeprom_bad_evt – EEPROM not detected */
#define RC38108_EEPROM_EVT_EMPTY_BIT     2  /* eeprom_config_empty_evt */
#define RC38108_EEPROM_EVT_LOAD_FAIL_BIT 1  /* eeprom_load_fail_evt */
#define RC38108_EEPROM_EVT_CRC_ERR_BIT   0  /* eeprom_crc_err_evt   */

/* ── Misc ── */
#define RC38108_DEVICE_READY_POLL_MAX    10  /* maximum poll iterations */
#define RC38108_DEVICE_READY_POLL_MS     20   /* ms between polls */


/* ── Return codes ── */
typedef enum {
	RC38108_UPDATE_SUCCESS        = 0,
	RC38108_UPDATE_ERR_NOT_READY  = -1, /* device_ready_sts never became 1 */
	RC38108_UPDATE_ERR_I2C        = -2, /* I2C transaction failed */
	RC38108_UPDATE_ERR_VERIFY     = -3, /* read-back mismatch after write */
	RC38108_UPDATE_ERR_BAD_PARAM  = -4, /* NULL pointer or zero length */
} rc38108_update_err_t;

/* ── Public API ── */

/**
 * pre_rc38108_eeprom_update() - Pre-update steps before writing the EEPROM.
 *
 * Must be called before rc38108_eeprom_update(). Executes:
 *   1. Poll device_ready_sts until RC38108 has finished EEPROM loading.
 *   2. Reconfigure RC38108 GPIO1 from APLL-lock output → general-purpose input.
 *      Mask MMC GPIO54 monitoring during the update.
 *   3. Assert U694_EN_R (GPIO30 HIGH) to enable the MMC I2C path to the EEPROM.
 *
 * Returns 0 on success, 1 on failure.
 */
uint8_t pre_rc38108_eeprom_update(void *fw_update_param);

/**
 * rc38108_eeprom_update() - Write and verify the EEPROM, then restore state.
 *
 * Must be preceded by a successful pre_rc38108_eeprom_update(). Executes:
 *   1. Write @data to the EEPROM page-by-page starting at @eeprom_start_addr.
 *   2. Read back and verify every byte.
 *   3. Deassert U694_EN_R (GPIO30 LOW) to isolate the EEPROM.
 *   4. Restore RC38108 GPIO1 to APLL-lock output and resume GPIO54 monitoring.
 *
 * @data:              Pointer to the firmware image to write.
 * @data_len:          Number of bytes to write (must be ≤ EEPROM size).
 * @eeprom_start_addr: First EEPROM byte address to write (usually 0x0000).
 *
 * Returns RC38108_UPDATE_SUCCESS (0) on success, negative on failure.
 */
rc38108_update_err_t rc38108_eeprom_update(const uint8_t *data, uint16_t data_len,
					   uint16_t eeprom_start_addr);

#endif /* PLAT_RC38108_H */