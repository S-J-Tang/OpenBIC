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
 * Debug shell for the ASIC flash ODS (output driver strength) workaround.
 *
 * "info" only reads registers, "run" replays the DC-on sequence of
 * plat_change_flash_ods() step by step with verbose logging.
 * "wbfix" restores a Winbond SR2 that was corrupted by "run ... force"
 * (CMP/SRL/QE), only allowed after a flash power cycle (SRL=0).
 * "wptest" checks write protection on an all-FF 4KB sector: program one byte
 * 0x00, read back; if the write went through, the sector is erased back to FF.
 *
 * NOTE: both commands hold the selected ASIC in POR/SYS reset while the SPI
 * mux is switched to BIC, then restore the original CPLD reset register.
 * If the ASIC was running, it will be reset by this command.
 */

#include <zephyr.h>
#include <shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/spi_nor.h>
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_cpld.h"
#include "plat_class.h"

#define ODS_CMD_RDID 0x9F
#define ODS_CMD_RDSCUR_MXIC 0x2B
#define ODS_CMD_RDSR2_WINBOND 0x35
#define ODS_CMD_WRSR2_WINBOND 0x31
#define ODS_WB_SR1_SRP BIT(7)
#define ODS_WB_SR2_SRL BIT(0)
#define ODS_WB_SR2_QE BIT(1)
#define ODS_WB_SR2_LB_MASK 0x38 /* LB3..LB1, OTP: cannot be cleared */
#define ODS_JEDEC_MFR_MXIC 0xC2
#define ODS_JEDEC_MFR_WINBOND 0xEF
#define ODS_SR_WIP BIT(0)
#define ODS_SR_WEL BIT(1)
#define ODS_WIP_POLL_TIMEOUT_MS 100
#define ODS_CMD_READ4B 0x13
#define ODS_CMD_PP4B 0x12
#define ODS_CMD_SE4B 0x21
#define ODS_SECTOR_SIZE 0x1000
#define ODS_WPTEST_DEF_ADDR 0x01FFF000 /* last 4KB sector of 256Mb */
#define ODS_WPTEST_CHUNK 64

enum ods_mode {
	ODS_MODE_INFO,
	ODS_MODE_RUN,
	ODS_MODE_WBFIX,
	ODS_MODE_WPTEST,
};

static uint32_t ods_wptest_addr = ODS_WPTEST_DEF_ADDR;

typedef struct {
	const char *name;
	enum asic_spi_chip chip;
	uint8_t mux_gpio;
	uint8_t sel0;
	uint8_t sel1;
	uint8_t por_bit;
	uint8_t sys_bit;
} ods_chip_t;

static const ods_chip_t ods_chips[] = {
	{ "hamsa", ASIC_SPI_HAMSA, SPI_HAMSA_MUX_IN1, 0, 0, ASIC_HAMSA_POR_BIT,
	  ASIC_HAMSA_SYS_RST_BIT },
	{ "nuwa0", ASIC_SPI_NUWA0, SPI_NUWA0_MUX_IN1, 1, 0, ASIC_NUWA0_POR_BIT,
	  ASIC_NUWA0_SYS_RST_BIT },
	{ "nuwa1", ASIC_SPI_NUWA1, SPI_NUWA1_MUX_IN1, 0, 1, ASIC_NUWA1_POR_BIT,
	  ASIC_NUWA1_SYS_RST_BIT },
};

static int ods_xfer(const struct device *fiu, uint8_t *tx, size_t tx_len, uint8_t *rx,
		    size_t rx_len)
{
	struct spi_config cfg = {
		.frequency = 1000000,
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = SPI_FIU0_CS1_SLAVE,
	};
	struct spi_buf tx_buf = { .buf = tx, .len = tx_len };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	if (rx_len == 0) {
		return spi_write(fiu, &cfg, &tx_set);
	}

	struct spi_buf rx_buf = { .buf = rx, .len = rx_len };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	return spi_transceive(fiu, &cfg, &tx_set, &rx_set);
}

/* read a 1-byte register with a 1-byte opcode, print and return rc */
static int ods_read_reg(const struct shell *shell, const struct device *fiu, uint8_t op,
			const char *tag, uint8_t *val)
{
	*val = 0;
	int rc = ods_xfer(fiu, &op, 1, val, 1);
	shell_print(shell, "    [%02Xh] %-12s = 0x%02x (rc=%d)", op, tag, *val, rc);
	return rc;
}

static int ods_read_hcr(const struct shell *shell, const struct device *fiu, uint8_t *val)
{
	uint8_t op[5] = { SPI_NOR_CMD_RDHCR, 0x00, 0x00, 0x06, 0x00 };

	*val = 0;
	int rc = ods_xfer(fiu, op, sizeof(op), val, 1);
	shell_print(shell, "    [%02Xh] %-12s = 0x%02x (rc=%d)", SPI_NOR_CMD_RDHCR, "HCR@0x600",
		    *val, rc);
	return rc;
}

/* same register the production code uses as "ODS register" */
static int ods_read_ods_reg(const struct shell *shell, const struct device *fiu, bool is_hamsa,
			    uint8_t *val)
{
	if (is_hamsa) {
		return ods_read_reg(shell, fiu, SPI_NOR_CMD_RDCR, "CR(15h)", val);
	}
	return ods_read_hcr(shell, fiu, val);
}

static int ods_wait_wip(const struct shell *shell, const struct device *fiu)
{
	uint8_t op = SPI_NOR_CMD_RDSR;
	uint8_t sr = 0;
	int64_t start = k_uptime_get();
	int64_t elapsed = 0;
	int rc;

	do {
		rc = ods_xfer(fiu, &op, 1, &sr, 1);
		elapsed = k_uptime_get() - start;
		if (rc || !(sr & ODS_SR_WIP)) {
			break;
		}
		k_msleep(1);
	} while (elapsed < ODS_WIP_POLL_TIMEOUT_MS);

	shell_print(shell, "    WIP poll: SR=0x%02x WIP=%d WEL=%d after %d ms (rc=%d)%s", sr,
		    !!(sr & ODS_SR_WIP), !!(sr & ODS_SR_WEL), (int)elapsed, rc,
		    (sr & ODS_SR_WIP) ? " TIMEOUT" : "");
	return rc;
}

static void ods_dump_regs(const struct shell *shell, const struct device *fiu, bool is_hamsa,
			  uint8_t mfr)
{
	uint8_t v = 0;

	ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(05h)", &v);
	shell_print(shell, "        SRWD/b7=%d QE/b6=%d BP=0x%x WEL=%d WIP=%d", !!(v & BIT(7)),
		    !!(v & BIT(6)), (v >> 2) & 0xF, !!(v & BIT(1)), !!(v & BIT(0)));

	if (mfr == ODS_JEDEC_MFR_MXIC) {
		ods_read_reg(shell, fiu, SPI_NOR_CMD_RDCR, "MX CR", &v);
		shell_print(shell, "        DC=%d 4BYTE=%d PBE=%d TB(OTP)=%d ODS=0x%x", (v >> 6) & 0x3,
			    !!(v & BIT(5)), !!(v & BIT(4)), !!(v & BIT(3)), v & ASIC_ODS_MASK);
		ods_read_reg(shell, fiu, ODS_CMD_RDSCUR_MXIC, "MX SCUR", &v);
		shell_print(shell, "        WPSEL=%d LDSO=%d", !!(v & BIT(7)), !!(v & BIT(1)));
	} else if (mfr == ODS_JEDEC_MFR_WINBOND) {
		ods_read_reg(shell, fiu, ODS_CMD_RDSR2_WINBOND, "WB SR2", &v);
		shell_print(shell, "        CMP=%d LB3=%d LB2=%d LB1=%d QE=%d SRL=%d", !!(v & BIT(6)),
			    !!(v & BIT(5)), !!(v & BIT(4)), !!(v & BIT(3)), !!(v & BIT(1)),
			    !!(v & BIT(0)));
		ods_read_reg(shell, fiu, SPI_NOR_CMD_RDCR, "WB SR3", &v);
		shell_print(shell, "        DRV=%d WPS=%d ADP=%d ADS=%d", (v >> 5) & 0x3,
			    !!(v & BIT(2)), !!(v & BIT(1)), !!(v & BIT(0)));
	} else if (!is_hamsa) {
		/* unknown vendor on NUWA: read the same HCR production code uses */
		ods_read_hcr(shell, fiu, &v);
		shell_print(shell, "        ODS(b[2:0])=0x%x", v & ASIC_ODS_MASK);
	} else {
		shell_warn(shell, "    unknown vendor on HAMSA, skip vendor-specific regs");
	}
}

/* restore Winbond SR2 to factory-like value: CMP=0, QE=1, SRL=0, keep OTP LB bits */
static void ods_wbfix(const struct shell *shell, const struct device *fiu, uint8_t mfr)
{
	uint8_t sr1 = 0, sr2 = 0, tmp = 0;

	if (mfr != ODS_JEDEC_MFR_WINBOND) {
		shell_warn(shell, "  not Winbond (mfr=0x%02x), skip wbfix", mfr);
		return;
	}
	if (ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR1", &sr1) ||
	    ods_read_reg(shell, fiu, ODS_CMD_RDSR2_WINBOND, "SR2", &sr2)) {
		shell_error(shell, "  read SR1/SR2 failed, skip wbfix");
		return;
	}

	uint8_t want = (sr2 & ODS_WB_SR2_LB_MASK) | ODS_WB_SR2_QE;
	shell_print(shell, "  -- wbfix: SR2 0x%02x -> 0x%02x --", sr2, want);
	if (sr2 == want) {
		shell_print(shell, "  SR2 already OK, nothing to do");
		return;
	}
	if (sr2 & ODS_WB_SR2_SRL) {
		shell_warn(shell, "  SRL=1: SR locked until flash power cycle, do AC/DC cycle first");
		return;
	}
	if (sr1 & ODS_WB_SR1_SRP) {
		shell_warn(shell, "  SRP=1: SR protected (/WP), skip wbfix");
		return;
	}

	uint8_t op = SPI_NOR_CMD_WREN;
	int rc = ods_xfer(fiu, &op, 1, NULL, 0);
	shell_print(shell, "    [06h] WREN rc=%d", rc);
	ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(WEL?)", &tmp);
	if (rc) {
		return;
	}

	uint8_t wr[2] = { ODS_CMD_WRSR2_WINBOND, want };
	rc = ods_xfer(fiu, wr, sizeof(wr), NULL, 0);
	shell_print(shell, "    [31h] WRSR2 %02x rc=%d", want, rc);
	if (rc) {
		return;
	}

	ods_wait_wip(shell, fiu);
	ods_read_reg(shell, fiu, ODS_CMD_RDSR2_WINBOND, "SR2", &sr2);
	shell_print(shell, "  wbfix result: SR2=0x%02x %s", sr2, (sr2 == want) ? "OK" : "FAIL");
}

static int ods_read4b(const struct device *fiu, uint32_t addr, uint8_t *buf, size_t len)
{
	uint8_t tx[5] = { ODS_CMD_READ4B, addr >> 24, addr >> 16, addr >> 8, addr };
	return ods_xfer(fiu, tx, sizeof(tx), buf, len);
}

/* return 0 if whole sector is FF, 1 if not, <0 on error */
static int ods_sector_blank(const struct shell *shell, const struct device *fiu, uint32_t addr)
{
	uint8_t buf[ODS_WPTEST_CHUNK];

	for (uint32_t off = 0; off < ODS_SECTOR_SIZE; off += sizeof(buf)) {
		int rc = ods_read4b(fiu, addr + off, buf, sizeof(buf));
		if (rc) {
			return rc;
		}
		for (size_t i = 0; i < sizeof(buf); i++) {
			if (buf[i] != 0xFF) {
				shell_print(shell, "    non-FF at 0x%08x = 0x%02x", addr + off + i,
					    buf[i]);
				return 1;
			}
		}
	}
	return 0;
}

static int ods_wren(const struct shell *shell, const struct device *fiu)
{
	uint8_t op = SPI_NOR_CMD_WREN, sr = 0;
	int rc = ods_xfer(fiu, &op, 1, NULL, 0);
	shell_print(shell, "    [06h] WREN rc=%d", rc);
	ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(WEL?)", &sr);
	return rc;
}

static void ods_wptest(const struct shell *shell, const struct device *fiu, uint8_t mfr)
{
	uint32_t addr = ods_wptest_addr;
	uint8_t b = 0;
	int rc;

	if (mfr != ODS_JEDEC_MFR_WINBOND && mfr != ODS_JEDEC_MFR_MXIC) {
		shell_warn(shell, "  unknown vendor 0x%02x, skip wptest", mfr);
		return;
	}

	shell_print(shell, "  -- wptest @0x%08x (4KB sector) --", addr);
	rc = ods_sector_blank(shell, fiu, addr);
	if (rc) {
		shell_warn(shell, "  sector is not blank (rc=%d), abort to keep ASIC image safe",
			   rc);
		return;
	}
	shell_print(shell, "  sector blank check: all FF");

	/* 1. page program one byte 0x00 */
	if (ods_wren(shell, fiu)) {
		return;
	}
	uint8_t pp[6] = { ODS_CMD_PP4B, addr >> 24, addr >> 16, addr >> 8, addr, 0x00 };
	rc = ods_xfer(fiu, pp, sizeof(pp), NULL, 0);
	shell_print(shell, "    [12h] PP4B @0x%08x data=00 rc=%d", addr, rc);
	ods_wait_wip(shell, fiu);
	ods_read4b(fiu, addr, &b, 1);
	bool pp_blocked = (b == 0xFF);
	shell_print(shell, "    [13h] read @0x%08x = 0x%02x -> program %s", addr, b,
		    pp_blocked ? "BLOCKED (read-only)" : "WENT THROUGH (writable)");

	/* 2. sector erase: blocked if protected, otherwise restores FF */
	if (ods_wren(shell, fiu)) {
		return;
	}
	uint8_t se[5] = { ODS_CMD_SE4B, addr >> 24, addr >> 16, addr >> 8, addr };
	rc = ods_xfer(fiu, se, sizeof(se), NULL, 0);
	shell_print(shell, "    [21h] SE4B @0x%08x rc=%d", addr, rc);
	ods_wait_wip(shell, fiu);
	rc = ods_sector_blank(shell, fiu, addr);
	shell_print(shell, "    sector after erase: %s", rc == 0 ? "all FF" : "NOT blank");

	shell_print(shell, "  wptest result: %s",
		    pp_blocked ? "array is WRITE PROTECTED" : "array is WRITABLE");
}

static bool ods_select(const struct shell *shell, const ods_chip_t *c)
{
	gpio_set(c->mux_gpio, 1);
	gpio_set(QSPI_CPLD_SEL_0, c->sel0);
	gpio_set(QSPI_CPLD_SEL_1, c->sel1);

	bool ok = set_cpld_bit(CPLD_OFFSET_ASIC_RESET, c->por_bit, 0) &&
		  set_cpld_bit(CPLD_OFFSET_ASIC_RESET, c->sys_bit, 0);
	uint8_t rst = 0;
	plat_read_cpld(CPLD_OFFSET_ASIC_RESET, &rst, 1);

	shell_print(shell, "  select: MUX_IN1=1 SEL0=%d SEL1=%d, hold POR(b%d)/SYS(b%d) -> %s, "
			   "CPLD reset reg=0x%02x",
		    c->sel0, c->sel1, c->por_bit, c->sys_bit, ok ? "ok" : "FAIL", rst);
	return ok;
}

static void ods_one_chip(const struct shell *shell, const ods_chip_t *c, enum ods_mode mode,
			 bool force)
{
	bool is_hamsa = (c->chip == ASIC_SPI_HAMSA);

	shell_print(shell, "===== %s (chip %d) =====", c->name, c->chip);

	if (!ods_select(shell, c)) {
		return;
	}

	const struct device *flash_dev = device_get_binding(ASIC_SPI_FLASH_DEV_NAME);
	const struct device *fiu = device_get_binding(SPI_FIU0_DEV_NAME);
	if (!flash_dev || !fiu) {
		shell_error(shell, "  device not found: flash=%p fiu=%p", flash_dev, fiu);
		return;
	}

	int rc = spi_nor_re_init(flash_dev);
	shell_print(shell, "  spi_nor_re_init rc=%d", rc);
	if (rc) {
		return;
	}

	uint8_t op = ODS_CMD_RDID;
	uint8_t id[3] = { 0 };
	rc = ods_xfer(fiu, &op, 1, id, sizeof(id));
	shell_print(shell, "  JEDEC ID: %02x %02x %02x (rc=%d) %s", id[0], id[1], id[2], rc,
		    (id[0] == ODS_JEDEC_MFR_MXIC)    ? "[Macronix]" :
		    (id[0] == ODS_JEDEC_MFR_WINBOND) ? "[Winbond]" :
						       "[other]");

	shell_print(shell, "  -- registers (before) --");
	ods_dump_regs(shell, fiu, is_hamsa, id[0]);

	if (mode == ODS_MODE_WBFIX) {
		ods_wbfix(shell, fiu, id[0]);
		shell_print(shell, "  -- registers (after) --");
		ods_dump_regs(shell, fiu, is_hamsa, id[0]);
		return;
	}
	if (mode == ODS_MODE_WPTEST) {
		ods_wptest(shell, fiu, id[0]);
		shell_print(shell, "  -- registers (after) --");
		ods_dump_regs(shell, fiu, is_hamsa, id[0]);
		return;
	}
	if (mode != ODS_MODE_RUN) {
		return;
	}

	/* guard: production opcodes are only valid for the expected vendor */
	if (!force) {
		if (is_hamsa && id[0] != ODS_JEDEC_MFR_MXIC) {
			shell_warn(shell, "  HAMSA flash is not Macronix, skip write "
					  "(add 'force' to override, may corrupt status regs!)");
			return;
		}
		if (!is_hamsa &&
		    (id[0] == ODS_JEDEC_MFR_MXIC || id[0] == ODS_JEDEC_MFR_WINBOND)) {
			shell_warn(shell, "  NUWA flash is MXIC/Winbond, 71h/72h undefined, skip "
					  "write (add 'force' to override)");
			return;
		}
	}

	/* ---- replay production sequence ---- */
	shell_print(shell, "  -- replay plat_change_flash_ods() --");
	uint8_t code = is_hamsa ? HAMSA_ODS_CODE : NUWA_ODS_CODE;
	uint8_t reg = 0;
	if (ods_read_ods_reg(shell, fiu, is_hamsa, &reg)) {
		shell_error(shell, "  read ODS reg failed");
		return;
	}
	uint8_t new_val = (reg & ~ASIC_ODS_MASK) | code;
	shell_print(shell, "  reg=0x%02x code=0x%x -> new_val=0x%02x", reg, code, new_val);

	uint8_t sr = 0;
	uint8_t tmp = 0;
	if (is_hamsa) {
		rc = ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(05h)", &sr);
		if (rc == 0) {
			op = SPI_NOR_CMD_WREN;
			rc = ods_xfer(fiu, &op, 1, NULL, 0);
			shell_print(shell, "    [06h] WREN rc=%d", rc);
			ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(WEL?)", &tmp);
		}
		if (rc == 0) {
			uint8_t wrsr[3] = { SPI_NOR_CMD_WRSR, sr, new_val };
			rc = ods_xfer(fiu, wrsr, sizeof(wrsr), NULL, 0);
			shell_print(shell, "    [01h] WRSR %02x %02x rc=%d", sr, new_val, rc);
		}
	} else {
		op = SPI_NOR_CMD_WREN;
		rc = ods_xfer(fiu, &op, 1, NULL, 0);
		shell_print(shell, "    [06h] WREN rc=%d", rc);
		ods_read_reg(shell, fiu, SPI_NOR_CMD_RDSR, "SR(WEL?)", &tmp);
		if (rc == 0) {
			uint8_t wrhcr[6] = { SPI_NOR_CMD_WRHCR, 0x00, 0x00, 0x06, 0x00, new_val };
			rc = ods_xfer(fiu, wrhcr, sizeof(wrhcr), NULL, 0);
			shell_print(shell, "    [72h] WRHCR @0x600 %02x rc=%d", new_val, rc);
		}
	}
	if (rc) {
		shell_error(shell, "  write failed rc=%d", rc);
		return;
	}

	/* production verify: immediately, no WIP wait */
	uint8_t verify = 0;
	ods_read_ods_reg(shell, fiu, is_hamsa, &verify);
	shell_print(shell, "  verify(immediate, like production): 0x%02x %s", verify,
		    (verify == new_val) ? "MATCH" : "MISMATCH");

	/* extra: wait WIP and verify again */
	ods_wait_wip(shell, fiu);
	ods_read_ods_reg(shell, fiu, is_hamsa, &verify);
	shell_print(shell, "  verify(after WIP): 0x%02x %s", verify,
		    (verify == new_val) ? "MATCH" : "MISMATCH");

	shell_print(shell, "  -- registers (after) --");
	ods_dump_regs(shell, fiu, is_hamsa, id[0]);
}

static int ods_cmd_common(const struct shell *shell, size_t argc, char **argv,
			  enum ods_mode mode)
{
	const char *target = argv[1];
	bool force = (argc >= 3 && !strcmp(argv[2], "force"));
	bool matched = false;

	if (!gpio_get(RST_ARKE_PWR_ON_PLD_R1_N)) {
		shell_warn(shell, "DC is off, do DC on first");
		return -1;
	}

	uint8_t board_id = get_asic_board_id();
	uint8_t vr = get_vr_module();
	shell_print(shell, "board_id=%d vr_module=%d vr_vendor=0x%02x -> production ODS %s",
		    board_id, vr, get_vr_vendor_module(),
		    (board_id == ASIC_BOARD_ID_ELECTRA && vr == VR_MODULE_RNS) ? "RUNS" :
										 "SKIPPED");

	uint8_t saved_rst = 0;
	if (!plat_read_cpld(CPLD_OFFSET_ASIC_RESET, &saved_rst, 1)) {
		shell_error(shell, "read CPLD reset reg failed, abort");
		return -1;
	}
	int saved_mux[3] = { gpio_get(SPI_HAMSA_MUX_IN1), gpio_get(SPI_NUWA0_MUX_IN1),
			     gpio_get(SPI_NUWA1_MUX_IN1) };
	int saved_sel0 = gpio_get(QSPI_CPLD_SEL_0);
	int saved_sel1 = gpio_get(QSPI_CPLD_SEL_1);
	shell_print(shell,
		    "saved: CPLD reset=0x%02x MUX(hamsa,nuwa0,nuwa1)=%d,%d,%d SEL0=%d SEL1=%d",
		    saved_rst, saved_mux[0], saved_mux[1], saved_mux[2], saved_sel0, saved_sel1);

	for (size_t i = 0; i < ARRAY_SIZE(ods_chips); i++) {
		if (strcmp(target, "all") && strcmp(target, ods_chips[i].name)) {
			continue;
		}
		matched = true;
		ods_one_chip(shell, &ods_chips[i], mode, force);
		/* release this chip's mux before selecting the next one */
		gpio_set(ods_chips[i].mux_gpio, 0);
	}

	/* restore (production forces all 0; restore original values instead) */
	gpio_set(SPI_HAMSA_MUX_IN1, saved_mux[0]);
	gpio_set(SPI_NUWA0_MUX_IN1, saved_mux[1]);
	gpio_set(SPI_NUWA1_MUX_IN1, saved_mux[2]);
	gpio_set(QSPI_CPLD_SEL_0, saved_sel0);
	gpio_set(QSPI_CPLD_SEL_1, saved_sel1);
	bool ok = plat_write_cpld(CPLD_OFFSET_ASIC_RESET, &saved_rst);
	uint8_t rst = 0;
	plat_read_cpld(CPLD_OFFSET_ASIC_RESET, &rst, 1);
	shell_print(shell, "restore: CPLD reset write %s, now=0x%02x", ok ? "ok" : "FAIL", rst);

	if (!matched) {
		shell_warn(shell, "unknown target '%s', use all|hamsa|nuwa0|nuwa1", target);
		return -1;
	}
	return 0;
}

static int cmd_ods_info(const struct shell *shell, size_t argc, char **argv)
{
	return ods_cmd_common(shell, argc, argv, ODS_MODE_INFO);
}

static int cmd_ods_run(const struct shell *shell, size_t argc, char **argv)
{
	return ods_cmd_common(shell, argc, argv, ODS_MODE_RUN);
}

static int cmd_ods_wptest(const struct shell *shell, size_t argc, char **argv)
{
	ods_wptest_addr = ODS_WPTEST_DEF_ADDR;
	if (argc >= 3) {
		char *end = NULL;
		unsigned long a = strtoul(argv[2], &end, 16);
		if (!end || *end || (a % ODS_SECTOR_SIZE) || a >= 0x02000000) {
			shell_error(shell, "addr must be 4KB aligned hex < 0x2000000");
			return -1;
		}
		ods_wptest_addr = a;
	}
	argc = 2; /* ods_cmd_common: argv[2] is not 'force' */
	return ods_cmd_common(shell, argc, argv, ODS_MODE_WPTEST);
}

static int cmd_ods_wbfix(const struct shell *shell, size_t argc, char **argv)
{
	return ods_cmd_common(shell, argc, argv, ODS_MODE_WBFIX);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_asic_ods_cmds,
	SHELL_CMD_ARG(info, NULL, "info <all|hamsa|nuwa0|nuwa1>: read-only dump of flash regs",
		      cmd_ods_info, 2, 0),
	SHELL_CMD_ARG(run, NULL,
		      "run <all|hamsa|nuwa0|nuwa1> [force]: replay ODS workaround with logs",
		      cmd_ods_run, 2, 1),
	SHELL_CMD_ARG(wbfix, NULL,
		      "wbfix <all|hamsa|nuwa0|nuwa1>: restore Winbond SR2 (CMP=0 QE=1 SRL=0) "
		      "after flash power cycle",
		      cmd_ods_wbfix, 2, 0),
	SHELL_CMD_ARG(wptest, NULL,
		      "wptest <hamsa|nuwa0|nuwa1> [sector_addr_hex]: program/erase test on an "
		      "all-FF 4KB sector (default 0x1FFF000)",
		      cmd_ods_wptest, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(asic_ods, &sub_asic_ods_cmds,
		   "ASIC flash ODS workaround debug (holds ASIC in reset while running)", NULL);
