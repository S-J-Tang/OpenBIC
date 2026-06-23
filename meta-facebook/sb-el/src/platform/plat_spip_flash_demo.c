#include <zephyr.h>
#include <shell/shell.h>
#include <drivers/flash.h>
#include <string.h>

#define SPIP_FLASH_DEV_NAME "spi_spip1_cs0"
#define TEST_ADDR           0x00000000U
#define ERASE_SZ            4096
#define WRITE_SZ            16
#define READ_SZ             32
#define WR_TEST_READ_SZ     64

static const char test_str[] = "SPIP write test";

static void print_hex_dump(const struct shell *sh, uint32_t base_addr,
			    const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i += 16) {
		size_t row_len = MIN(16, len - i);

		shell_fprintf(sh, SHELL_NORMAL, "%08x:", base_addr + i);

		for (size_t j = 0; j < 16; j++) {
			if (j == 8)
				shell_fprintf(sh, SHELL_NORMAL, " ");
			if (j < row_len)
				shell_fprintf(sh, SHELL_NORMAL, " %02x", buf[i + j]);
			else
				shell_fprintf(sh, SHELL_NORMAL, "   ");
		}

		shell_fprintf(sh, SHELL_NORMAL, " |");
		for (size_t j = 0; j < 16; j++) {
			if (j == 8)
				shell_fprintf(sh, SHELL_NORMAL, " ");
			if (j < row_len) {
				uint8_t c = buf[i + j];
				shell_fprintf(sh, SHELL_NORMAL, "%c",
					      (c >= 0x20 && c < 0x7f) ? c : '.');
			} else {
				shell_fprintf(sh, SHELL_NORMAL, " ");
			}
		}
		shell_fprintf(sh, SHELL_NORMAL, "|\n");
	}
}

static int cmd_write_demo(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *flash_dev;
	uint8_t write_buf[WRITE_SZ];
	uint8_t read_back_buf[WR_TEST_READ_SZ];
	int ret;

	flash_dev = device_get_binding(SPIP_FLASH_DEV_NAME);
	if (!flash_dev) {
		shell_error(sh, "Cannot find device: %s", SPIP_FLASH_DEV_NAME);
		return -ENODEV;
	}

	/* flash_erase(dev, offset, size): must erase before write, erase unit is block (4KB) */
	shell_print(sh, "Erasing %d bytes at addr 0x%08x...", ERASE_SZ, TEST_ADDR);
	ret = flash_erase(flash_dev, TEST_ADDR, ERASE_SZ);
	if (ret) {
		shell_error(sh, "Erase failed: %d", ret);
		return ret;
	}

	memset(write_buf, 0xff, WRITE_SZ);
	memcpy(write_buf, test_str, strlen(test_str));

	/* flash_write(dev, offset, data, size) */
	shell_print(sh, "Writing \"%s\" to addr 0x%08x...", test_str, TEST_ADDR);
	ret = flash_write(flash_dev, TEST_ADDR, write_buf, WRITE_SZ);
	if (ret) {
		shell_error(sh, "Write failed: %d", ret);
		return ret;
	}

	/* flash_read(dev, offset, buf, size): read does not require prior erase */
	shell_print(sh, "Read after write:");
	ret = flash_read(flash_dev, TEST_ADDR, read_back_buf, WR_TEST_READ_SZ);
	if (ret) {
		shell_error(sh, "Read failed: %d", ret);
		return ret;
	}
	print_hex_dump(sh, TEST_ADDR, read_back_buf, WR_TEST_READ_SZ);

	return 0;
}

static int cmd_erase(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *flash_dev;
	int ret;

	flash_dev = device_get_binding(SPIP_FLASH_DEV_NAME);
	if (!flash_dev) {
		shell_error(sh, "Cannot find device: %s", SPIP_FLASH_DEV_NAME);
		return -ENODEV;
	}

	shell_print(sh, "Erasing %d bytes at addr 0x%08x...", ERASE_SZ, TEST_ADDR);
	ret = flash_erase(flash_dev, TEST_ADDR, ERASE_SZ);
	if (ret) {
		shell_error(sh, "Erase failed: %d", ret);
		return ret;
	}
	shell_print(sh, "Erase done.");

	return 0;
}

static int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *flash_dev;
	uint8_t read_back_buf[READ_SZ];
	int ret;

	flash_dev = device_get_binding(SPIP_FLASH_DEV_NAME);
	if (!flash_dev) {
		shell_error(sh, "Cannot find device: %s", SPIP_FLASH_DEV_NAME);
		return -ENODEV;
	}

	ret = flash_read(flash_dev, TEST_ADDR, read_back_buf, READ_SZ);
	if (ret) {
		shell_error(sh, "Read failed: %d", ret);
		return ret;
	}
	print_hex_dump(sh, TEST_ADDR, read_back_buf, READ_SZ);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(spip_flash_cmds,
	SHELL_CMD_ARG(write_demo, NULL,
		      "Erase, write 'SPIP write test' to addr 0x0, read back 64 bytes",
		      cmd_write_demo, 1, 0),
	SHELL_CMD_ARG(erase, NULL,
		      "Erase 4KB at addr 0x0",
		      cmd_erase, 1, 0),
	SHELL_CMD_ARG(read, NULL,
		      "Read 32 bytes from addr 0x0",
		      cmd_read, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(spip_flash, &spip_flash_cmds, "SPIP flash demo commands", NULL);
