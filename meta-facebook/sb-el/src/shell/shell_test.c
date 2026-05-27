#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_adc.h"
#include "plat_class.h"

#define I2C_DEVICE_PREFIX "I2C_"
#define MAX_I2C_BYTES 31

static int8_t name2idx(const char *name)
{
	if (name == NULL)
		return -1;

	if (strncmp(name, I2C_DEVICE_PREFIX, strlen(I2C_DEVICE_PREFIX)) != 0)
		return -1;

	return strtol(name + strlen(I2C_DEVICE_PREFIX), NULL, 10);
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, I2C_DEVICE_PREFIX);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	// test code
	shell_warn(shell, "Hello!");
}

void cmd_read_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t len = strtoul(argv[3], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if ((sensor_id == 0) || (sensor_id >= SENSOR_NUM_NUMBERS)) {
		if (!plat_read_cpld(offset, data, 1)) {
			shell_warn(shell, "cpld read 0x%02x fail", offset);
			return;
		}
	} else {
		if (!get_raw_data_from_sensor_id(sensor_id, offset, data, len)) {
			shell_warn(shell, "sensor_id 0x%02x read 0x%02x fail", sensor_id, offset);
			return;
		}
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

void cmd_read_info(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	if (cfg == NULL)
		return;

	shell_print(shell, "sensor_id 0x%02x bus: %d, addr: 0x%x(0x%x)", sensor_id, cfg->port,
		    cfg->target_addr, (cfg->target_addr >> 1));
}

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t len = strtoul(argv[2], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if (!plat_read_cpld(offset, data, len)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}
void cmd_cpld_write(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld write <offset> <data>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t data = strtoul(argv[2], NULL, 16);

	if (!plat_write_cpld(offset, &data)) {
		shell_warn(shell, "cpld write 0x%02x fail", offset);
		return;
	}

	shell_warn(shell, "cpld write %02x to offset %02x", data, offset);
}

void cmd_info(const struct shell *shell, size_t argc, char **argv)
{
	static const char *const vr_module_str[] = {
		[VR_MODULE_MPS] = "MPS",
		[VR_MODULE_RNS] = "RNS",
	};

	static const char *const ubc_module_str[] = {
		[UBC_MODULE_DELTA] = "DELTA",
		[UBC_MODULE_MPS] = "MPS",
		[UBC_MODULE_FLEX] = "FLEX",
		[UBC_MODULE_LUXSHARE] = "LUXSHARE",
	};

	static const char *const asic_board_id_str[] = {
		[ASIC_BOARD_ID_RSVD1] = "RSVD1",
		[ASIC_BOARD_ID_RSVD2] = "RSVD2",
		[ASIC_BOARD_ID_ELECTRA] = "ELECTRA",
		[ASIC_BOARD_ID_EVB] = "EVB",
	};

	uint8_t vr = get_vr_module();
	uint8_t ubc = get_ubc_module();
	uint8_t board_id = get_asic_board_id();
	uint8_t board_rev = get_board_rev_id();
	uint8_t adc_idx = get_adc_type();
	uint8_t tray_loc = get_tray_location();

	shell_warn(shell, "vr module: %s",
		   (vr < VR_MODULE_UNKNOWN) ? vr_module_str[vr] : "UNKNOWN");
	shell_warn(shell, "ubc module: %s",
		   (ubc < UBC_MODULE_UNKNOWN) ? ubc_module_str[ubc] : "UNKNOWN");
	shell_warn(shell, "mmc slot: %d", get_mmc_slot() + 1);
	shell_warn(shell, "asic board id: %s",
		   (board_id < ASIC_BOARD_ID_UNKNOWN) ? asic_board_id_str[board_id] : "UNKNOWN");
	shell_warn(shell, "asic board rev id: %d", board_rev);
	shell_warn(shell, "adc idx: %d (0:ADI, 1:TI)", adc_idx);
	shell_warn(shell, "tray location: %d", tray_loc);
}

void cmd_test_write(const struct shell *shell, size_t argc, char **argv)
{
	/*
	 * Usage:
	 * test write <bus> <devaddr> <write_byte1> [<write_byte2> ...]
	 *
	 * Example:
	 * test write I2C_2 0x08 0x02 0x40 0x2C 0x30
	 *   -> pure write 0x02 0x40 0x2C 0x30
	 */
	if (argc < 4) {
		shell_warn(shell,
			   "Help: test write <bus> <devaddr> <write_byte1> [<write_byte2> ...]");
		return;
	}

	I2C_MSG msg = { 0 };

	msg.bus = name2idx(argv[1]);
	if (msg.bus == 0xff) {
		shell_error(shell, "Invalid bus name: %s", argv[1]);
		return;
	}

	msg.target_addr = strtol(argv[2], NULL, 16);
	msg.rx_len = 0;
	msg.tx_len = argc - 3;

	if (msg.tx_len > I2C_BUFF_SIZE) {
		shell_error(shell, "write bytes exceed max buffer size");
		return;
	}

	for (int i = 0; i < msg.tx_len; i++) {
		msg.data[i] = strtol(argv[3 + i], NULL, 16);
	}

	if (i2c_master_write(&msg, 5)) {
		shell_error(shell, "Failed to write to bus %d device: 0x%x", msg.bus,
			    msg.target_addr);
		return;
	}

	shell_print(shell, "Write success");
	shell_hexdump(shell, msg.data, msg.tx_len);
	shell_print(shell, "");
}

void cmd_test_write_read(const struct shell *shell, size_t argc, char **argv)
{
	/*
	 * Usage:
	 * test write_read <bus> <devaddr> <read_bytes> <write_byte1> [<write_byte2> ...]
	 *
	 * Example:
	 * test write_read I2C_2 0x09 4 0x00 0xA8
	 *   -> write 0x00 0xA8, repeated-start, read 4 bytes
	 */
	if (argc < 5) {
		shell_warn(shell,
			   "Help: test write_read <bus> <devaddr> <read_bytes> <write_byte1> [<write_byte2> ...]");
		return;
	}

	I2C_MSG msg = { 0 };

	msg.bus = name2idx(argv[1]);
	if (msg.bus == 0xff) {
		shell_error(shell, "Invalid bus name: %s", argv[1]);
		return;
	}

	msg.target_addr = strtol(argv[2], NULL, 16);
	msg.rx_len = strtol(argv[3], NULL, 16);
	msg.tx_len = argc - 4;

	if (msg.rx_len == 0) {
		shell_error(shell, "read_bytes should not be 0, use 'test write' instead");
		return;
	}

	if (msg.tx_len == 0) {
		shell_error(shell, "write bytes should not be empty");
		return;
	}

	if (msg.tx_len > I2C_BUFF_SIZE) {
		shell_error(shell, "write bytes exceed max buffer size");
		return;
	}

	for (int i = 0; i < msg.tx_len; i++) {
		msg.data[i] = strtol(argv[4 + i], NULL, 16);
	}

	if (i2c_master_read(&msg, 5)) {
		shell_error(shell, "Failed to write-read from bus %d device: 0x%x", msg.bus,
			    msg.target_addr);
		return;
	}

	shell_hexdump(shell, msg.data, msg.rx_len);
	shell_print(shell, "");
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			SHELL_CMD(write, NULL, "write cpld register", cmd_cpld_write),
			SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
			SHELL_CMD(read_raw, NULL, "read raw data test command",cmd_read_raw),
			SHELL_CMD(read_info, NULL, "read sensor info test command", cmd_read_info),
			SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
			SHELL_CMD(info, NULL, "info commands", cmd_info),
			SHELL_CMD_ARG(write_read, &dsub_device_name,
		      "test write_read <bus> <devaddr> <read_bytes> <write_byte1> [<write_byte2> ...]",
		      cmd_test_write_read, 5, MAX_I2C_BYTES),
			SHELL_CMD_ARG(write, &dsub_device_name,
		      "test write <bus> <devaddr> <write_byte1> [<write_byte2> ...]",
		      cmd_test_write, 4, MAX_I2C_BYTES),
			SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);