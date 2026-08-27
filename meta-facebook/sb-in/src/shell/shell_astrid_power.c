#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_i2c.h"
#include "plat_pldm_sensor.h"
#include "plat_class.h"
#include "plat_gpio.h"

static bool astrid_power_control(uint8_t onoff)
{
	uint8_t tmp = onoff ? 0x80 : 0x00;
	return plat_write_cpld(0x38, &tmp);
}
void cmd_astrid_power_on(const struct shell *shell, size_t argc, char **argv)
{
	if (!astrid_power_control(1))
		shell_warn(shell, "astrid power on set cpld fail!");
	// wait 1s
	k_msleep(1500);
	if (gpio_get(RST_ASTRID_PWR_ON_PLD_R1_N) == GPIO_HIGH) {
		shell_print(shell, "astrid power on success!");
	} else {
		shell_warn(shell, "astrid power on fail!");
	}
}
void cmd_astrid_power_off(const struct shell *shell, size_t argc, char **argv)
{
	if (!astrid_power_control(0))
		shell_warn(shell, "astrid power off set cpld fail!");
	// wait 1s
	k_msleep(1500);
	if (gpio_get(FM_PLD_UBC_EN_R) == GPIO_LOW) {
		shell_print(shell, "astrid power off success!");
	} else {
		shell_warn(shell, "astrid power off fail!");
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_astrid_power_cmd,
			       SHELL_CMD(on, NULL, "astrid power on", cmd_astrid_power_on),
				   SHELL_CMD(off, NULL, "astrid power off", cmd_astrid_power_off),
			       SHELL_SUBCMD_SET_END);

/* Root of command echo */
SHELL_CMD_REGISTER(astrid_power, &sub_astrid_power_cmd, "astrid power commands", NULL);
