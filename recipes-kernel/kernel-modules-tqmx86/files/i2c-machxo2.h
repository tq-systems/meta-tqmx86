/*
 * i2c-machxo2.h - definitions for the i2c-machxo2 interface
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _LINUX_I2C_MACHXO2_H
#define _LINUX_I2C_MACHXO2_H

struct machxo2_i2c_platform_data {
	u32 clock_khz; /* input clock in kHz */
	u32 speed;
	u8 num_devices; /* number of devices in the devices list */
	struct i2c_board_info const *devices; /* devices connected to the bus */
};

#endif /* _LINUX_I2C_MACHXO2_H */
