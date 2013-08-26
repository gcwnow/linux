#ifndef __PANEL_IT6610_H
#define __PANEL_IT6610_H

/* The PCADR pin selects the address from the following two options. */
#define IT6610_I2C_ADDR_LO 0x4C
#define IT6610_I2C_ADDR_HI 0x4D

/* Note: Interrupt is specified via struct i2c_board_info. */

struct it6610_i2c_platform_data {
	int gpio_reset;		/* SYSRSTN pin, -1 for none */
};

#endif /* __PANEL_IT6610_H */
