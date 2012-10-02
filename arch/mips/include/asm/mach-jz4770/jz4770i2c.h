#ifndef __JZ4770I2C_H__
#define __JZ4770I2C_H__

struct i2c_jz4770_platform_data {
	bool	use_dma;
};

#define JZ_I2C_NUM	3

#define	I2C_BASE(n)	(0xB0050000 | !!(n) << 12 | ((n) & 2) << 13)

#endif /* __JZ4770I2C_H__ */
