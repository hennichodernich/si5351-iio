/*
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rational.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <asm/unaligned.h>
#include <asm/div64.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "si5351_defs.h"
#include "si5351-iio.h"

static int si5351_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		return IIO_VAL_INT;
	default:
		break;
	}
	return -EINVAL;
}

static int si5351_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t si5351_write_ext(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct si5351_state *st = iio_priv(indio_dev);
	struct i2c_client *i2c = to_i2c_client(st->dev);
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case SI5351_FREQ:
		st->freq_cache[chan->channel] = si5351_config_msynth(i2c, chan->channel, PLL_A, (unsigned int)readin, st->fVCO);
		si5351_ctrl_msynth(i2c, chan->channel, 1, SI5351_CLK_INPUT_MULTISYNTH_N, SI5351_CLK_DRIVE_STRENGTH_8MA, 0);
		ret = 0;
		break;
	case SI5351_PHASE:
		if (readin<180)
		{
			st->phase_cache[chan->channel] = si5351_config_msynth_phase(i2c, chan->channel, PLL_A, st->freq_cache[chan->channel], st->fVCO, (unsigned int)readin);
			si5351_ctrl_msynth(i2c, chan->channel, 1, SI5351_CLK_INPUT_MULTISYNTH_N, SI5351_CLK_DRIVE_STRENGTH_8MA, 0);
		}
		else
		{
			st->phase_cache[chan->channel] = si5351_config_msynth_phase(i2c, chan->channel, PLL_A, st->freq_cache[chan->channel], st->fVCO, (unsigned int)readin-180);
			st->phase_cache[chan->channel] += 180;
			si5351_ctrl_msynth(i2c, chan->channel, 1, SI5351_CLK_INPUT_MULTISYNTH_N, SI5351_CLK_DRIVE_STRENGTH_8MA, 1);
		}

		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t si5351_read_ext(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct si5351_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case SI5351_FREQ:
		val = st->freq_cache[chan->channel];
		break;
	case SI5351_PHASE:
		val = st->phase_cache[chan->channel];
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}



static const struct iio_info si5351_info = {
	.read_raw = si5351_read_raw,
	.write_raw = si5351_write_raw,
};

static const struct iio_chan_spec_ext_info si5351_ext_info[] = {
{ \
	.name = "frequency", \
	.read = si5351_read_ext, \
	.write = si5351_write_ext, \
	.private = SI5351_FREQ, \
	.shared = IIO_SEPARATE, \
},
{ \
	.name = "phase", \
	.read = si5351_read_ext, \
	.write = si5351_write_ext, \
	.private = SI5351_PHASE, \
	.shared = IIO_SEPARATE, \
},
//	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad5064_powerdown_mode_enum),
//	IIO_ENUM_AVAILABLE("powerdown_mode", &ad5064_powerdown_mode_enum),
	{ },
};

#define SI5351_CHANNEL(chan, addr, bits, _shift, _ext_info) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
	BIT(IIO_CHAN_INFO_SCALE),					\
	.address = addr,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (bits),				\
		.storagebits = 16,				\
		.shift = (_shift),				\
	},							\
	.ext_info = (_ext_info),				\
}

#define DECLARE_SI5351C_CHANNELS(name, bits, shift, ext_info) \
const struct iio_chan_spec name[] = { \
	SI5351_CHANNEL(0, 0, bits, shift, ext_info), \
	SI5351_CHANNEL(1, 1, bits, shift, ext_info), \
	SI5351_CHANNEL(2, 2, bits, shift, ext_info), \
	SI5351_CHANNEL(3, 3, bits, shift, ext_info), \
	SI5351_CHANNEL(4, 4, bits, shift, ext_info), \
	SI5351_CHANNEL(5, 5, bits, shift, ext_info), \
	SI5351_CHANNEL(6, 6, bits, shift, ext_info), \
	SI5351_CHANNEL(7, 7, bits, shift, ext_info), \
}

#define DECLARE_SI5351A_CHANNELS(name, bits, shift, ext_info) \
const struct iio_chan_spec name[] = { \
	SI5351_CHANNEL(0, 0, bits, shift, ext_info), \
	SI5351_CHANNEL(1, 1, bits, shift, ext_info), \
	SI5351_CHANNEL(2, 2, bits, shift, ext_info), \
}


static DECLARE_SI5351A_CHANNELS(si5351a_channels, 16, 0, si5351_ext_info);
static DECLARE_SI5351C_CHANNELS(si5351c_channels, 16, 0, si5351_ext_info);

static const struct si5351_chip_info si5351_chip_info_tbl[] = {
	[ID_SI5351A] = {
		.channels = si5351a_channels,
		.num_channels = 3,
	},
	[ID_SI5351C] = {
		.channels = si5351c_channels,
		.num_channels = 8,
	},
};

static void si5351_write_parameters(struct i2c_client *i2c,
				    unsigned int start_reg, struct si5351_multisynth_parameters *params)
{
	u8 buf[SI5351_PARAMETERS_LENGTH];

	switch (start_reg) {
	case SI5351_CLK6_PARAMETERS:
	case SI5351_CLK7_PARAMETERS:
		buf[0] = params->p1 & 0xff;
		i2c_smbus_write_byte_data(i2c, start_reg, buf[0]);
		break;
	default:
		buf[0] = ((params->p3 & 0x0ff00) >> 8) & 0xff;
		buf[1] = params->p3 & 0xff;
		/* save rdiv and divby4 */
		buf[2] = i2c_smbus_read_byte_data(i2c, start_reg + 2) & ~0x03;
		buf[2] |= ((params->p1 & 0x30000) >> 16) & 0x03;
		buf[3] = ((params->p1 & 0x0ff00) >> 8) & 0xff;
		buf[4] = params->p1 & 0xff;
		buf[5] = ((params->p3 & 0xf0000) >> 12) |
			((params->p2 & 0xf0000) >> 16);
		buf[6] = ((params->p2 & 0x0ff00) >> 8) & 0xff;
		buf[7] = params->p2 & 0xff;
		dev_dbg(&i2c->dev, "si5351a-iio: writing %02x %02x %02x %02x %02x %02x %02x %02x at reg %d\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],start_reg);
		i2c_smbus_write_i2c_block_data(i2c, start_reg, SI5351_PARAMETERS_LENGTH, buf);
	}
}

static int si5351_setup_pll(struct i2c_client *i2c, unsigned int pll, unsigned int fVCO, unsigned int fXTAL)
{
	struct si5351_multisynth_parameters params;
	unsigned long rfrac, denom, a, b, c;
		unsigned long long lltmp;
		int val;

		unsigned int start_reg = (pll == PLL_A) ? SI5351_PLLA_PARAMETERS : SI5351_PLLB_PARAMETERS;

		if (fVCO < SI5351_PLL_VCO_MIN)
			fVCO = SI5351_PLL_VCO_MIN;
		if (fVCO > SI5351_PLL_VCO_MAX)
			fVCO = SI5351_PLL_VCO_MAX;

		/* determine integer part of feedback equation */
		a = fVCO / fXTAL;

		if (a < SI5351_PLL_A_MIN)
			fVCO = fXTAL * SI5351_PLL_A_MIN;
		if (a > SI5351_PLL_A_MAX)
			fVCO = fXTAL * SI5351_PLL_A_MAX;

		/* find best approximation for b/c = fVCO mod fIN */
		denom = 1000 * 1000;
		lltmp = fVCO % fXTAL;
		lltmp *= denom;
		do_div(lltmp, fXTAL);
		rfrac = (unsigned long)lltmp;

		b = 0;
		c = 1;
		if (rfrac)
			rational_best_approximation(rfrac, denom,
					    SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);

		/* calculate parameters */
		params.p3  = c;
		params.p2  = (128 * b) % c;
		params.p1  = 128 * a;
		params.p1 += (128 * b / c);
		params.p1 -= 512;

		/* recalculate rate by fIN * (a + b/c) */
		lltmp  = fXTAL;
		lltmp *= b;
		do_div(lltmp, c);

		fVCO  = (unsigned long)lltmp;
		fVCO += fXTAL * a;

		dev_dbg(&i2c->dev, "si5351-iio: found a=%d, b=%d, c=%d\n", a, b, c);
		dev_dbg(&i2c->dev, "si5351-iio: found p1=%d, p2=%d, p3=%d\n", params.p1, params.p2, params.p3);

		si5351_write_parameters(i2c, start_reg, &params);
		/* plla/pllb ctrl is in clk6/clk7 ctrl registers */
		val = i2c_smbus_read_byte_data(i2c, SI5351_CLK6_CTRL + pll);
		if (params.p2 == 0)
			val |= SI5351_CLK_INTEGER_MODE;
		else
			val &= ~SI5351_CLK_INTEGER_MODE;
		i2c_smbus_write_byte_data(i2c, SI5351_CLK6_CTRL + pll, val);

			/* Do a pll soft reset on the affected pll */
		i2c_smbus_write_byte_data(i2c, SI5351_PLL_RESET,
					 (pll == PLL_A) ? SI5351_PLL_RESET_A :
							    SI5351_PLL_RESET_B);
		return fVCO;

}

static inline u8 si5351_msynth_params_address(int num)
{
	if (num > 5)
		return SI5351_CLK6_PARAMETERS + (num - 6);
	return SI5351_CLK0_PARAMETERS + (SI5351_PARAMETERS_LENGTH * num);
}

static unsigned int si5351_config_msynth(struct i2c_client *i2c, unsigned int output, unsigned int pll, unsigned int fout, unsigned int fVCO)
{
	struct si5351_multisynth_parameters params;
	unsigned long a, b, c;
	unsigned long long lltmp;
	int divby4;
	u8 start_reg;
	int val;

	/* multisync6-7 can only handle freqencies < 150MHz */
	if (output >= 6 && fout > SI5351_MULTISYNTH67_MAX_FREQ)
		fout = SI5351_MULTISYNTH67_MAX_FREQ;

	/* multisync frequency is 1MHz .. 160MHz */
	if (fout > SI5351_MULTISYNTH_MAX_FREQ)
		fout = SI5351_MULTISYNTH_MAX_FREQ;
	if (fout < SI5351_MULTISYNTH_MIN_FREQ)
		fout = SI5351_MULTISYNTH_MIN_FREQ;

	divby4 = 0;
	if (fout > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	if (output >= 6) {
		/* determine the closest integer divider */
		a = DIV_ROUND_CLOSEST(fVCO, fout);
		if (a < SI5351_MULTISYNTH_A_MIN)
			a = SI5351_MULTISYNTH_A_MIN;
		if (a > SI5351_MULTISYNTH67_A_MAX)
			a = SI5351_MULTISYNTH67_A_MAX;

		b = 0;
		c = 1;
	} else {
		unsigned long rfrac, denom;

		/* disable divby4 */
		if (divby4) {
			fout = SI5351_MULTISYNTH_DIVBY4_FREQ;
			divby4 = 0;
		}

		/* determine integer part of divider equation */
		a = fVCO / fout;
		if (a < SI5351_MULTISYNTH_A_MIN)
			a = SI5351_MULTISYNTH_A_MIN;
		if (a > SI5351_MULTISYNTH_A_MAX)
			a = SI5351_MULTISYNTH_A_MAX;

		/* find best approximation for b/c = fVCO mod fOUT */
		denom = 1000 * 1000;
		lltmp = (fVCO) % fout;
		lltmp *= denom;
		do_div(lltmp, fout);
		rfrac = (unsigned long)lltmp;

		b = 0;
		c = 1;
		if (rfrac)
			rational_best_approximation(rfrac, denom,
			    SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX,
			    &b, &c);
	}

	if (b==0)
		params.intmode=1;
	else
		params.intmode=0;

	/* recalculate fout by fOUT = fIN / (a + b/c) */
	lltmp  = fVCO;
	lltmp *= c;
	do_div(lltmp, a * c + b);
	fout  = (unsigned long)lltmp;

	/* calculate parameters */
	if (divby4) {
		params.p3 = 1;
		params.p2 = 0;
		params.p1 = 0;
	} else if (output >= 6) {
		params.p3 = 0;
		params.p2 = 0;
		params.p1 = a;
	} else {
		params.p3  = c;
		params.p2  = (128 * b) % c;
		params.p1  = 128 * a;
		params.p1 += (128 * b / c);
		params.p1 -= 512;
	}

	dev_dbg(&i2c->dev, "si5351-iio: using fVCO=%d\n", fVCO);
	dev_dbg(&i2c->dev, "si5351-iio: found a=%d, b=%d, c=%d\n", a, b, c);
	dev_dbg(&i2c->dev, "si5351-iio: found p1=%d, p2=%d, p3=%d, divby4=%d\n", params.p1, params.p2, params.p3, divby4);
	dev_dbg(&i2c->dev, "si5351-iio: fout=%d\n", fout);

	start_reg = si5351_msynth_params_address(output);
	/* write multisynth parameters */
	si5351_write_parameters(i2c, start_reg, &params);

	if (fout > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* enable/disable integer mode and divby4 on multisynth0-5 */
	if (output < 6)
	{
		val = i2c_smbus_read_byte_data(i2c, start_reg + 2);
		if (divby4)
			val |= SI5351_OUTPUT_CLK_DIVBY4;
		else
			val &= ~SI5351_OUTPUT_CLK_DIVBY4;
		i2c_smbus_write_byte_data(i2c, start_reg + 2, val);

		val = i2c_smbus_read_byte_data(i2c, SI5351_CLK0_CTRL + output);
		if (params.intmode == 1)
			val |= SI5351_CLK_INTEGER_MODE;
		else
			val &= ~SI5351_CLK_INTEGER_MODE;
		i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + output, val);
	}

	val = i2c_smbus_read_byte_data(i2c, SI5351_CLK0_CTRL + output);
	if (pll == PLL_B)
		val |= SI5351_CLK_PLL_SELECT;
	else
		val &= ~SI5351_CLK_PLL_SELECT;
	i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + output, val);

	dev_dbg(&i2c->dev, "si5351-iio: wrote CTRL byte %02x\n", val);

	return fout;
}

static unsigned int si5351_config_msynth_phase(struct i2c_client *i2c, unsigned int output, unsigned int pll, unsigned int fout, unsigned int fVCO, unsigned int degrees)
{
	struct si5351_multisynth_parameters params;
	unsigned long a, b, c, phase_val;
	unsigned long long lltmp;
	int divby4;
	u8 start_reg;
	int val;

	/* multisync6-7 can only handle freqencies < 150MHz */
	if (output >= 6 && fout > SI5351_MULTISYNTH67_MAX_FREQ)
		fout = SI5351_MULTISYNTH67_MAX_FREQ;

	/* multisync frequency is 1MHz .. 160MHz */
	if (fout > SI5351_MULTISYNTH_MAX_FREQ)
		fout = SI5351_MULTISYNTH_MAX_FREQ;
	if (fout < SI5351_MULTISYNTH_MIN_FREQ)
		fout = SI5351_MULTISYNTH_MIN_FREQ;

	divby4 = 0;
	if (fout > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	if (output >= 6) {
		/* determine the closest integer divider */
		a = DIV_ROUND_CLOSEST(fVCO, fout);
		if (a < SI5351_MULTISYNTH_A_MIN)
			a = SI5351_MULTISYNTH_A_MIN;
		if (a > SI5351_MULTISYNTH67_A_MAX)
			a = SI5351_MULTISYNTH67_A_MAX;

		b = 0;
		c = 1;
	} else {
		unsigned long rfrac, denom;

		/* disable divby4 */
		if (divby4) {
			fout = SI5351_MULTISYNTH_DIVBY4_FREQ;
			divby4 = 0;
		}

		/* determine integer part of divider equation */
		a = fVCO / fout;
		if (a < SI5351_MULTISYNTH_A_MIN)
			a = SI5351_MULTISYNTH_A_MIN;
		if (a > SI5351_MULTISYNTH_A_MAX)
			a = SI5351_MULTISYNTH_A_MAX;

		/* find best approximation for b/c = fVCO mod fOUT */
		denom = 1000 * 1000;
		lltmp = (fVCO) % fout;
		lltmp *= denom;
		do_div(lltmp, fout);
		rfrac = (unsigned long)lltmp;

		b = 0;
		c = 1;
		if (rfrac)
			rational_best_approximation(rfrac, denom,
			    SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX,
			    &b, &c);
	}

	/* recalculate fout by fOUT = fIN / (a + b/c) */
	lltmp  = fVCO;
	lltmp *= c;
	do_div(lltmp, a * c + b);
	fout  = (unsigned long)lltmp;

	/* calculate parameters */
	if (divby4) {
		params.p3 = 1;
		params.p2 = 0;
		params.p1 = 0;
	} else if (output >= 6) {
		params.p3 = 0;
		params.p2 = 0;
		params.p1 = a;
	} else {
		params.p3  = c;
		params.p2  = (128 * b) % c;
		params.p1  = 128 * a;
		params.p1 += (128 * b / c);
		params.p1 -= 512;
	}

	phase_val = (fVCO / fout) * degrees / 90;
	if (phase_val > 127)
	{
		phase_val = 127;
		dev_err(&i2c->dev, "si5351-iio: limiting phase_val to %d\n", phase_val);
	}
	degrees = phase_val * 90 / (fVCO / fout);

	dev_dbg(&i2c->dev, "si5351-iio: using fVCO=%d\n", fVCO);
	dev_dbg(&i2c->dev, "si5351-iio: found a=%d, b=%d, c=%d\n", a, b, c);
	dev_dbg(&i2c->dev, "si5351-iio: found p1=%d, p2=%d, p3=%d, divby4=%d\n", params.p1, params.p2, params.p3, divby4);
	dev_dbg(&i2c->dev, "si5351-iio: fout=%d\n", fout);
	dev_dbg(&i2c->dev, "si5351-iio: phase_val=%d\n", phase_val);

	start_reg = si5351_msynth_params_address(output);
	/* write multisynth parameters */
	si5351_write_parameters(i2c, start_reg, &params);

	if (fout > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* enable/disable integer mode and divby4 on multisynth0-5 */
	if (output < 6)
	{
		val = i2c_smbus_read_byte_data(i2c, start_reg + 2);
		if (divby4)
			val |= SI5351_OUTPUT_CLK_DIVBY4;
		else
			val &= ~SI5351_OUTPUT_CLK_DIVBY4;
		i2c_smbus_write_byte_data(i2c, start_reg + 2, val);

		val = i2c_smbus_read_byte_data(i2c, SI5351_CLK0_CTRL + output);
		val &= ~SI5351_CLK_INTEGER_MODE;
		i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + output, val);
		i2c_smbus_write_byte_data(i2c, SI5351_CLK0_PHASE_OFFSET + output, phase_val & 0x7F);
		dev_dbg(&i2c->dev, "si5351-iio: readback phase offset %d\n", i2c_smbus_read_byte_data(i2c, SI5351_CLK0_PHASE_OFFSET + output));
	}

	i2c_smbus_write_byte_data(i2c, SI5351_PLL_RESET,
						 (pll == PLL_A) ? SI5351_PLL_RESET_A :
								    SI5351_PLL_RESET_B);

	val = i2c_smbus_read_byte_data(i2c, SI5351_CLK0_CTRL + output);
	if (pll == PLL_B)
		val |= SI5351_CLK_PLL_SELECT;
	else
		val &= ~SI5351_CLK_PLL_SELECT;
	i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + output, val);

	dev_dbg(&i2c->dev, "si5351-iio: wrote CTRL byte %02x\n", val);

	return degrees;
}


static unsigned int si5351_ctrl_msynth(struct i2c_client *i2c, unsigned int output, unsigned int enable, unsigned int input, unsigned int strength, unsigned int inversion)
{
	int val;
	unsigned int bits = 0, allmask = 0;

	allmask = SI5351_CLK_INPUT_MASK | SI5351_CLK_DRIVE_STRENGTH_MASK | SI5351_CLK_INVERT | SI5351_CLK_POWERDOWN;

	input &= SI5351_CLK_INPUT_MASK;
	bits |= input;

	strength &= SI5351_CLK_DRIVE_STRENGTH_MASK;
	bits |= strength;

	if (inversion!=0)
		bits |= SI5351_CLK_INVERT;

	if (enable==0)
		bits |= SI5351_CLK_POWERDOWN;

	if (output < 8)
	{
		val = i2c_smbus_read_byte_data(i2c, SI5351_CLK0_CTRL + output);
		val &= ~allmask; // remove all masked bits
		val |= bits; // set bits where needed
		i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + output, val);
		dev_dbg(&i2c->dev, "si5351-iio: wrote CTRL byte %02x\n", val);

		val = i2c_smbus_read_byte_data(i2c, SI5351_OUTPUT_ENABLE_CTRL);
		if (enable==1)
			val &= ~(1 << output);
		else
			val |=  (1 << output);
		i2c_smbus_write_byte_data(i2c, SI5351_OUTPUT_ENABLE_CTRL, val);
		dev_dbg(&i2c->dev, "si5351-iio: wrote OUTPUT ENABLE byte %02x\n", val);
	}

	return bits;
}

static void si5351_safe_defaults(struct i2c_client *i2c)
{
	int i;
	i2c_smbus_write_byte_data(i2c, SI5351_OUTPUT_ENABLE_CTRL, 0xFF);
	for(i=0;i<8;i++)
		i2c_smbus_write_byte_data(i2c, SI5351_CLK0_CTRL + i, 0x80);
	i2c_smbus_write_byte_data(i2c, SI5351_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_10PF);

	return;
}

static int si5351_identify(struct i2c_client *client)
{
	int retval;

	retval = i2c_smbus_read_byte_data(client, 0);
	if (retval < 0)
		return retval;
	else
		return 0;
}

static int si5351_i2c_probe(struct i2c_client *i2c,	const struct i2c_device_id *id)
{
		struct iio_dev *indio_dev;
		struct device_node *np = i2c->dev.of_node;
		struct si5351_state *st;
		unsigned int i;
		int ret;
		unsigned int xtal_rate = DEFAULT_XTAL_RATE;

		if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		{
			dev_err(&i2c->dev, "I2C adapter not functional\n");
			return -EOPNOTSUPP;
		}
		dev_dbg(&i2c->dev, "si5351-iio: I2C adapter ok\n");
		ret = si5351_identify(i2c);
		if (ret < 0) {
			dev_err(&i2c->dev, "Si5351 not found: error %d\n", ret);
			return -ENODEV;
		}
		dev_dbg(&i2c->dev, "si5351-iio: I2C slave device found\n");
		indio_dev = devm_iio_device_alloc(&i2c->dev, sizeof(*st));
		if (indio_dev == NULL)
			return  -ENOMEM;


		st = iio_priv(indio_dev);
		dev_set_drvdata(&i2c->dev, indio_dev);

		st->chip_info = &si5351_chip_info_tbl[id->driver_data];
		st->dev = &i2c->dev;

		indio_dev->dev.parent = &i2c->dev;
		indio_dev->name = id->name;
		if (IS_ENABLED(CONFIG_OF) && np)
			of_property_read_string(np, "devname", &indio_dev->name);
		else
			dev_dbg(&i2c->dev, "using default name\n");

		if (IS_ENABLED(CONFIG_OF) && np)
		{
			ret = of_property_read_u32(np, "xtal-freq", &xtal_rate);
			if(ret)
                		xtal_rate = DEFAULT_XTAL_RATE;
		}

		indio_dev->info = &si5351_info;
		indio_dev->modes = INDIO_DIRECT_MODE;
		indio_dev->channels = st->chip_info->channels;
		indio_dev->num_channels = st->chip_info->num_channels;

		for (i = 0; i < st->chip_info->num_channels; ++i) {
			st->freq_cache[i] = 0;
		}

		ret = iio_device_register(indio_dev);

		si5351_safe_defaults(i2c);

		st->fVCO = si5351_setup_pll(i2c, PLL_A, 32*xtal_rate, xtal_rate);
		printk(KERN_INFO "si5351-iio: Si5351 detected, xtal freq = %d MHz, using PLL_A VCO freq = %d MHz\n", xtal_rate/1000000, st->fVCO/1000000);

		return 0;
}

static void si5351_i2c_remove(struct i2c_client *i2c)
{
		struct iio_dev *indio_dev = dev_get_drvdata(&i2c->dev);

		iio_device_unregister(indio_dev);
}

static const struct i2c_device_id si5351_i2c_ids[] = {
	{"si5351a", ID_SI5351A },
	{"si5351c", ID_SI5351C },
	{}
};
MODULE_DEVICE_TABLE(i2c, si5351_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id si5351_of_i2c_match[] = {
	{ .compatible = "silabs,si5351a", .data = (void *)ID_SI5351A },
	{ .compatible = "silabs,si5351c", .data = (void *)ID_SI5351C },
	{ },
};
MODULE_DEVICE_TABLE(of, si5351_of_i2c_match);
#else
#define si5351_of_i2c_match NULL
#endif


static struct i2c_driver si5351_i2c_driver = {
	.driver = {
		   .name = "si5351",
	},
	.probe = si5351_i2c_probe,
	.remove = si5351_i2c_remove,
	.id_table = si5351_i2c_ids,
};

static int __init si5351_i2c_register_driver(void)
{
	return i2c_add_driver(&si5351_i2c_driver);
}

static void __exit si5351_i2c_unregister_driver(void)
{
	i2c_del_driver(&si5351_i2c_driver);
}


static int __init si5351_init(void)
{
	int ret;

	ret = si5351_i2c_register_driver();

	return 0;
}
module_init(si5351_init);

static void __exit si5351_exit(void)
{
	si5351_i2c_unregister_driver();
}
module_exit(si5351_exit);

MODULE_AUTHOR("Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("Si5351 IIO driver");
MODULE_LICENSE("GPL v2");
