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

static int si5351_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m);
static int si5351_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask);

static ssize_t si5351_write_ext(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len);

static ssize_t si5351_read_ext(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf);


static int si5351_setup_pll(struct i2c_client *i2c, unsigned int pll, unsigned int fVCO, unsigned int fXTAL);

static inline u8 si5351_msynth_params_address(int num);

static unsigned int si5351_config_msynth(struct i2c_client *i2c, unsigned int output, unsigned int pll, unsigned int fout, unsigned int fVCO);
static unsigned int si5351_config_msynth_phase(struct i2c_client *i2c, unsigned int output, unsigned int pll, unsigned int fout, unsigned int fVCO, unsigned int degrees);
static unsigned int si5351_ctrl_msynth(struct i2c_client *i2c, unsigned int output, unsigned int enable, unsigned int input, unsigned int strength, unsigned int inversion);
static void si5351_safe_defaults(struct i2c_client *i2c);
static int si5351_identify(struct i2c_client *client);
static int si5351_i2c_probe(struct i2c_client *i2c,	const struct i2c_device_id *id);
static int si5351_i2c_remove(struct i2c_client *i2c);
static int __init si5351_i2c_register_driver(void);
static void __exit si5351_i2c_unregister_driver(void);
static int __init si5351_init(void);
static void __exit si5351_exit(void);
