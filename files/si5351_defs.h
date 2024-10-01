/*
 * clk-si5351.h: Silicon Laboratories Si5351A/B/C I2C Clock Generator
 *
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _SI5351_DEFS_H_
#define _SI5351_DEFS_H_

#define SI5351_BUS_BASE_ADDR			0x60

#define SI5351_PLL_VCO_MIN			600000000
#define SI5351_PLL_VCO_MAX			900000000
#define SI5351_MULTISYNTH_MIN_FREQ		1000000
#define SI5351_MULTISYNTH_DIVBY4_FREQ		150000000
#define SI5351_MULTISYNTH_MAX_FREQ		160000000
#define SI5351_MULTISYNTH67_MAX_FREQ		SI5351_MULTISYNTH_DIVBY4_FREQ
#define SI5351_CLKOUT_MIN_FREQ			8000
#define SI5351_CLKOUT_MAX_FREQ			SI5351_MULTISYNTH_MAX_FREQ
#define SI5351_CLKOUT67_MAX_FREQ		SI5351_MULTISYNTH67_MAX_FREQ

#define SI5351_PLL_A_MIN			15
#define SI5351_PLL_A_MAX			90
#define SI5351_PLL_B_MAX			(SI5351_PLL_C_MAX-1)
#define SI5351_PLL_C_MAX			1048575
#define SI5351_MULTISYNTH_A_MIN			6
#define SI5351_MULTISYNTH_A_MAX			1800
#define SI5351_MULTISYNTH67_A_MAX		254
#define SI5351_MULTISYNTH_B_MAX			(SI5351_MULTISYNTH_C_MAX-1)
#define SI5351_MULTISYNTH_C_MAX			1048575
#define SI5351_MULTISYNTH_P1_MAX		((1<<18)-1)
#define SI5351_MULTISYNTH_P2_MAX		((1<<20)-1)
#define SI5351_MULTISYNTH_P3_MAX		((1<<20)-1)

#define SI5351_DEVICE_STATUS			0
#define SI5351_INTERRUPT_STATUS			1
#define SI5351_INTERRUPT_MASK			2
#define  SI5351_STATUS_SYS_INIT			(1<<7)
#define  SI5351_STATUS_LOL_B			(1<<6)
#define  SI5351_STATUS_LOL_A			(1<<5)
#define  SI5351_STATUS_LOS			(1<<4)
#define SI5351_OUTPUT_ENABLE_CTRL		3
#define SI5351_OEB_PIN_ENABLE_CTRL		9
#define SI5351_PLL_INPUT_SOURCE			15
#define  SI5351_CLKIN_DIV_MASK			(3<<6)
#define  SI5351_CLKIN_DIV_1			(0<<6)
#define  SI5351_CLKIN_DIV_2			(1<<6)
#define  SI5351_CLKIN_DIV_4			(2<<6)
#define  SI5351_CLKIN_DIV_8			(3<<6)
#define  SI5351_PLLB_SOURCE			(1<<3)
#define  SI5351_PLLA_SOURCE			(1<<2)

#define SI5351_CLK0_CTRL			16
#define SI5351_CLK1_CTRL			17
#define SI5351_CLK2_CTRL			18
#define SI5351_CLK3_CTRL			19
#define SI5351_CLK4_CTRL			20
#define SI5351_CLK5_CTRL			21
#define SI5351_CLK6_CTRL			22
#define SI5351_CLK7_CTRL			23
#define  SI5351_CLK_POWERDOWN			(1<<7)
#define  SI5351_CLK_INTEGER_MODE		(1<<6)
#define  SI5351_CLK_PLL_SELECT			(1<<5)
#define  SI5351_CLK_INVERT			(1<<4)
#define  SI5351_CLK_INPUT_MASK			(3<<2)
#define  SI5351_CLK_INPUT_XTAL			(0<<2)
#define  SI5351_CLK_INPUT_CLKIN			(1<<2)
#define  SI5351_CLK_INPUT_MULTISYNTH_0_4	(2<<2)
#define  SI5351_CLK_INPUT_MULTISYNTH_N		(3<<2)
#define  SI5351_CLK_DRIVE_STRENGTH_MASK		(3<<0)
#define  SI5351_CLK_DRIVE_STRENGTH_2MA		(0<<0)
#define  SI5351_CLK_DRIVE_STRENGTH_4MA		(1<<0)
#define  SI5351_CLK_DRIVE_STRENGTH_6MA		(2<<0)
#define  SI5351_CLK_DRIVE_STRENGTH_8MA		(3<<0)

#define SI5351_CLK3_0_DISABLE_STATE		24
#define SI5351_CLK7_4_DISABLE_STATE		25
#define  SI5351_CLK_DISABLE_STATE_MASK		3
#define  SI5351_CLK_DISABLE_STATE_LOW		0
#define  SI5351_CLK_DISABLE_STATE_HIGH		1
#define  SI5351_CLK_DISABLE_STATE_FLOAT		2
#define  SI5351_CLK_DISABLE_STATE_NEVER		3

#define SI5351_PARAMETERS_LENGTH		8
#define SI5351_PLLA_PARAMETERS			26
#define SI5351_PLLB_PARAMETERS			34
#define SI5351_CLK0_PARAMETERS			42
#define SI5351_CLK1_PARAMETERS			50
#define SI5351_CLK2_PARAMETERS			58
#define SI5351_CLK3_PARAMETERS			66
#define SI5351_CLK4_PARAMETERS			74
#define SI5351_CLK5_PARAMETERS			82
#define SI5351_CLK6_PARAMETERS			90
#define SI5351_CLK7_PARAMETERS			91
#define SI5351_CLK6_7_OUTPUT_DIVIDER		92
#define  SI5351_OUTPUT_CLK_DIV_MASK		(7 << 4)
#define  SI5351_OUTPUT_CLK6_DIV_MASK		(7 << 0)
#define  SI5351_OUTPUT_CLK_DIV_SHIFT		4
#define  SI5351_OUTPUT_CLK_DIV6_SHIFT		0
#define  SI5351_OUTPUT_CLK_DIV_1		0
#define  SI5351_OUTPUT_CLK_DIV_2		1
#define  SI5351_OUTPUT_CLK_DIV_4		2
#define  SI5351_OUTPUT_CLK_DIV_8		3
#define  SI5351_OUTPUT_CLK_DIV_16		4
#define  SI5351_OUTPUT_CLK_DIV_32		5
#define  SI5351_OUTPUT_CLK_DIV_64		6
#define  SI5351_OUTPUT_CLK_DIV_128		7
#define  SI5351_OUTPUT_CLK_DIVBY4		(3<<2)

#define SI5351_SSC_PARAM0			149
#define SI5351_SSC_PARAM1			150
#define SI5351_SSC_PARAM2			151
#define SI5351_SSC_PARAM3			152
#define SI5351_SSC_PARAM4			153
#define SI5351_SSC_PARAM5			154
#define SI5351_SSC_PARAM6			155
#define SI5351_SSC_PARAM7			156
#define SI5351_SSC_PARAM8			157
#define SI5351_SSC_PARAM9			158
#define SI5351_SSC_PARAM10			159
#define SI5351_SSC_PARAM11			160
#define SI5351_SSC_PARAM12			161

#define SI5351_VXCO_PARAMETERS_LOW		162
#define SI5351_VXCO_PARAMETERS_MID		163
#define SI5351_VXCO_PARAMETERS_HIGH		164

#define SI5351_CLK0_PHASE_OFFSET		165
#define SI5351_CLK1_PHASE_OFFSET		166
#define SI5351_CLK2_PHASE_OFFSET		167
#define SI5351_CLK3_PHASE_OFFSET		168
#define SI5351_CLK4_PHASE_OFFSET		169
#define SI5351_CLK5_PHASE_OFFSET		170

#define SI5351_PLL_RESET			177
#define  SI5351_PLL_RESET_B			(1<<7)
#define  SI5351_PLL_RESET_A			(1<<5)

#define SI5351_CRYSTAL_LOAD			183
#define  SI5351_CRYSTAL_LOAD_MASK		(3<<6)
#define  SI5351_CRYSTAL_LOAD_6PF		(1<<6)
#define  SI5351_CRYSTAL_LOAD_8PF		(2<<6)
#define  SI5351_CRYSTAL_LOAD_10PF		(3<<6)

#define SI5351_FANOUT_ENABLE			187
#define  SI5351_CLKIN_ENABLE			(1<<7)
#define  SI5351_XTAL_ENABLE			(1<<6)
#define  SI5351_MULTISYNTH_ENABLE		(1<<4)

/**
 * enum si5351_variant - SiLabs Si5351 chip variant
 * @SI5351_VARIANT_A: Si5351A (8 output clocks, XTAL input)
 * @SI5351_VARIANT_A3: Si5351A MSOP10 (3 output clocks, XTAL input)
 * @SI5351_VARIANT_B: Si5351B (8 output clocks, XTAL/VXCO input)
 * @SI5351_VARIANT_C: Si5351C (8 output clocks, XTAL/CLKIN input)
 */
enum si5351_variant {
	SI5351_VARIANT_A = 1,
	SI5351_VARIANT_A3 = 2,
	SI5351_VARIANT_B = 3,
	SI5351_VARIANT_C = 4,
};

#define SI5351_MAX_CHANNELS 8
#define PLL_A 0
#define PLL_B 1
#define DEFAULT_XTAL_RATE 25000000
#define TUNE_STEP 500

enum {
	SI5351_FREQ,
	SI5351_PHASE,
};

struct si5351_multisynth_parameters {
	unsigned long	p1;
	unsigned long	p2;
	unsigned long	p3;
	int		valid;
	int		intmode;
};

struct si5351_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

struct si5351_state {
	struct device			*dev;
	const struct si5351_chip_info	*chip_info;
	unsigned int			freq_cache[SI5351_MAX_CHANNELS];
	unsigned int			phase_cache[SI5351_MAX_CHANNELS];
	unsigned int			fVCO;
	unsigned int			xtal_rate;
	int 				quad_mode;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		u8 i2c[3];
	} data ____cacheline_aligned;
};

/*
enum si5351_type {
	ID_SI5351A,
	ID_SI5351C,
};
*/
#define ID_SI5351A 0x60
#define ID_SI5351C 0x61

#endif
