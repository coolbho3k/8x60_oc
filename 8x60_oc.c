/*
 * Overclocked cpufreq driver for MSM 8x60 devices (Sensation, EVO 3D).
 * Partly based on the MSM architecture cpufreq driver and MSM 8x60 acpuclock
 * driver from Code Aurora Forum.
 *
 * See, HTC, this is possible even if you don't unlock the bootloader. It took
 * a while, but it can be done. So unlock the bootloader already!
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2011 Michael Huang
 * Author: Michael Huang <mike.g.huang@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/kallsyms.h>
#include <linux/sysfs.h>

#define DRIVER_AUTHOR "Michael Huang <coolbho3000@gmail.com>"
#define DRIVER_DESCRIPTION "MSM 8x60 Overclock Driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

/* Module params.
   Overclocked speed (MHz) = scpll_l_val * 27 * 2 */

static uint scpll_l_val = 0x1C; /* 1512MHz */
static uint vdd_uv = 1250000;

module_param(scpll_l_val, uint, 0444);
module_param(vdd_uv, uint, 0444);

#ifdef __ASSEMBLY__
#define IOMEM(x)	x
#else
#define IOMEM(x)	((void __force __iomem *)(x))
#endif

#define MSM_SCPLL_BASE          IOMEM(0xFA018000)
#define MSM_ACC0_BASE           IOMEM(0xFA300000)
#define MSM_ACC1_BASE           IOMEM(0xFA301000)
#define MSM_GCC_BASE            IOMEM(0xFA003000)
#define MSM_QFPROM_BASE         IOMEM(0xFA700000)

/* Frequency switch modes. */
#define SHOT_SWITCH		4
#define HOP_SWITCH		5
#define SIMPLE_SLEW		6
#define COMPLEX_SLEW		7

/* PLL calibration limits.
 * Qualcomm says the PLL hardware is capable of 384MHz to 1536MHz, but we know
 * they're bluffing! */
#define L_VAL_SCPLL_CAL_MIN	0x08 /* = 2*27*0x08 = 432 */
#define L_VAL_SCPLL_CAL_MAX	0x25 /* = 2*27*0x25 = 1998 */

#define MAX_VDD_SC		1250000 /* uV */
#define MAX_AXI			 310500 /* KHz */
#define SCPLL_LOW_VDD_FMAX	 594000 /* KHz */
#define SCPLL_LOW_VDD		1000000 /* uV */
#define SCPLL_NOMINAL_VDD	1100000 /* uV */

/* SCPLL Modes. */
#define SCPLL_POWER_DOWN	0
#define SCPLL_BYPASS		1
#define SCPLL_STANDBY		2
#define SCPLL_FULL_CAL		4
#define SCPLL_HALF_CAL		5
#define SCPLL_STEP_CAL		6
#define SCPLL_NORMAL		7

#define SCPLL_DEBUG_NONE	0
#define SCPLL_DEBUG_FULL	3

/* SCPLL registers offsets. */
#define SCPLL_DEBUG_OFFSET		0x0
#define SCPLL_CTL_OFFSET		0x4
#define SCPLL_CAL_OFFSET		0x8
#define SCPLL_STATUS_OFFSET		0x10
#define SCPLL_CFG_OFFSET		0x1C
#define SCPLL_FSM_CTL_EXT_OFFSET	0x24
#define SCPLL_LUT_A_HW_MAX		(0x38 + ((L_VAL_SCPLL_CAL_MAX / 4) * 4))

/* Clock registers. */
#define SPSS0_CLK_CTL_ADDR		(MSM_ACC0_BASE + 0x04)
#define SPSS0_CLK_SEL_ADDR		(MSM_ACC0_BASE + 0x08)
#define SPSS1_CLK_CTL_ADDR		(MSM_ACC1_BASE + 0x04)
#define SPSS1_CLK_SEL_ADDR		(MSM_ACC1_BASE + 0x08)
#define SPSS_L2_CLK_SEL_ADDR		(MSM_GCC_BASE  + 0x38)

/* Speed bin register. */
#define QFPROM_SPEED_BIN_ADDR		(MSM_QFPROM_BASE + 0x00C0)

static const void * const clk_ctl_addr[] = {SPSS0_CLK_CTL_ADDR,
			SPSS1_CLK_CTL_ADDR};
static const void * const clk_sel_addr[] = {SPSS0_CLK_SEL_ADDR,
			SPSS1_CLK_SEL_ADDR, SPSS_L2_CLK_SEL_ADDR};

static const int rpm_vreg_voter[] = { 1, 2 };
/*static struct regulator *regulator_sc[NR_CPUS];*/

enum scplls {
	CPU0 = 0,
	CPU1,
	L2,
};

static const void * const sc_pll_base[] = {
	[CPU0]	= MSM_SCPLL_BASE + 0x200,
	[CPU1]	= MSM_SCPLL_BASE + 0x300,
	[L2]	= MSM_SCPLL_BASE + 0x400,
};

enum sc_src {
	ACPU_AFAB,
	ACPU_PLL_8,
	ACPU_SCPLL,
};

struct clkctl_l2_speed {
	unsigned int     khz;
	unsigned int     src_sel;
	unsigned int     l_val;
	unsigned int     vdd_dig;
	unsigned int     vdd_mem;
	unsigned int     bw_level;
};

struct clkctl_acpu_speed {
	unsigned int     use_for_scaling[2]; /* One for each CPU. */
	unsigned int     acpuclk_khz;
	int              pll;
	unsigned int     acpuclk_src_sel;
	unsigned int     acpuclk_src_div;
	unsigned int     core_src_sel;
	unsigned int     l_val;
	struct clkctl_l2_speed *l2_level;
	unsigned int     vdd_sc;
	unsigned int     avsdscr_setting;
};

/* acpu_freq_tbl row to use when reconfiguring SC/L2 PLLs. */
#define CAL_IDX 1

enum setrate_reason {
         SETRATE_CPUFREQ = 0,
         SETRATE_SWFI,
         SETRATE_PC,
         SETRATE_HOTPLUG,
         SETRATE_INIT,
};

static void scpll_disable(int sc_pll)
{
	/* Power down SCPLL. */
	writel(SCPLL_POWER_DOWN, sc_pll_base[sc_pll] + SCPLL_CTL_OFFSET);
}

static void __init scpll_init(int sc_pll)
{
	uint32_t regval;

	printk(KERN_INFO "8x60_oc: Clear calibration LUT registers %u\n", sc_pll);
	/* Clear calibration LUT registers containing max frequency entry.
	 * LUT registers are only writeable in debug mode. */
	writel(SCPLL_DEBUG_FULL, sc_pll_base[sc_pll] + SCPLL_DEBUG_OFFSET);
	writel(0x0, sc_pll_base[sc_pll] + SCPLL_LUT_A_HW_MAX);
	writel(SCPLL_DEBUG_NONE, sc_pll_base[sc_pll] + SCPLL_DEBUG_OFFSET);

	printk(KERN_INFO "8x60_oc: SCPLL standby mode %u\n", sc_pll);
	/* Power-up SCPLL into standby mode. */
	writel(SCPLL_STANDBY, sc_pll_base[sc_pll] + SCPLL_CTL_OFFSET);
	dsb();
	udelay(10);

	printk(KERN_INFO "8x60_oc: Calibrate SCPLL %u\n", sc_pll);
	/* Calibrate the SCPLL to the maximum range supported by the h/w. We
	 * might not use the full range of calibrated frequencies, but this
	 * simplifies changes required for future increases in max CPU freq.
	 */
	regval = (L_VAL_SCPLL_CAL_MAX << 24) | (L_VAL_SCPLL_CAL_MIN << 16);
	writel(regval, sc_pll_base[sc_pll] + SCPLL_CAL_OFFSET);

	printk(KERN_INFO "8x60_oc: Calibrate SCPLL start %u\n", sc_pll);
	/* Start calibration */
	writel(SCPLL_FULL_CAL, sc_pll_base[sc_pll] + SCPLL_CTL_OFFSET);

	/* Wait for proof that calibration has started before checking the
	 * 'calibration done' bit in the status register. Waiting for the
	 * LUT register we cleared to contain data accomplishes this.
	 * This is required since the 'calibration done' bit takes time to
	 * transition from 'done' to 'not done' when starting a calibration.
	 */
	printk(KERN_INFO "8x60_oc: Wait for calibration start %u\n", sc_pll);
	while (readl(sc_pll_base[sc_pll] + SCPLL_LUT_A_HW_MAX) == 0)
		cpu_relax();

	printk(KERN_INFO "8x60_oc: Wait for calibration complete %u\n", sc_pll);
	/* Wait for calibration to complete. */
	while (readl(sc_pll_base[sc_pll] + SCPLL_STATUS_OFFSET) & 0x2)
		cpu_relax();

	printk(KERN_INFO "8x60_oc: Disable SCPLL %u\n", sc_pll);
	/* Power-down SCPLL. */
	scpll_disable(sc_pll);
}

/* What 1250 mV and 1450 mV look like in memory */
static char hex_1250mv[4] __initdata =
		"\xd0\x12\x13\x00" //dcd 0x1312d0
		;

static char hex_1450mv[4] __initdata =
		"\x10\x20\x16\x00" //dcd 0x162010
		;

static char hex_1188mhz[4] __initdata =
		"\x00\x71\xcf\x46" //0x46cf7100
		;

static char hex_hugemhz[4] __initdata =
		"\x00\x5e\xd0\xb2" //0xb2d05e00
		;

static int __init overclock_init(void)
{
	struct cpufreq_policy *policy_cpu0;
	struct cpufreq_policy *policy_cpu1;
	struct cpufreq_frequency_table *freq_table_cpu0;
	struct cpufreq_frequency_table *freq_table_cpu1;
	unsigned long scan = 0xc0000000;
	struct clkctl_acpu_speed *table;
	char *voltage;
	int i;
	int ret;

	freq_table_cpu0 = cpufreq_frequency_get_table(0);
	freq_table_cpu1 = cpufreq_frequency_get_table(1);

	printk(KERN_INFO "8x60_oc: %s version %s\n", DRIVER_DESCRIPTION,
		DRIVER_VERSION);
	printk(KERN_INFO "8x60_oc: by %s\n", DRIVER_AUTHOR);

	policy_cpu0 = cpufreq_cpu_get(CPU0);
	policy_cpu1 = cpufreq_cpu_get(CPU1);

	printk(KERN_INFO "8x60_oc: Recalibrating SCPLL for CPU0\n");
	scpll_init(CPU0);

	printk(KERN_INFO "8x60_oc: Recalibrating SCPLL for CPU1\n");
	scpll_init(CPU1);

	/* Scan memory for 1250mV and replace with 1450mV. Currently hacky. */
	printk(KERN_INFO "8x60_oc: *** SCANNING FOR VOLTAGES ***\n");
	while(scan < 0xc1000000)
	{
		scan++;
		voltage = (char*) scan;
		if(*voltage == hex_1250mv[0])
		{
			for(i = 1; i <= 3; i++)
			{
				voltage = (char*) (scan+i);
				if(*voltage != hex_1250mv[i])
					break;
			}
		
			if(i >= 3)
			{
				printk(KERN_INFO "8x60_oc: EPIC PWN: %lu\n", scan);
				memcpy((void*) scan, &hex_1450mv,
					sizeof(hex_1450mv));
			}
		}
	}

	/* Scan memory for clkctl_acpu_speed. Slightly less hacky. */
	printk(KERN_INFO "8x60_oc: *** SCANNING FOR TABLE ***\n");
	scan = 0xc071b000;
	while(scan < 0xc1000000)
	{
		scan++;

		/* We are sure of the values of the second column of the first
		three rows of the struct (and actually more, but this is all
		that's really necessary). */

		table = (struct clkctl_acpu_speed*) scan;
		if(table[0].acpuclk_khz == 192000 && table[1].acpuclk_khz ==
			MAX_AXI && table[2].acpuclk_khz == 384000)
		{
			printk(KERN_INFO "8x60_oc: EPIC PWN: %lu\n", scan);
			break;
		}
	}

	/* Make any changes we want to the clkctl_acpu_speed. We have to
	   sacrifice the 1134000 speed to make room for our overclocked speed.*
	   */
	printk(KERN_INFO "8x60_oc: Injecting new table\n");

	if(table[17].acpuclk_khz == 1188000)
	{
	table[17].acpuclk_khz = scpll_l_val*2*27*1000;
	table[17].l_val = scpll_l_val;
	table[17].vdd_sc = vdd_uv;
	table[16].acpuclk_khz = 1188000;
	table[16].l_val = 0x16;
	table[16].vdd_sc = 1187500;
	}
	printk(KERN_INFO "8x60_oc: Tables injected\n");

	/* Register new table with cpufreq */
	printk(KERN_INFO "8x60_oc: Registering new cpufreq tables\n");

	freq_table_cpu0[16].frequency = scpll_l_val*2*27*1000;
	freq_table_cpu1[16].frequency = scpll_l_val*2*27*1000;
	freq_table_cpu0[15].frequency = 1188000;
	freq_table_cpu1[15].frequency = 1188000;

	ret = sysfs_create_file(&policy_cpu0->kobj,
		&cpufreq_freq_attr_scaling_available_freqs.attr);

	/* Notify the policy of changes to max freq */	
	printk(KERN_INFO "8x60_oc: Notifying cpufreq\n");
	policy_cpu0->cpuinfo.min_freq = 192000;
	policy_cpu0->cpuinfo.max_freq = scpll_l_val*2*27*1000;
	policy_cpu0->min = 384000;
	policy_cpu0->max = 1188000;
	
	/* perflock_notifier_call does not seem to work in this kernel, but
	   perflock is still insidious because it locks us to 1.2GHz (HARD
	   coded by HTC) when we're doing certain things. */
	scan = 0xc0000000;

	while(scan < 0xc1000000)
	{
		scan++;
		voltage = (char*) scan;
		if(*voltage == hex_1188mhz[0])
		{
			for(i = 1; i <= 3; i++)
			{
				voltage = (char*) (scan+i);
				if(*voltage != hex_1188mhz[i])
					break;
			}
		
			if(i >= 3)
			{
				printk(KERN_INFO "8x60_oc: EPIC PWN: %lu\n", scan);
				memcpy((void*) scan, &hex_hugemhz,
					sizeof(hex_hugemhz));
				break;
			}
		}
	}

	return 0;
}

static void __exit overclock_exit(void)
{
	printk(KERN_INFO "8x60_oc: unloaded\n");
}

module_init(overclock_init);
module_exit(overclock_exit);

