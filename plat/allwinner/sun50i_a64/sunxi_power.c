/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2018, Icenowy Zheng <icenowy@aosc.io>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <libfdt.h>
#include <mentor/mi2cv.h>
#include <mmio.h>
#include <platform_def.h>
#include <sunxi_def.h>
#include <sunxi_mmap.h>

enum pmic_type {
	GENERIC_H5,
	GENERIC_A64,
	REF_DESIGN_H5,	/* regulators controlled by GPIO pins on port L */
	AXP803_I2C,	/* PMIC on most A64 boards, using I2C */
} pmic;

#define AXP803_ADDR	(0x68 >> 1)

/* Enable GPIOs PL5, PL8 and PL9 as GPIO outputs. */
static void sunxi_init_pl_pio(void)
{
	/* un-gate PortL GPIO controller clock */
	mmio_setbits_32(SUNXI_R_PRCM_BASE + 0x28, BIT(0));

	/* program GPIOs to correct ON levels: PL5: 0, PL8,9: 1 */
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x10, 0x20, 0x300);

	/* switch pins PL5, PL8 and PL9 to GPIO out. */
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x00, 0xfU << 20, 0x1U << 20);
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x04, 0xff, 0x11);
}

/*
 * On boards without a proper PMIC we struggle to turn off the system properly.
 * Try to turn off as much off the system as we can, to reduce power
 * consumption. This should be entered with only one core running and SMP
 * disabled. This leaves core 0 running, it's the caller's responsibility
 * to turn that off (or go to WFI).
 */
void sunxi_turn_off_soc(uint16_t socid)
{
	int i;

	/** Turn off most peripherals, most importantly DRAM users. **/
	/* Keep DRAM controller running for now. */
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x2c0, ~BIT_32(14));
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x60, ~BIT_32(14));
	/* Keep msgbox and spinlock running for now (?) */
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x2c4, ~(BIT_32(21) | BIT_32(22)));
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x64, ~(BIT_32(21) | BIT_32(22)));
	mmio_write_32(SUNXI_CCU_BASE + 0x2c8, 0);
	/* Keep PIO controller running for now. */
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x68, ~(BIT_32(5)));
	mmio_write_32(SUNXI_CCU_BASE + 0x2d0, 0);
	/* Keep UART0 running for now. */
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x2d8, ~(BIT_32(16)));
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x6c, ~(BIT_32(16)));
	mmio_write_32(SUNXI_CCU_BASE + 0x70, 0);

	/** Turn off DRAM controller. **/
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x2c0, BIT_32(14));
	mmio_clrbits_32(SUNXI_CCU_BASE + 0x60, BIT_32(14));

	/** Migrate CPU and bus clocks away from the PLLs. **/
	/* AHB1: use OSC24M/1, APB1 = AHB1 / 2 */
	mmio_write_32(SUNXI_CCU_BASE + 0x54, 0x1000);
	/* APB2: use OSC24M */
	mmio_write_32(SUNXI_CCU_BASE + 0x58, 0x1000000);
	/* AHB2: use AHB1 clock */
	mmio_write_32(SUNXI_CCU_BASE + 0x5c, 0);
	/* CPU: use OSC24M */
	mmio_write_32(SUNXI_CCU_BASE + 0x50, 0x10000);

	/** turn off PLLs **/
	for (i = 0; i < 6; i++)
		mmio_clrbits_32(SUNXI_CCU_BASE + i * 8, BIT(31));
	switch (socid) {
		case SUNXI_SOC_H5:
			mmio_clrbits_32(SUNXI_CCU_BASE + 0x44, BIT(31));
			break;
		case SUNXI_SOC_A64:
			mmio_clrbits_32(SUNXI_CCU_BASE + 0x2c, BIT(31));
			mmio_clrbits_32(SUNXI_CCU_BASE + 0x4c, BIT(31));
			break;
	}
}

static int sunxi_init_r_i2c(uint16_t socid)
{
	uint32_t i2c_func;

	switch (socid) {
	case SUNXI_SOC_H5: i2c_func = 0x22; break;
	case SUNXI_SOC_A64: i2c_func = 0x33; break;
	default:
		INFO("R_I2C on Allwinner 0x%x SoC not supported\n", socid);
		return -ENODEV;
	}

	/* un-gate R_PIO clock */
	mmio_setbits_32(SUNXI_R_PRCM_BASE + 0x28, BIT(0));

	/* switch pins PL0 and PL1 to I2C */
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x00, 0xffU, i2c_func);

	/* level 2 drive strength */
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x14, 0x0fU, 0xaU);

	/* set both pins to pull-up */
	mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x1c, 0x0fU, 0x5U);

	/* assert, then de-assert reset of R_I2C */
	mmio_clrbits_32(SUNXI_R_PRCM_BASE + 0xb0, BIT(6));
	udelay(100);
	mmio_setbits_32(SUNXI_R_PRCM_BASE + 0xb0, BIT(6));

        /* un-gate R_I2C clock */
	mmio_setbits_32(SUNXI_R_PRCM_BASE + 0x28, BIT(6));

	/* call mi2cv driver */
	i2c_init((void *)SUNXI_R_I2C_BASE);

	return 0;
}

static int axp803_i2c_probe(void)
{
	int ret;
	uint8_t val = 0;

	/* read the "IC type no." register */
	ret = i2c_read(AXP803_ADDR, 0x3, 1, &val, 1);

	if ((ret == 0) && ((val & 0xcf) == 0x41))
		return 0;

	if (ret) {
		NOTICE("BL31: PMIC: No I2C communication with AXP803.\n");
		ret = -EPERM;
	} else
		NOTICE("BL31: PMIC: Non-AXP803 chip attached at AXP803's address.\n");

	return ret;
}

static int axp_write(uint8_t reg, uint8_t val)
{
	return i2c_write(AXP803_ADDR, reg, 1, &val, 1);
}

static int axp_setbits_8(uint8_t reg, uint8_t set_mask)
{
	uint8_t regval;
	int ret;

	ret = i2c_read(AXP803_ADDR, reg, 1, &regval, 1);
	if (ret)
		return ret;

	regval |= set_mask;

	return i2c_write(AXP803_ADDR, reg, 1, &regval, 1);
}

static bool fdt_handle_regulator(const void *fdt, int node)
{
	if (fdt_getprop(fdt, node, "phandle", NULL) != NULL)
		return true;
	if (fdt_getprop(fdt, node, "regulator-always-on", NULL) != NULL)
		return true;
	return false;
}

/*
 * Retrieve the voltage from a given regulator DTB node.
 * Both the regulator-{min,max}-microvolt properties must be present and
 * have the same value. Return that value in millivolts.
 */
static int fdt_get_regulator_millivolt(const void *fdt, int node)
{
	const fdt32_t *prop;
	uint32_t min_volt;

	prop = fdt_getprop(fdt, node, "regulator-min-microvolt", NULL);
	if (prop == NULL)
		return -EINVAL;
	min_volt = fdt32_to_cpu(*prop);

	prop = fdt_getprop(fdt, node, "regulator-max-microvolt", NULL);
	if (prop == NULL)
		return -EINVAL;

	if (fdt32_to_cpu(*prop) != min_volt)
		return -EINVAL;

	return min_volt / 1000;
}

#define NO_SPLIT 0xff

struct axp_regulator {
	char * dt_name;
	uint16_t min_volt;
	uint16_t max_volt;
	uint16_t step;
	unsigned char split;
	unsigned char volt_reg;
	unsigned char switch_reg;
	unsigned char switch_bit;
} regulators[] = {
	{"dcdc1", 1600, 3400, 100, NO_SPLIT, 0x20, 0xff, 9},
	{"dcdc5",  800, 1840,  10,       32, 0x24, 0xff, 9},
	{"dldo1",  700, 3300, 100, NO_SPLIT, 0x15, 0x12, 3},
	{"dldo2",  700, 4200, 100,       27, 0x16, 0x12, 4},
	{"dldo3",  700, 3300, 100, NO_SPLIT, 0x17, 0x12, 5},
	{"fldo1",  700, 1450,  50, NO_SPLIT, 0x1c, 0x13, 2},
	{}
};

static int setup_regulator(const void *fdt, int node,
			   const struct axp_regulator *reg)
{
	int mvolt;
	uint8_t regval;

	if (!fdt_handle_regulator(fdt, node))
		return -ENOENT;

	mvolt = fdt_get_regulator_millivolt(fdt, node);
	if (mvolt < reg->min_volt || mvolt > reg->max_volt)
		return -EINVAL;

	regval = (mvolt / reg->step) - (reg->min_volt / reg->step);
	if (regval > reg->split)
		regval = ((regval - reg->split) / 2) + reg->split;

	axp_write(reg->volt_reg, regval);
	if (reg->switch_reg < 0xff)
		axp_setbits_8(reg->switch_reg, BIT(reg->switch_bit));

	INFO("PMIC: AXP803: %s voltage: %d.%03dV\n", reg->dt_name,
	     mvolt / 1000, mvolt % 1000);

	return 0;
}

static void setup_axp803_rails(const void *fdt)
{
	int node;
	bool dc1sw = false;

	/* locate the PMIC DT node, bail out if not found */
	node = fdt_node_offset_by_compatible(fdt, -1, "x-powers,axp803");
	if (node == -FDT_ERR_NOTFOUND) {
		WARN("BL31: PMIC: No AXP803 DT node, skipping initial setup.\n");
		return;
	}

	if (fdt_getprop(fdt, node, "x-powers,drive-vbus-en", NULL))
		axp_setbits_8(0x8f, BIT(4));

	/* descend into the "regulators" subnode */
	node = fdt_first_subnode(fdt, node);

	/* iterate over all regulators to find used ones */
	for (node = fdt_first_subnode(fdt, node);
	     node != -FDT_ERR_NOTFOUND;
	     node = fdt_next_subnode(fdt, node)) {
		struct axp_regulator *reg;
		const char *name;
		int length;

		name = fdt_get_name(fdt, node, &length);
		for (reg = regulators; reg->dt_name; reg++) {
			if (!strncmp(name, reg->dt_name, length)) {
				setup_regulator(fdt, node, reg);
				break;
			}
		}

		if (!strncmp(name, "dc1sw", length)) {
			/* Delay DC1SW enablement to avoid overheating. */
			if (fdt_handle_regulator(fdt, node))
				dc1sw = true;
			continue;
		}
	}
	/*
	 * If DLDO2 is enabled after DC1SW, the PMIC overheats and shuts
	 * down. So always enable DC1SW as the very last regulator.
	 */
	if (dc1sw) {
		INFO("PMIC: AXP803: Enabling DC1SW\n");
		axp_setbits_8(0x12, BIT(7));
	}
}

int sunxi_pmic_setup(uint16_t socid, const void *fdt)
{
	int ret;

	switch (socid) {
	case SUNXI_SOC_H5:
		pmic = REF_DESIGN_H5;
		NOTICE("BL31: PMIC: Defaulting to PortL GPIO according to H5 reference design.\n");
		break;
	case SUNXI_SOC_A64:
		pmic = GENERIC_A64;
		ret = sunxi_init_r_i2c(socid);
		if (ret)
			return ret;

		ret = axp803_i2c_probe();
		if (ret)
			return ret;

		pmic = AXP803_I2C;
		NOTICE("BL31: PMIC: Found AXP803 on R_I2C.\n");
		setup_axp803_rails(fdt);
		break;
	default:
		NOTICE("BL31: PMIC: No support for Allwinner %x SoC.\n", socid);
		return -ENODEV;
	}
	return 0;
}

static int switch_axp_to_i2c(void)
{
	uint32_t reg = mmio_read_32(SUNXI_R_PIO_BASE + 0x00);

	/* If PL0/1 are configured for I2C already, there is nothing to do. */
	if ((reg & 0xff) == 0x33)
		return 0;

	/* If PL0/1 is not configured for RSB, we are clueless. Give up. */
	if ((reg & 0xff) != 0x22)
		return -ENODEV;

	/* The AXP is probably driven via RSB. Set it back to I2C. */
	mmio_write_32(SUNXI_R_RSB_BASE + 0x2c, 0x4e);	/* byte write */
	mmio_write_32(SUNXI_R_RSB_BASE + 0x30, 0x2d << 16);	/* chip addr */
	mmio_write_32(SUNXI_R_RSB_BASE + 0x10, 0x3e);	/* register 0x3e */
	mmio_write_32(SUNXI_R_RSB_BASE + 0x1c, 0);	/* != 0x7c: I2C */
	mmio_write_32(SUNXI_R_RSB_BASE + 0x00, 0x80);/* start transaction */

	/* wait for PMIC to switch over */
	udelay(1000);

	return 0;
}

void __dead2 sunxi_power_down(void)
{
	switch (pmic) {
	case GENERIC_H5:
		/* Turn off as many peripherals and clocks as we can. */
		sunxi_turn_off_soc(SUNXI_SOC_H5);
		break;
	case GENERIC_A64:
		/* Turn off as many peripherals and clocks as we can. */
		sunxi_turn_off_soc(SUNXI_SOC_A64);
		break;
	case REF_DESIGN_H5:
		sunxi_init_pl_pio();

		sunxi_turn_off_soc(SUNXI_SOC_H5);

		/*
		 * Switch PL pins to power off the board:
		 * - PL5 (VCC_IO) -> high
		 * - PL8 (PWR-STB = CPU power supply) -> low
		 * - PL9 (PWR-DRAM) ->low
		 * - PL10 (power LED) -> low
		 * Note: Clearing PL8 will reset the board, so keep it up.
		 */
		mmio_clrsetbits_32(SUNXI_R_PIO_BASE + 0x10, 0x600, 0x20);
		break;
	case AXP803_I2C:
		switch_axp_to_i2c();
		/* (Re-)init I2C in case the rich OS has disabled it. */
		sunxi_init_r_i2c(SUNXI_SOC_A64);

		/* Set "power disable control" bit */
		axp_setbits_8(0x32, BIT(7));
		break;
	default:
		break;
	}

	udelay(1000);
	ERROR("PSCI: Cannot turn off system, halting.\n");
	wfi();
	panic();
}
