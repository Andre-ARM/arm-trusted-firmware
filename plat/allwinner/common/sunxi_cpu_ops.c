/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <debug.h>
#include <delay_timer.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <sunxi_mmap.h>
#include <sunxi_cpucfg.h>
#include <utils_def.h>

#include "sunxi_private.h"

static void sunxi_cpu_disable_power(unsigned int cluster, unsigned int core)
{
	if (mmio_read_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core)) == 0xff)
		return;

	VERBOSE("PSCI: Disabling power to cluster %d core %d\n", cluster, core);

	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0xff);
}

static void sunxi_cpu_enable_power(unsigned int cluster, unsigned int core)
{
	if (mmio_read_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core)) == 0)
		return;

	VERBOSE("PSCI: Enabling power to cluster %d core %d\n", cluster, core);

	/* Power enable sequence from original Allwinner sources */
	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0xfe);
	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0xf8);
	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0xe0);
	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0x80);
	mmio_write_32(SUNXI_CPU_POWER_CLAMP_REG(cluster, core), 0x00);
}

/*
 * Tell the "arisc" SCP core to turn an ARM core off.
 * If we don't have SCPI running, we create some OpenRISC code, put the
 * address of that into the reset vector and release the reset line.
 * The SCP will execute that code and pull that line up again.
 * We need to do the last part of sunxi_cpu_off() below: flipping bits
 * in three MMIO registers. To keep the assembly code compact, we fixup
 * the code to put the core number directly into the instructions. This
 * is easy since OpenRISC puts immediates always in the lower 16 bits.
 */
static void arisc_turn_off_arm_core(unsigned int cluster, unsigned int core)
{
	uintptr_t arisc_reset_vec = SUNXI_SRAM_A2_BASE - 0x4000 + 0x100;
	volatile uint32_t arisc_code[] = {
		0x19a001f0,	/*	l.movhi r13,0x1f0	*/
		0xaa200001,	/*	l.ori	r17,r0,0x1	*/
		0xba310000,	/*	l.slli	r17,r17,0x0  F!	*/
		0x9ded0000,	/*	l.addi	r15,r13,0    F!	*/
		0x84af1500,	/*	l.lwz	r5,5376(r15)	*/
		0xe0a58804,	/*	l.or	r5,r5,r17	*/
		0xd44f2d00,	/*	l.sw	5376(r15),r5	*/
		0xae31ffff,	/*	l.xori	r17,r17,-1	*/
		0x84af1c30,	/*	l.lwz	r5,7216(r15)	*/
		0xe0a58803,	/*	l.and	r5,r5,r17	*/
		0xd46f2c30,	/*	l.sw	7216(r15),r5	*/
		0xa8a000ff,	/*	l.ori	r5,r0,0xff	*/
		0x9ded0000,	/*	l.addi	r15,r13,0    F!	*/
		0xd44f2d40,	/*	l.sw	5440(r15),r5	*/
		0xd46d0400,	/*	l.sw	7168(r13),r0	*/
		0x00000000,	/*	l.j	3c		*/
		0x15000000,	/*	l.nop	0x0		*/
	};

	/* Patch up the code with the parameters */
	arisc_code[2] = (arisc_code[2] & ~0xffff) | core;
	arisc_code[3] = (arisc_code[3] & ~0xffff) | (cluster << 2);
	arisc_code[12] = (arisc_code[12] & ~0xffff) |
			 ((cluster << 4) | (core << 2));
	/*
	 * Is a DSB enough? Do we need to flush the array? Mapped uncacheable?
	 *
	 * flush_dcache_range((uintptr_t)arisc_code, sizeof(arisc_code));
	 */
	dsb();

	mmio_write_32(arisc_reset_vec,
		      ((uintptr_t)arisc_code - arisc_reset_vec) / 4);

	/* Pulse the arisc reset line. */
	mmio_clrbits_32(SUNXI_R_CPUCFG_BASE, BIT(0));
	udelay(1);
	mmio_setbits_32(SUNXI_R_CPUCFG_BASE, BIT(0));

	/* wait to be turned off */
	/* Hack to get into WFI quickly, until ARISC detects it properly. */
	wfi();
}

void sunxi_cpu_off(unsigned int cluster, unsigned int core)
{
	VERBOSE("PSCI: Powering off cluster %d core %d\n", cluster, core);

	/* Deassert DBGPWRDUP */
	mmio_clrbits_32(SUNXI_CPUCFG_DBG_REG0, BIT(core));

	/* We can't turn ourself off like this, but it works for other cores. */
	if (plat_my_core_pos() !=
	    cluster * PLATFORM_MAX_CPUS_PER_CLUSTER + core) {
		/* Activate the core output clamps */
		mmio_setbits_32(SUNXI_POWEROFF_GATING_REG(cluster), BIT(core));
		/* Assert CPU power-on reset */
		mmio_clrbits_32(SUNXI_POWERON_RST_REG(cluster), BIT(core));
		/* Remove power from the CPU */
		sunxi_cpu_disable_power(cluster, core);

		return;
	}

	/* Tell the SCP to do the dirty work for us. */
	arisc_turn_off_arm_core(cluster, core);
}

void sunxi_cpu_on(unsigned int cluster, unsigned int core)
{
	VERBOSE("PSCI: Powering on cluster %d core %d\n", cluster, core);

	/* Assert CPU core reset */
	mmio_clrbits_32(SUNXI_CPUCFG_RST_CTRL_REG(cluster), BIT(core));
	/* Assert CPU power-on reset */
	mmio_clrbits_32(SUNXI_POWERON_RST_REG(cluster), BIT(core));
	/* Set CPU to start in AArch64 mode */
	mmio_setbits_32(SUNXI_CPUCFG_CLS_CTRL_REG0(cluster), BIT(24 + core));
	/* Apply power to the CPU */
	sunxi_cpu_enable_power(cluster, core);
	/* Release the core output clamps */
	mmio_clrbits_32(SUNXI_POWEROFF_GATING_REG(cluster), BIT(core));
	/* Deassert CPU power-on reset */
	mmio_setbits_32(SUNXI_POWERON_RST_REG(cluster), BIT(core));
	/* Deassert CPU core reset */
	mmio_setbits_32(SUNXI_CPUCFG_RST_CTRL_REG(cluster), BIT(core));
	/* Assert DBGPWRDUP */
	mmio_setbits_32(SUNXI_CPUCFG_DBG_REG0, BIT(core));
}

void sunxi_disable_secondary_cpus(unsigned int primary_cpu)
{
	for (unsigned int cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu += 1) {
		if (cpu == primary_cpu)
			continue;
		sunxi_cpu_off(cpu / PLATFORM_MAX_CPUS_PER_CLUSTER,
			       cpu % PLATFORM_MAX_CPUS_PER_CLUSTER);
	}
}
