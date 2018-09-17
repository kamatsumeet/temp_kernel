/*
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 *  Copyright (C) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Interrupt architecture for the GIC:
 *
 * o There is one Interrupt Distributor, which receives interrupts
 *   from system devices and sends them to the Interrupt Controllers.
 *
 * o There is one CPU Interface per CPU, which sends interrupts sent
 *   by the Distributor, and interrupts generated locally, to the
 *   associated CPU. The base address of the CPU interface is usually
 *   aliased so that the same address points to different chips depending
 *   on the CPU it is accessed from.
 *
 * Note that IRQs 0-31 are special - they are local to each CPU.
 * As such, the enable set/clear, pending set/clear and active bit
 * registers are banked per-cpu for these sources.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/acpi.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/cputype.h>
#ifdef CONFIG_FIQ
#include <asm/fiq.h>
#endif
#include <asm/irq.h>
#include <asm/exception.h>
#include <asm/smp_plat.h>
#include <asm/virt.h>
#ifdef CONFIG_TEGRA_APE_AGIC
#include <linux/irqchip/tegra-agic.h>
#endif

#include "irq-gic-common.h"

#ifdef CONFIG_ARM64
#include <asm/cpufeature.h>

static void gic_check_cpu_features(void)
{
	WARN_TAINT_ONCE(cpus_have_cap(ARM64_HAS_SYSREG_GIC_CPUIF),
			TAINT_CPU_OUT_OF_SPEC,
			"GICv3 system registers enabled, broken firmware!\n");
}
#else
#define gic_check_cpu_features()	do { } while(0)
#endif

#ifdef CONFIG_TEGRA_APE_AGIC
/* APE CLOCKS */
#define CLK_SOURCE_APE			0x6c0
#define CLK_OUT_ENB_SET_Y		0x29c
#define CLK_OUT_ENB_SET_V		0x440
#define CLK_RST_DEV_Y_CLR		0x2ac

#define	SELECT_CLK_M			(6 << 29)
#define ENABLE_APE_CLK			(1 << 6)
#define ENABLE_APB2APE_CLK		(1 << 11)
#define RESET_APE				(1 << 6)

static int enable_t210_agic_clks(struct device_node *node)
{
	void __iomem *clk_base;
	u32 val;

	clk_base = of_iomap(node, 2);
	WARN(!clk_base, "unable to map agic clock registers\n");

	/* Set CLK M as APE clk's soruce */
	val = readl(clk_base + CLK_SOURCE_APE);
	val &= ~GENMASK(31, 29);
	val |= SELECT_CLK_M;
	writel(val, clk_base +  CLK_SOURCE_APE);

	writel(ENABLE_APE_CLK, clk_base + CLK_OUT_ENB_SET_Y);
	writel(ENABLE_APB2APE_CLK, clk_base + CLK_OUT_ENB_SET_V);
	udelay(2);
	writel(RESET_APE, clk_base + CLK_RST_DEV_Y_CLR);

	pr_info("%s:%d ape clocked & reset cleared\n", __func__, __LINE__);

	return 0;
}
#endif

union gic_base {
	void __iomem *common_base;
	void __percpu * __iomem *percpu_base;
};

struct gic_chip_data {
	union gic_base dist_base;
	union gic_base cpu_base;
	struct notifier_block pm_notifier_block;
	u32 saved_spi_enable[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_active[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(1020, 16)];
	u32 saved_spi_target[DIV_ROUND_UP(1020, 4)];
	u32 saved_spi_group[DIV_ROUND_UP(1020, 32)];
	u32 __percpu *saved_ppi_enable;
	u32 __percpu *saved_ppi_active;
	u32 __percpu *saved_ppi_conf;
	struct irq_domain *domain;
	unsigned int gic_irqs;
#ifdef CONFIG_GIC_NON_BANKED
	void __iomem *(*get_base)(union gic_base *);
#endif
	bool is_percpu;
#ifdef CONFIG_FIQ
	bool fiq_enable;
#endif
	bool is_agic;
	u32 num_interfaces;
};

static DEFINE_RAW_SPINLOCK(irq_controller_lock);

/*
 * The GIC mapping of CPU interfaces does not necessarily match
 * the logical CPU numbering.  Let's use a mapping as returned
 * by the GIC itself.
 */
#define NR_GIC_CPU_IF 8
static u8 gic_cpu_map[NR_GIC_CPU_IF] __read_mostly;

static struct static_key supports_deactivate = STATIC_KEY_INIT_TRUE;

#ifndef MAX_GIC_NR
#define MAX_GIC_NR	1
#endif

static struct gic_chip_data gic_data[MAX_GIC_NR] __read_mostly;

static u8 gic_get_cpumask(struct gic_chip_data *gic);

#ifdef CONFIG_GIC_NON_BANKED
static void __iomem *gic_get_percpu_base(union gic_base *base)
{
	return raw_cpu_read(*base->percpu_base);
}

static void __iomem *gic_get_common_base(union gic_base *base)
{
	return base->common_base;
}

static inline void __iomem *gic_data_dist_base(struct gic_chip_data *data)
{
	return data->get_base(&data->dist_base);
}

static inline void __iomem *gic_data_cpu_base(struct gic_chip_data *data)
{
	return data->get_base(&data->cpu_base);
}

static inline void gic_set_base_accessor(struct gic_chip_data *data,
					 void __iomem *(*f)(union gic_base *))
{
	data->get_base = f;
}
#else
#define gic_data_dist_base(d)	((d)->dist_base.common_base)
#define gic_data_cpu_base(d)	((d)->cpu_base.common_base)
#define gic_set_base_accessor(d, f)
#endif

#ifdef CONFIG_TEGRA_APE_AGIC

static struct gic_chip_data *tegra_agic;
static bool tegra_agic_suspended;
static int gic_notifier(struct notifier_block *, unsigned long, void *);

bool tegra_agic_irq_is_pending(int irq)
{
	int value;
	void __iomem *dist_base;
	u32 pending;

	BUG_ON(!tegra_agic);

	dist_base = gic_data_dist_base(tegra_agic);
	pending = GIC_DIST_PENDING_SET + (irq / 32 * 4);

	value = readl_relaxed(dist_base + pending);
	/* checks the irq bit is set */
	return value & (1 << (irq % 32));
}
EXPORT_SYMBOL_GPL(tegra_agic_irq_is_pending);


void tegra_agic_clear_pending(int irq)
{
	void __iomem *dist_base;
	u32 irq_target;
	u32 pending;
	u8 curr_cpu;
	u8 val8;

	BUG_ON(!tegra_agic);

	pending = GIC_DIST_PENDING_CLEAR + (irq / 32 * 4);
	dist_base = gic_data_dist_base(tegra_agic);
	curr_cpu = gic_get_cpumask(tegra_agic);
	irq_target = GIC_DIST_TARGET + irq;

	raw_spin_lock(&irq_controller_lock);
	val8 = readb_relaxed(dist_base + irq_target);
	if (!tegra_agic->is_percpu && !(val8 & curr_cpu)) {
		pr_err("irq %d does not belong to this cpu\n", irq);
		goto end;
	}

	writel_relaxed(1 << (irq % 32), dist_base + pending);
end:
	raw_spin_unlock(&irq_controller_lock);
}
EXPORT_SYMBOL_GPL(tegra_agic_clear_pending);

bool tegra_agic_irq_is_active(int irq)
{
	void __iomem *dist_base;
	int value;
	u32 active;

	BUG_ON(!tegra_agic);

	dist_base = gic_data_dist_base(tegra_agic);
	active = GIC_DIST_ACTIVE_SET + (irq / 32 * 4);

	value = readl_relaxed(dist_base + active);
	/* checks the irq bit is set */
	return value & (1 << (irq % 32));
}
EXPORT_SYMBOL_GPL(tegra_agic_irq_is_active);

void tegra_agic_clear_active(int irq)
{
	void __iomem *dist_base;
	u32 irq_target;
	u8 curr_cpu;
	u32 active;
	u8 val8;

	BUG_ON(!tegra_agic);

	active = GIC_DIST_ACTIVE_CLEAR + (irq / 32 * 4);
	dist_base = gic_data_dist_base(tegra_agic);
	curr_cpu = gic_get_cpumask(tegra_agic);
	irq_target = GIC_DIST_TARGET + irq;

	raw_spin_lock(&irq_controller_lock);
	val8 = readb_relaxed(dist_base + irq_target);
	if (!tegra_agic->is_percpu && !(val8 & curr_cpu)) {
		pr_err("irq %d does not belong to this cpu\n", irq);
		goto end;
	}

	writel_relaxed(1 << (irq % 32), dist_base + active);
end:
	raw_spin_unlock(&irq_controller_lock);
}
EXPORT_SYMBOL_GPL(tegra_agic_clear_active);


int tegra_agic_route_interrupt(int irq, enum tegra_agic_cpu cpu)
{
	void __iomem *dist_base;
	u32 irq_target;
	u32 shift;
	u32 irq_clear_enable;
	u32 val32;
	u32 irq_aff;
	u8 routing_cpu = 1 << (u32)cpu;
	unsigned long flags;

	BUG_ON(!tegra_agic);

	dist_base = gic_data_dist_base(tegra_agic);
	irq_target = GIC_DIST_TARGET + (irq & ~3);
	shift = (irq % 4) * 8;
	irq_clear_enable = GIC_DIST_ENABLE_CLEAR + (irq / 32) * 4;

	raw_spin_lock_irqsave(&irq_controller_lock, flags);
	irq_aff = readl_relaxed(dist_base + irq_target);
	if (irq_aff & (routing_cpu << shift)) {
		raw_spin_unlock_irqrestore(&irq_controller_lock,
							flags);
		pr_debug("routing agic irq %d to same cpu\n", irq);
		return 0;
	}

	val32 = readl(dist_base + irq_clear_enable);

	/* Check whether the irq is enabled */
	if (val32 & (1 << (irq % 32))) {
		raw_spin_unlock_irqrestore(&irq_controller_lock,
							flags);
		pr_info("agic irq %d is enabled, cannot be routed\n",
								irq);
		return -EPERM;
	}

	/* clear the byte with the word field */
	irq_aff = irq_aff & ~(0xFF << shift);
	writel_relaxed(irq_aff, dist_base + irq_target);
	irq_aff = irq_aff | (routing_cpu << shift);
	writel_relaxed(irq_aff, dist_base + irq_target);
	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_agic_route_interrupt);

void tegra_agic_save_registers(void)
{
	BUG_ON(!tegra_agic);

	gic_notifier(&tegra_agic->pm_notifier_block,
			MOD_DOMAIN_POWER_OFF, NULL);
}
EXPORT_SYMBOL_GPL(tegra_agic_save_registers);

void tegra_agic_restore_registers(void)
{
	BUG_ON(!tegra_agic);

	gic_notifier(&tegra_agic->pm_notifier_block,
			MOD_DOMAIN_POWER_ON, NULL);
}
EXPORT_SYMBOL_GPL(tegra_agic_restore_registers);
#endif

static inline bool gic_data_fiq_enable(struct gic_chip_data *data)
{
#ifdef CONFIG_FIQ
	return data->fiq_enable;
#else
	return false;
#endif
}

static inline void __iomem *gic_dist_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_dist_base(gic_data);
}

static inline void __iomem *gic_cpu_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_cpu_base(gic_data);
}

static inline unsigned int gic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static inline bool cascading_gic_irq(struct irq_data *d)
{
	void *data = irq_data_get_irq_handler_data(d);

	/*
	 * If handler_data is set, this is a cascading interrupt, and
	 * it cannot possibly be forwarded.
	 */
	return data != NULL;
}

/*
 * Routines to acknowledge, disable and enable interrupts
 */
static void gic_poke_irq(struct irq_data *d, u32 offset)
{
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);
	u8 val8;
	u32 mask = 1 << (gic_irq(d) % 32);
	u8 curr_cpu;
	u32 irq_target = GIC_DIST_TARGET + gic_irq(d);

#ifdef CONFIG_TEGRA_APE_AGIC
	if (!gic->is_percpu && tegra_agic_suspended)
		return;
#endif
	curr_cpu = gic_get_cpumask(gic);

	raw_spin_lock(&irq_controller_lock);
	/*
	 * if it is not per-cpu then we should make sure the irq has
	 * been routed to CPU.
	 */
	val8 = readb_relaxed(gic_dist_base(d) + irq_target);
	if (!gic->is_percpu && !(val8 & curr_cpu))
		goto end;

	writel_relaxed(mask, gic_dist_base(d) + offset + (gic_irq(d) / 32) * 4);

end:
	raw_spin_unlock(&irq_controller_lock);
}

static int gic_peek_irq(struct irq_data *d, u32 offset)
{
	u32 mask = 1 << (gic_irq(d) % 32);
	return !!(readl_relaxed(gic_dist_base(d) + offset + (gic_irq(d) / 32) * 4) & mask);
}

static void gic_mask_irq(struct irq_data *d)
{
	gic_poke_irq(d, GIC_DIST_ENABLE_CLEAR);
}

static void gic_eoimode1_mask_irq(struct irq_data *d)
{
	gic_mask_irq(d);
	/*
	 * When masking a forwarded interrupt, make sure it is
	 * deactivated as well.
	 *
	 * This ensures that an interrupt that is getting
	 * disabled/masked will not get "stuck", because there is
	 * noone to deactivate it (guest is being terminated).
	 */
	if (irqd_is_forwarded_to_vcpu(d))
		gic_poke_irq(d, GIC_DIST_ACTIVE_CLEAR);
}

static void gic_unmask_irq(struct irq_data *d)
{
	gic_poke_irq(d, GIC_DIST_ENABLE_SET);
}

static inline void gic_irq_enable(struct irq_data *d)
{
#ifdef CONFIG_TEGRA_APE_AGIC
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);

	if (!gic->is_percpu && tegra_agic_suspended)
		return;
#endif

	gic_unmask_irq(d);
}

static inline void gic_irq_disable(struct irq_data *d)
{
#ifdef CONFIG_TEGRA_APE_AGIC
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);

	if (!gic->is_percpu && tegra_agic_suspended)
		return;
#endif
	gic_mask_irq(d);
}

static void gic_eoi_irq(struct irq_data *d)
{
	writel_relaxed(gic_irq(d), gic_cpu_base(d) + GIC_CPU_EOI);
}

static void gic_eoimode1_eoi_irq(struct irq_data *d)
{
	/* Do not deactivate an IRQ forwarded to a vcpu. */
	if (irqd_is_forwarded_to_vcpu(d))
		return;

	writel_relaxed(gic_irq(d), gic_cpu_base(d) + GIC_CPU_DEACTIVATE);
}

static int gic_irq_set_irqchip_state(struct irq_data *d,
				     enum irqchip_irq_state which, bool val)
{
	u32 reg;

	switch (which) {
	case IRQCHIP_STATE_PENDING:
		reg = val ? GIC_DIST_PENDING_SET : GIC_DIST_PENDING_CLEAR;
		break;

	case IRQCHIP_STATE_ACTIVE:
		reg = val ? GIC_DIST_ACTIVE_SET : GIC_DIST_ACTIVE_CLEAR;
		break;

	case IRQCHIP_STATE_MASKED:
		reg = val ? GIC_DIST_ENABLE_CLEAR : GIC_DIST_ENABLE_SET;
		break;

	default:
		return -EINVAL;
	}

	gic_poke_irq(d, reg);
	return 0;
}

static int gic_irq_get_irqchip_state(struct irq_data *d,
				      enum irqchip_irq_state which, bool *val)
{
	switch (which) {
	case IRQCHIP_STATE_PENDING:
		*val = gic_peek_irq(d, GIC_DIST_PENDING_SET);
		break;

	case IRQCHIP_STATE_ACTIVE:
		*val = gic_peek_irq(d, GIC_DIST_ACTIVE_SET);
		break;

	case IRQCHIP_STATE_MASKED:
		*val = !gic_peek_irq(d, GIC_DIST_ENABLE_SET);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int gic_set_type(struct irq_data *d, unsigned int type)
{
	void __iomem *base = gic_dist_base(d);
	unsigned int gicirq = gic_irq(d);

	/* Interrupt configuration for SGIs can't be changed */
	if (gicirq < 16)
		return -EINVAL;

	/* SPIs have restrictions on the supported types */
	if (gicirq >= 32 && type != IRQ_TYPE_LEVEL_HIGH &&
			    type != IRQ_TYPE_EDGE_RISING)
		return -EINVAL;

	return gic_configure_irq(gicirq, type, base, NULL);
}

static int gic_irq_set_vcpu_affinity(struct irq_data *d, void *vcpu)
{
	/* Only interrupts on the primary GIC can be forwarded to a vcpu. */
	if (cascading_gic_irq(d))
		return -EINVAL;

	if (vcpu)
		irqd_set_forwarded_to_vcpu(d);
	else
		irqd_clr_forwarded_to_vcpu(d);
	return 0;
}

#ifdef CONFIG_SMP
static int gic_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
			    bool force)
{
	void __iomem *reg = gic_dist_base(d) + GIC_DIST_TARGET + (gic_irq(d) & ~3);
	unsigned int cpu, shift = (gic_irq(d) % 4) * 8;
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);
	u32 val, mask, bit;
	unsigned long flags;

	if (!force)
		cpu = cpumask_any_and(mask_val, cpu_online_mask);
	else
		cpu = cpumask_first(mask_val);

	if (cpu >= NR_GIC_CPU_IF || cpu >= nr_cpu_ids)
		return -EINVAL;

	/* do not set affinity to gic's which are not per cpu*/
	if (!gic->is_percpu)
		goto end;
	raw_spin_lock_irqsave(&irq_controller_lock, flags);
	mask = 0xff << shift;
	bit = gic_cpu_map[cpu] << shift;
	val = readl_relaxed(reg) & ~mask;
	writel_relaxed(val | bit, reg);
	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);
end:
	return IRQ_SET_MASK_OK;
}
#endif

static void __exception_irq_entry gic_handle_irq(struct pt_regs *regs)
{
	u32 irqstat, irqnr;
	struct gic_chip_data *gic = &gic_data[0];
	void __iomem *cpu_base = gic_data_cpu_base(gic);

	do {
		irqstat = readl_relaxed(cpu_base + GIC_CPU_INTACK);
		irqnr = irqstat & GICC_IAR_INT_ID_MASK;

		if (likely(irqnr > 15 && irqnr < 1021)) {
			if (static_key_true(&supports_deactivate))
				writel_relaxed(irqstat, cpu_base + GIC_CPU_EOI);
			handle_domain_irq(gic->domain, irqnr, regs);
			continue;
		}
		if (irqnr < 16) {
			writel_relaxed(irqstat, cpu_base + GIC_CPU_EOI);
			if (static_key_true(&supports_deactivate))
				writel_relaxed(irqstat, cpu_base + GIC_CPU_DEACTIVATE);
#ifdef CONFIG_SMP
			/*
			 * Ensure any shared data written by the CPU sending
			 * the IPI is read after we've read the ACK register
			 * on the GIC.
			 *
			 * Pairs with the write barrier in gic_raise_softirq
			 */
			smp_rmb();
			handle_IPI(irqnr, regs);
#endif
			continue;
		}
		break;
	} while (1);
}

static void gic_handle_cascade_irq(struct irq_desc *desc)
{
	struct gic_chip_data *chip_data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int cascade_irq, gic_irq;
	unsigned long status;

	chained_irq_enter(chip, desc);

	raw_spin_lock(&irq_controller_lock);
	status = readl_relaxed(gic_data_cpu_base(chip_data) + GIC_CPU_INTACK);
	raw_spin_unlock(&irq_controller_lock);

	gic_irq = (status & GICC_IAR_INT_ID_MASK);
	if (gic_irq == GICC_INT_SPURIOUS)
		goto out;

	cascade_irq = irq_find_mapping(chip_data->domain, gic_irq);
	if (unlikely(gic_irq < 32 || gic_irq > 1020))
		handle_bad_irq(desc);
	else
		generic_handle_irq(cascade_irq);

 out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip gic_chip = {
	.name			= "GIC",
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
	.irq_get_irqchip_state	= gic_irq_get_irqchip_state,
	.irq_set_irqchip_state	= gic_irq_set_irqchip_state,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

static struct irq_chip gic_eoimode1_chip = {
	.name			= "GICv2",
	.irq_mask		= gic_eoimode1_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoimode1_eoi_irq,
	.irq_set_type		= gic_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
	.irq_get_irqchip_state	= gic_irq_get_irqchip_state,
	.irq_set_irqchip_state	= gic_irq_set_irqchip_state,
	.irq_set_vcpu_affinity	= gic_irq_set_vcpu_affinity,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

#ifdef CONFIG_FIQ
/*
 * Shift an interrupt between Group 0 and Group 1.
 *
 * In addition to changing the group we also modify the priority to
 * match what "ARM strongly recommends" for a system where no Group 1
 * interrupt must ever preempt a Group 0 interrupt.
 */
static void gic_set_group_irq(struct irq_data *d, int group)
{
	unsigned int grp_reg = gic_irq(d) / 32 * 4;
	u32 grp_mask = 1 << (gic_irq(d) % 32);
	u32 grp_val;

	unsigned int pri_reg = (gic_irq(d) / 4) * 4;
	u32 pri_mask = 1 << (7 + ((gic_irq(d) % 4) * 8));
	u32 pri_val;

	raw_spin_lock(&irq_controller_lock);

	grp_val = readl_relaxed(gic_dist_base(d) + GIC_DIST_IGROUP + grp_reg);
	pri_val = readl_relaxed(gic_dist_base(d) + GIC_DIST_PRI + pri_reg);

	if (group) {
		grp_val |= grp_mask;
		pri_val |= pri_mask;
	} else {
		grp_val &= ~grp_mask;
		pri_val &= ~pri_mask;
	}

	writel_relaxed(grp_val, gic_dist_base(d) + GIC_DIST_IGROUP + grp_reg);
	writel_relaxed(pri_val, gic_dist_base(d) + GIC_DIST_PRI + pri_reg);

	raw_spin_unlock(&irq_controller_lock);
}

static void gic_enable_fiq(struct irq_data *d)
{
	gic_set_group_irq(d, 0);
}

static void gic_disable_fiq(struct irq_data *d)
{
	gic_set_group_irq(d, 1);
}

static int gic_ack_fiq(struct irq_data *d)
{
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);
	u32 irqstat, irqnr;

	irqstat = readl_relaxed(gic_data_cpu_base(gic) + GIC_CPU_INTACK);
	irqnr = irqstat & GICC_IAR_INT_ID_MASK;
	return irq_find_mapping(gic->domain, irqnr);
}

static struct fiq_chip gic_fiq = {
	.fiq_enable		= gic_enable_fiq,
	.fiq_disable            = gic_disable_fiq,
	.fiq_ack		= gic_ack_fiq,
	.fiq_eoi		= gic_eoi_irq,
};

static void __init gic_init_fiq(struct gic_chip_data *gic,
				irq_hw_number_t first_irq,
				unsigned int num_irqs)
{
	void __iomem *dist_base = gic_data_dist_base(gic);
	int i;

	/*
	 * If grouping is not available (not implemented or prohibited by
	 * security mode) these registers a read-as-zero/write-ignored.
	 * However as a precaution we restore the reset default regardless of
	 * the result of the test.
	 */
	writel_relaxed(1, dist_base + GIC_DIST_IGROUP + 0);
	gic->fiq_enable = readl_relaxed(dist_base + GIC_DIST_IGROUP + 0);
	writel_relaxed(0, dist_base + GIC_DIST_IGROUP + 0);
	pr_debug("gic: FIQ support %s\n",
		 gic->fiq_enable ? "enabled" : "disabled");

	if (!gic->fiq_enable)
		return;
	/*
	 * FIQ is supported on this device! Register our chip data.
	 */
	for (i = 0; i < num_irqs; i++)
		fiq_register_mapping(first_irq + i, &gic_fiq);
}
#else /* CONFIG_FIQ */
static inline void gic_init_fiq(struct gic_chip_data *gic,
				irq_hw_number_t first_irq,
				unsigned int num_irqs)
{
	/* empty */
}
#endif /* CONFIG_FIQ */

void __init gic_cascade_irq(unsigned int gic_nr, unsigned int irq)
{
	if (gic_nr >= MAX_GIC_NR)
		BUG();
	irq_set_chained_handler_and_data(irq, gic_handle_cascade_irq,
					 &gic_data[gic_nr]);
}

static u8 gic_get_cpumask(struct gic_chip_data *gic)
{
	void __iomem *base = gic_data_dist_base(gic);
	u32 mask, i;

	for (i = mask = 0; i < 32; i += 4) {
		mask = readl_relaxed(base + GIC_DIST_TARGET + i);
		mask |= mask >> 16;
		mask |= mask >> 8;
		if (mask)
			break;
	}

	if (!mask && num_possible_cpus() > 1)
		pr_crit("GIC CPU mask not found - kernel will fail to boot.\n");

	return mask;
}

static void gic_cpu_if_up(struct gic_chip_data *gic)
{
	void __iomem *cpu_base = gic_data_cpu_base(gic);
	u32 bypass = 0;
	u32 mode = 0;

	if (gic == &gic_data[0] && static_key_true(&supports_deactivate))
		mode = GIC_CPU_CTRL_EOImodeNS;

	/*
	* Preserve bypass disable bits to be written back later
	*/
	bypass = readl(cpu_base + GIC_CPU_CTRL);
	bypass &= GICC_DIS_BYPASS_MASK;

	if (gic_data_fiq_enable(gic))
		bypass |= 0x1f;

	writel_relaxed(bypass | mode | GICC_ENABLE, cpu_base + GIC_CPU_CTRL);
}


static void __init gic_dist_init(struct gic_chip_data *gic)
{
	unsigned int i;
	u32 cpumask;
	unsigned int gic_irqs = gic->gic_irqs;
	void __iomem *base = gic_data_dist_base(gic);

	writel_relaxed(GICD_DISABLE, base + GIC_DIST_CTRL);

	/*
	 * Set all global interrupts to this CPU only.
	 */
	cpumask = gic_get_cpumask(gic);
	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;
	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);

	gic_dist_config(base, gic_irqs, NULL);

	/*
	 * Optionally set all global interrupts to be group 1.
	 */
	if (gic_data_fiq_enable(gic))
		for (i = 32; i < gic_irqs; i += 32)
			writel_relaxed(0xffffffff,
			       base + GIC_DIST_IGROUP + i * 4 / 32);

	/*
	 * Set EnableGrp1/EnableGrp0 (bit 1 and 0) or EnableGrp (bit 0 only,
	 * bit 1 ignored)
	 */
	if (gic_data_fiq_enable(gic))
		writel_relaxed(3, base + GIC_DIST_CTRL);
	else
		writel_relaxed(1, base + GIC_DIST_CTRL);
}

static void gic_cpu_init(struct gic_chip_data *gic)
{
	void __iomem *dist_base = gic_data_dist_base(gic);
	void __iomem *base = gic_data_cpu_base(gic);
	unsigned int cpu_mask, cpu = smp_processor_id();
	int i;

	/*
	 * Setting up the CPU map is only relevant for the primary GIC
	 * because any nested/secondary GICs do not directly interface
	 * with the CPU(s).
	 */
	if (gic == &gic_data[0]) {
		/*
		 * Get what the GIC says our CPU mask is.
		 */
		BUG_ON(cpu >= NR_GIC_CPU_IF);
		cpu_mask = gic_get_cpumask(gic);
		gic_cpu_map[cpu] = cpu_mask;

		/*
		 * Clear our mask from the other map entries in case they're
		 * still undefined.
		 */
		for (i = 0; i < NR_GIC_CPU_IF; i++)
			if (i != cpu)
				gic_cpu_map[i] &= ~cpu_mask;
	}

	gic_cpu_config(dist_base, NULL);

	/*
	 * Set all PPI and SGI interrupts to be group 1.
	 *
	 * If grouping is not available (not implemented or prohibited by
	 * security mode) these registers are read-as-zero/write-ignored.
	 */
	if (gic_data_fiq_enable(gic))
		writel_relaxed(0xffffffff, dist_base + GIC_DIST_IGROUP + 0);

	writel_relaxed(GICC_INT_PRI_THRESHOLD, base + GIC_CPU_PRIMASK);
	gic_cpu_if_up(gic);
}

int gic_cpu_if_down(unsigned int gic_nr)
{
	void __iomem *cpu_base;
	u32 val = 0;

	if (gic_nr >= MAX_GIC_NR)
		return -EINVAL;

	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);
	val = readl(cpu_base + GIC_CPU_CTRL);
	val &= ~GICC_ENABLE;
	writel_relaxed(val, cpu_base + GIC_CPU_CTRL);

	return 0;
}

/*
 * Saves the GIC distributor registers during suspend or idle.  Must be called
 * with interrupts disabled but before powering down the GIC.  After calling
 * this function, no interrupts will be delivered by the GIC, and another
 * platform-specific wakeup source must be enabled.
 */
static void gic_dist_save(struct gic_chip_data *gic)
{
	unsigned int gic_irqs;
	void __iomem *dist_base;
	int i;

	gic_irqs = gic->gic_irqs;
	dist_base = gic_data_dist_base(gic);

	if (!dist_base)
		return;

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		gic->saved_spi_conf[i] =
			readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		gic->saved_spi_target[i] =
			readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic->saved_spi_enable[i] =
			readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic_data->saved_spi_active[i] =
			readl_relaxed(dist_base + GIC_DIST_ACTIVE_SET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic->saved_spi_group[i] =
			readl_relaxed(dist_base + GIC_DIST_IGROUP + i * 4);

}

/*
 * Restores the GIC distributor registers during resume or when coming out of
 * idle.  Must be called before enabling interrupts.  If a level interrupt
 * that occured while the GIC was suspended is still present, it will be
 * handled normally, but any edge interrupts that occured will not be seen by
 * the GIC and need to be handled by the platform-specific wakeup source.
 */
static void gic_dist_restore(struct gic_chip_data *gic)
{
	unsigned int gic_irqs;
	unsigned int i;
	void __iomem *dist_base;

	gic_irqs = gic->gic_irqs;
	dist_base = gic_data_dist_base(gic);

	if (!dist_base)
		return;

	writel_relaxed(GICD_DISABLE, dist_base + GIC_DIST_CTRL);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		writel_relaxed(gic->saved_spi_conf[i],
			dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(GICD_INT_DEF_PRI_X4,
			dist_base + GIC_DIST_PRI + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(gic->saved_spi_target[i],
			dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++) {
		writel_relaxed(GICD_INT_EN_CLR_X32,
			dist_base + GIC_DIST_ENABLE_CLEAR + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		writel_relaxed(gic->saved_spi_group[i],
			dist_base + GIC_DIST_IGROUP + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		writel_relaxed(gic->saved_spi_enable[i],
			dist_base + GIC_DIST_ENABLE_SET + i * 4);
	}

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++) {
		writel_relaxed(GICD_INT_EN_CLR_X32,
			dist_base + GIC_DIST_ACTIVE_CLEAR + i * 4);
		writel_relaxed(gic->saved_spi_active[i],
			dist_base + GIC_DIST_ACTIVE_SET + i * 4);
	}

	if (gic_data_fiq_enable(gic))
		writel_relaxed(3, dist_base + GIC_DIST_CTRL);
	else
		writel_relaxed(1, dist_base + GIC_DIST_CTRL);
}

static void gic_cpu_save(struct gic_chip_data *gic)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	dist_base = gic_data_dist_base(gic);
	cpu_base = gic_data_cpu_base(gic);

	if (!dist_base || !cpu_base)
		return;

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_enable);
	else
		ptr = per_cpu_ptr(gic->saved_ppi_enable, 0);

	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_active);
	else
		ptr = per_cpu_ptr(gic->saved_ppi_active, 0);
		
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_ACTIVE_SET + i * 4);

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_conf);
	else
		ptr = per_cpu_ptr(gic->saved_ppi_conf, 0);

	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

}

static void gic_cpu_restore(struct gic_chip_data *gic)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	dist_base = gic_data_dist_base(gic);
	cpu_base = gic_data_cpu_base(gic);

	if (!dist_base || !cpu_base)
		return;

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_enable);
	else
		ptr = per_cpu_ptr(gic->saved_ppi_enable, 0);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++) {
		writel_relaxed(GICD_INT_EN_CLR_X32,
			       dist_base + GIC_DIST_ENABLE_CLEAR + i * 4);
		writel_relaxed(ptr[i], dist_base + GIC_DIST_ENABLE_SET + i * 4);
	}

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_active);
        else
		ptr = per_cpu_ptr(gic->saved_ppi_active, 0);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++) {
		writel_relaxed(GICD_INT_EN_CLR_X32,
			       dist_base + GIC_DIST_ACTIVE_CLEAR + i * 4);
		writel_relaxed(ptr[i], dist_base + GIC_DIST_ACTIVE_SET + i * 4);
	}

	if (gic->is_percpu)
		ptr = raw_cpu_ptr(gic->saved_ppi_conf);
	else
		ptr = per_cpu_ptr(gic->saved_ppi_conf, 0);

	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		writel_relaxed(ptr[i], dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(32, 4); i++)
		writel_relaxed(GICD_INT_DEF_PRI_X4,
					dist_base + GIC_DIST_PRI + i * 4);

	writel_relaxed(GICC_INT_PRI_THRESHOLD, cpu_base + GIC_CPU_PRIMASK);
	gic_cpu_if_up(gic);
}

static int gic_notifier(struct notifier_block *self, unsigned long cmd,	void *v)
{
	struct gic_chip_data *gic =
		container_of(self, struct gic_chip_data, pm_notifier_block);
#ifdef CONFIG_GIC_NON_BANKED
		/* Skip over unused GICs */
		if (!gic->get_base)
			continue;
#endif
	if (gic->is_percpu) {
			switch (cmd) {
			case CPU_PM_ENTER:
				gic_cpu_save(gic);
				break;
			case CPU_PM_ENTER_FAILED:
			case CPU_PM_EXIT:
				gic_cpu_restore(gic);
				break;
			case CPU_CLUSTER_PM_ENTER:
				gic_dist_save(gic);
				break;
			case CPU_CLUSTER_PM_ENTER_FAILED:
			case CPU_CLUSTER_PM_EXIT:
				gic_dist_restore(gic);
				break;
			}
		} else {
			switch (cmd) {
			case MOD_DOMAIN_POWER_ON:
				gic_dist_restore(gic);
				gic_cpu_restore(gic);
#ifdef CONFIG_TEGRA_APE_AGIC
				tegra_agic_suspended = false;
#endif
				break;
			case MOD_DOMAIN_POWER_OFF:
				gic_cpu_save(gic);
				gic_dist_save(gic);
#ifdef CONFIG_TEGRA_APE_AGIC
				tegra_agic_suspended = true;
#endif
				break;
			}
		}
	return NOTIFY_OK;
}

static void __init gic_pm_init(struct gic_chip_data *gic)
{
	gic->saved_ppi_enable = __alloc_percpu(DIV_ROUND_UP(32, 32) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_enable);

	gic->saved_ppi_active = __alloc_percpu(DIV_ROUND_UP(32, 32) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_active);

	gic->saved_ppi_conf = __alloc_percpu(DIV_ROUND_UP(32, 16) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_conf);

	gic->pm_notifier_block.notifier_call = gic_notifier;

#ifdef CONFIG_CPU_PM
	if (gic->is_percpu)
		cpu_pm_register_notifier(&gic->pm_notifier_block);
#endif
}

#ifdef CONFIG_SMP
static void gic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	int cpu;
	unsigned long flags, map = 0;
	unsigned long softint;

	raw_spin_lock_irqsave(&irq_controller_lock, flags);

	/* Convert our logical CPU mask into a physical one. */
	for_each_cpu(cpu, mask)
		map |= gic_cpu_map[cpu];

	/*
	 * Ensure that stores to Normal memory are visible to the
	 * other CPUs before they observe us issuing the IPI.
	 */
	dmb(ishst);

	/* this always happens on GIC0 */
	writel_relaxed(map << 16 | irq, gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);

	softint = map << 16 | irq;
	if (gic_data_fiq_enable(&gic_data[0]))
		softint |= 0x8000;
	writel_relaxed(softint,
		       gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);

	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);
}
#endif

#ifdef CONFIG_BL_SWITCHER
/*
 * gic_send_sgi - send a SGI directly to given CPU interface number
 *
 * cpu_id: the ID for the destination CPU interface
 * irq: the IPI number to send a SGI for
 */
void gic_send_sgi(unsigned int cpu_id, unsigned int irq)
{
	BUG_ON(cpu_id >= NR_GIC_CPU_IF);
	cpu_id = 1 << cpu_id;
	/* this always happens on GIC0 */
	writel_relaxed((cpu_id << 16) | irq, gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);
}

/*
 * gic_get_cpu_id - get the CPU interface ID for the specified CPU
 *
 * @cpu: the logical CPU number to get the GIC ID for.
 *
 * Return the CPU interface ID for the given logical CPU number,
 * or -1 if the CPU number is too large or the interface ID is
 * unknown (more than one bit set).
 */
int gic_get_cpu_id(unsigned int cpu)
{
	unsigned int cpu_bit;

	if (cpu >= NR_GIC_CPU_IF)
		return -1;
	cpu_bit = gic_cpu_map[cpu];
	if (cpu_bit & (cpu_bit - 1))
		return -1;
	return __ffs(cpu_bit);
}

/*
 * gic_migrate_target - migrate IRQs to another CPU interface
 *
 * @new_cpu_id: the CPU target ID to migrate IRQs to
 *
 * Migrate all peripheral interrupts with a target matching the current CPU
 * to the interface corresponding to @new_cpu_id.  The CPU interface mapping
 * is also updated.  Targets to other CPU interfaces are unchanged.
 * This must be called with IRQs locally disabled.
 */
void gic_migrate_target(unsigned int new_cpu_id)
{
	unsigned int cur_cpu_id, gic_irqs, gic_nr = 0;
	void __iomem *dist_base;
	int i, ror_val, cpu = smp_processor_id();
	u32 val, cur_target_mask, active_mask;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	if (!dist_base)
		return;
	gic_irqs = gic_data[gic_nr].gic_irqs;

	cur_cpu_id = __ffs(gic_cpu_map[cpu]);
	cur_target_mask = 0x01010101 << cur_cpu_id;
	ror_val = (cur_cpu_id - new_cpu_id) & 31;

	raw_spin_lock(&irq_controller_lock);

	/* Update the target interface for this logical CPU */
	gic_cpu_map[cpu] = 1 << new_cpu_id;

	/*
	 * Find all the peripheral interrupts targetting the current
	 * CPU interface and migrate them to the new CPU interface.
	 * We skip DIST_TARGET 0 to 7 as they are read-only.
	 */
	for (i = 8; i < DIV_ROUND_UP(gic_irqs, 4); i++) {
		val = readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);
		active_mask = val & cur_target_mask;
		if (active_mask) {
			val &= ~active_mask;
			val |= ror32(active_mask, ror_val);
			writel_relaxed(val, dist_base + GIC_DIST_TARGET + i*4);
		}
	}

	raw_spin_unlock(&irq_controller_lock);

	/*
	 * Now let's migrate and clear any potential SGIs that might be
	 * pending for us (cur_cpu_id).  Since GIC_DIST_SGI_PENDING_SET
	 * is a banked register, we can only forward the SGI using
	 * GIC_DIST_SOFTINT.  The original SGI source is lost but Linux
	 * doesn't use that information anyway.
	 *
	 * For the same reason we do not adjust SGI source information
	 * for previously sent SGIs by us to other CPUs either.
	 */
	for (i = 0; i < 16; i += 4) {
		int j;
		val = readl_relaxed(dist_base + GIC_DIST_SGI_PENDING_SET + i);
		if (!val)
			continue;
		writel_relaxed(val, dist_base + GIC_DIST_SGI_PENDING_CLEAR + i);
		for (j = i; j < i + 4; j++) {
			if (val & 0xff)
				writel_relaxed((1 << (new_cpu_id + 16)) | j,
						dist_base + GIC_DIST_SOFTINT);
			val >>= 8;
		}
	}
}

/*
 * gic_get_sgir_physaddr - get the physical address for the SGI register
 *
 * REturn the physical address of the SGI register to be used
 * by some early assembly code when the kernel is not yet available.
 */
static unsigned long gic_dist_physaddr;

unsigned long gic_get_sgir_physaddr(void)
{
	if (!gic_dist_physaddr)
		return 0;
	return gic_dist_physaddr + GIC_DIST_SOFTINT;
}

void __init gic_init_physaddr(struct device_node *node)
{
	struct resource res;
	if (of_address_to_resource(node, 0, &res) == 0) {
		gic_dist_physaddr = res.start;
		pr_info("GIC physical location is %#lx\n", gic_dist_physaddr);
	}
}

#else
#define gic_init_physaddr(node)  do { } while (0)
#endif

static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	struct irq_chip *chip = &gic_chip;

	if (static_key_true(&supports_deactivate)) {
		if (d->host_data == (void *)&gic_data[0])
			chip = &gic_eoimode1_chip;
	}

	if (hw < 32) {
		irq_set_percpu_devid(irq);
		irq_domain_set_info(d, irq, hw, chip, d->host_data,
				    handle_percpu_devid_irq, NULL, NULL);
		irq_set_status_flags(irq, IRQ_NOAUTOEN);
	} else {
		irq_domain_set_info(d, irq, hw, chip, d->host_data,
				    handle_fasteoi_irq, NULL, NULL);
		irq_set_probe(irq);
	}
	return 0;
}

static void gic_irq_domain_unmap(struct irq_domain *d, unsigned int irq)
{
}

static int gic_irq_domain_translate(struct irq_domain *d,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	struct gic_chip_data *gic = (struct gic_chip_data *)d->host_data;

	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count < 3)
			return -EINVAL;

		/* Get the interrupt number and add 16 to skip over SGIs */
		*hwirq = fwspec->param[1] + 16;

		/*
		 * For SPIs, we need to add 16 more to get the GIC irq
		 * ID number
		 */
		if (!fwspec->param[0])
			*hwirq += 16;

		*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;

#ifdef CONFIG_TEGRA_APE_AGIC
	if ((gic->is_agic)
		&& (fwspec->param_count == 4) && (fwspec->param[3] < gic->num_interfaces))
		return tegra_agic_route_interrupt(*hwirq, fwspec->param[3]);
	return 0;
#else
		return 0;
#endif
	}

	if (fwspec->fwnode->type == FWNODE_IRQCHIP) {
		if(fwspec->param_count != 2)
			return -EINVAL;

		*hwirq = fwspec->param[0];
		*type = fwspec->param[1];
		return 0;
	}

	return -EINVAL;
}

#ifdef CONFIG_SMP
static int gic_secondary_init(struct notifier_block *nfb, unsigned long action,
			      void *hcpu)
{
	if (action == CPU_STARTING || action == CPU_STARTING_FROZEN)
		gic_cpu_init(&gic_data[0]);
	return NOTIFY_OK;
}

/*
 * Notifier for enabling the GIC CPU interface. Set an arbitrarily high
 * priority because the GIC needs to be up before the ARM generic timers.
 */
static struct notifier_block gic_cpu_notifier = {
	.notifier_call = gic_secondary_init,
	.priority = 100,
};
#endif

static int gic_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	int i, ret;
	irq_hw_number_t hwirq;
	unsigned int type = IRQ_TYPE_NONE;
	struct irq_fwspec *fwspec = arg;

	ret = gic_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++)
		gic_irq_domain_map(domain, virq + i, hwirq + i);

	return 0;
}

static const struct irq_domain_ops gic_irq_domain_hierarchy_ops = {
	.translate = gic_irq_domain_translate,
	.alloc = gic_irq_domain_alloc,
	.free = irq_domain_free_irqs_top,
};

static const struct irq_domain_ops gic_irq_domain_ops = {
	.map = gic_irq_domain_map,
	.unmap = gic_irq_domain_unmap,
};

static void __init __gic_init_bases(unsigned int gic_nr, int irq_start,
			   void __iomem *dist_base, void __iomem *cpu_base,
			   u32 percpu_offset, bool is_percpu,
			   struct fwnode_handle *handle)
{
	irq_hw_number_t hwirq_base;
	struct gic_chip_data *gic;
	int gic_irqs, irq_base, i;

	BUG_ON(gic_nr >= MAX_GIC_NR);

	gic_check_cpu_features();

	gic = &gic_data[gic_nr];

	gic->is_percpu = is_percpu;

#ifdef CONFIG_GIC_NON_BANKED
	if (percpu_offset) { /* Frankein-GIC without banked registers... */
		unsigned int cpu;

		gic->dist_base.percpu_base = alloc_percpu(void __iomem *);
		gic->cpu_base.percpu_base = alloc_percpu(void __iomem *);
		if (WARN_ON(!gic->dist_base.percpu_base ||
			    !gic->cpu_base.percpu_base)) {
			free_percpu(gic->dist_base.percpu_base);
			free_percpu(gic->cpu_base.percpu_base);
			return;
		}

		for_each_possible_cpu(cpu) {
			u32 mpidr = cpu_logical_map(cpu);
			u32 core_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			unsigned long offset = percpu_offset * core_id;
			*per_cpu_ptr(gic->dist_base.percpu_base, cpu) = dist_base + offset;
			*per_cpu_ptr(gic->cpu_base.percpu_base, cpu) = cpu_base + offset;
		}

		gic_set_base_accessor(gic, gic_get_percpu_base);
	} else
#endif
	{			/* Normal, sane GIC... */
		WARN(percpu_offset,
		     "GIC_NON_BANKED not enabled, ignoring %08x offset!",
		     percpu_offset);
		gic->dist_base.common_base = dist_base;
		gic->cpu_base.common_base = cpu_base;
		gic_set_base_accessor(gic, gic_get_common_base);
	}

	/*
	 * Find out how many interrupts are supported.
	 * The GIC only supports up to 1020 interrupt sources.
	 */
	gic_irqs = readl_relaxed(gic_data_dist_base(gic) + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	gic->gic_irqs = gic_irqs;

	if (handle) {		/* DT/ACPI */
		gic->domain = irq_domain_create_linear(handle, gic_irqs,
						       &gic_irq_domain_hierarchy_ops,
						       gic);
	} else {		/* Legacy support */
		/*
		 * For primary GICs, skip over SGIs.
		 * For secondary GICs, skip over PPIs, too.
		 */
		if (gic_nr == 0 && (irq_start & 31) > 0) {
			hwirq_base = 16;
			if (irq_start != -1)
				irq_start = (irq_start & ~31) + 16;
		} else {
			hwirq_base = 32;
		}

		gic_irqs -= hwirq_base; /* calculate # of irqs to allocate */

		irq_base = irq_alloc_descs(irq_start, 16, gic_irqs,
					   numa_node_id());
		if (IS_ERR_VALUE(irq_base)) {
			WARN(1, "Cannot allocate irq_descs @ IRQ%d, assuming pre-allocated\n",
			     irq_start);
			irq_base = irq_start;
		}

		gic->domain = irq_domain_add_legacy(NULL, gic_irqs, irq_base,
					hwirq_base, &gic_irq_domain_ops, gic);
	}
	gic_init_fiq(gic, irq_base, gic_irqs);

	if (WARN_ON(!gic->domain))
		return;

	if (gic_nr == 0) {
		/*
		 * Initialize the CPU interface map to all CPUs.
		 * It will be refined as each CPU probes its ID.
		 * This is only necessary for the primary GIC.
		 */
		for (i = 0; i < NR_GIC_CPU_IF; i++)
			gic_cpu_map[i] = 0xff;
#ifdef CONFIG_SMP
		if (gic->is_percpu) {
			set_smp_cross_call(gic_raise_softirq);
			register_cpu_notifier(&gic_cpu_notifier);
		}
#endif
		set_handle_irq(gic_handle_irq);
		if (static_key_true(&supports_deactivate))
			pr_info("GIC: Using split EOI/Deactivate mode\n");
	}

#ifdef CONFIG_TEGRA_APE_AGIC
	/*
	 * need to disable/enable the interrupt on hardware when
	 * disable_irq/enable_irq API is being called.
	 */
	if (!gic->is_percpu) {
		gic_chip.irq_enable = gic_irq_enable;
		gic_chip.irq_disable = gic_irq_disable;
	}
#endif

	gic_dist_init(gic);
	gic_cpu_init(gic);
	gic_pm_init(gic);
}

void __init gic_init(unsigned int gic_nr, int irq_start,
		     void __iomem *dist_base, void __iomem *cpu_base)
{
	/*
	 * Non-DT/ACPI systems won't run a hypervisor, so let's not
	 * bother with these...
	 */
	static_key_slow_dec(&supports_deactivate);
	__gic_init_bases(gic_nr, irq_start, dist_base, cpu_base, 0, false,
			NULL);
}

#ifdef CONFIG_OF
static int gic_cnt __initdata;

static bool gic_check_eoimode(struct device_node *node, void __iomem **base)
{
	struct resource cpuif_res;

	of_address_to_resource(node, 1, &cpuif_res);

	if (!is_hyp_mode_available())
		return false;
	if (resource_size(&cpuif_res) < SZ_8K)
		return false;
	if (resource_size(&cpuif_res) == SZ_128K) {
		u32 val_low, val_high;

		/*
		 * Verify that we have the first 4kB of a GIC400
		 * aliased over the first 64kB by checking the
		 * GICC_IIDR register on both ends.
		 */
		val_low = readl_relaxed(*base + GIC_CPU_IDENT);
		val_high = readl_relaxed(*base + GIC_CPU_IDENT + 0xf000);
		if ((val_low & 0xffff0fff) != 0x0202043B ||
		    val_low != val_high)
			return false;

		/*
		 * Move the base up by 60kB, so that we have a 8kB
		 * contiguous region, which allows us to use GICC_DIR
		 * at its normal offset. Please pass me that bucket.
		 */
		*base += 0xf000;
		cpuif_res.start += 0xf000;
		pr_warn("GIC: Adjusting CPU interface base to %pa",
			&cpuif_res.start);
	}

	return true;
}

static int __init
gic_of_init(struct device_node *node, struct device_node *parent)
{
	void __iomem *cpu_base;
	void __iomem *dist_base;
	u32 percpu_offset;
	bool is_percpu;
	int irq;

	if (WARN_ON(!node))
		return -ENODEV;

	dist_base = of_iomap(node, 0);
	WARN(!dist_base, "unable to map gic dist registers\n");

	cpu_base = of_iomap(node, 1);
	WARN(!cpu_base, "unable to map gic cpu registers\n");

	/*
	 * Disable split EOI/Deactivate if either HYP is not available
	 * or the CPU interface is too small.
	 */
	if (gic_cnt == 0 && !gic_check_eoimode(node, &cpu_base))
		static_key_slow_dec(&supports_deactivate);

	if (of_property_read_u32(node, "cpu-offset", &percpu_offset))
		percpu_offset = 0;

	is_percpu = !of_property_read_bool(node, "not-per-cpu");

	__gic_init_bases(gic_cnt, -1, dist_base, cpu_base,
			 percpu_offset, is_percpu,
			 &node->fwnode);
	if (!gic_cnt)
		gic_init_physaddr(node);

	if (parent) {
		irq = irq_of_parse_and_map(node, 0);
		gic_cascade_irq(gic_cnt, irq);
	}

	if (IS_ENABLED(CONFIG_ARM_GIC_V2M))
		gicv2m_of_init(node, gic_data[gic_cnt].domain);

	gic_cnt++;
	return 0;
}

#ifdef CONFIG_TEGRA_APE_AGIC
static int __init
agic_t210_of_init(struct device_node *node, struct device_node *parent)
{
	if (of_property_read_bool(node, "enable-agic-clks"))
		enable_t210_agic_clks(node);

	tegra_agic = &gic_data[gic_cnt];
	tegra_agic->is_agic = true;
	tegra_agic->num_interfaces = MAX_AGIC_T210_INTERFACES;

	return gic_of_init(node, parent);
}

static int __init
agic_t18x_of_init(struct device_node *node, struct device_node *parent)
{
	tegra_agic = &gic_data[gic_cnt];
	tegra_agic->is_agic = true;
	tegra_agic->num_interfaces = MAX_AGIC_T18x_INTERFACES;

	return gic_of_init(node, parent);
}
#endif

IRQCHIP_DECLARE(gic_400, "arm,gic-400", gic_of_init);
IRQCHIP_DECLARE(arm11mp_gic, "arm,arm11mp-gic", gic_of_init);
IRQCHIP_DECLARE(arm1176jzf_dc_gic, "arm,arm1176jzf-devchip-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a15_gic, "arm,cortex-a15-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a9_gic, "arm,cortex-a9-gic", gic_of_init);
IRQCHIP_DECLARE(cortex_a7_gic, "arm,cortex-a7-gic", gic_of_init);
IRQCHIP_DECLARE(msm_8660_qgic, "qcom,msm-8660-qgic", gic_of_init);
IRQCHIP_DECLARE(msm_qgic2, "qcom,msm-qgic2", gic_of_init);
IRQCHIP_DECLARE(pl390, "arm,pl390", gic_of_init);
#ifdef CONFIG_TEGRA_APE_AGIC
IRQCHIP_DECLARE(tegra_agic_t210, "nvidia,tegra210-agic", agic_t210_of_init);
IRQCHIP_DECLARE(tegra_agic_t18x, "nvidia,tegra18x-agic", agic_t18x_of_init);
#endif

#endif

#ifdef CONFIG_ACPI
static phys_addr_t cpu_phy_base __initdata;

static int __init
gic_acpi_parse_madt_cpu(struct acpi_subtable_header *header,
			const unsigned long end)
{
	struct acpi_madt_generic_interrupt *processor;
	phys_addr_t gic_cpu_base;
	static int cpu_base_assigned;

	processor = (struct acpi_madt_generic_interrupt *)header;

	if (BAD_MADT_GICC_ENTRY(processor, end))
		return -EINVAL;

	/*
	 * There is no support for non-banked GICv1/2 register in ACPI spec.
	 * All CPU interface addresses have to be the same.
	 */
	gic_cpu_base = processor->base_address;
	if (cpu_base_assigned && gic_cpu_base != cpu_phy_base)
		return -EINVAL;

	cpu_phy_base = gic_cpu_base;
	cpu_base_assigned = 1;
	return 0;
}

/* The things you have to do to just *count* something... */
static int __init acpi_dummy_func(struct acpi_subtable_header *header,
				  const unsigned long end)
{
	return 0;
}

static bool __init acpi_gic_redist_is_present(void)
{
	return acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR,
				     acpi_dummy_func, 0) > 0;
}

static bool __init gic_validate_dist(struct acpi_subtable_header *header,
				     struct acpi_probe_entry *ape)
{
	struct acpi_madt_generic_distributor *dist;
	dist = (struct acpi_madt_generic_distributor *)header;

	return (dist->version == ape->driver_data &&
		(dist->version != ACPI_MADT_GIC_VERSION_NONE ||
		 !acpi_gic_redist_is_present()));
}

#define ACPI_GICV2_DIST_MEM_SIZE	(SZ_4K)
#define ACPI_GIC_CPU_IF_MEM_SIZE	(SZ_8K)

static int __init gic_v2_acpi_init(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_generic_distributor *dist;
	void __iomem *cpu_base, *dist_base;
	struct fwnode_handle *domain_handle;
	int count;

	/* Collect CPU base addresses */
	count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_INTERRUPT,
				      gic_acpi_parse_madt_cpu, 0);
	if (count <= 0) {
		pr_err("No valid GICC entries exist\n");
		return -EINVAL;
	}

	cpu_base = ioremap(cpu_phy_base, ACPI_GIC_CPU_IF_MEM_SIZE);
	if (!cpu_base) {
		pr_err("Unable to map GICC registers\n");
		return -ENOMEM;
	}

	dist = (struct acpi_madt_generic_distributor *)header;
	dist_base = ioremap(dist->base_address, ACPI_GICV2_DIST_MEM_SIZE);
	if (!dist_base) {
		pr_err("Unable to map GICD registers\n");
		iounmap(cpu_base);
		return -ENOMEM;
	}

	/*
	 * Disable split EOI/Deactivate if HYP is not available. ACPI
	 * guarantees that we'll always have a GICv2, so the CPU
	 * interface will always be the right size.
	 */
	if (!is_hyp_mode_available())
		static_key_slow_dec(&supports_deactivate);

	/*
	 * Initialize GIC instance zero (no multi-GIC support).
	 */
	domain_handle = irq_domain_alloc_fwnode(dist_base);
	if (!domain_handle) {
		pr_err("Unable to allocate domain handle\n");
		iounmap(cpu_base);
		iounmap(dist_base);
		return -ENOMEM;
	}

	__gic_init_bases(0, -1, dist_base, cpu_base, 0, false, domain_handle);

	acpi_set_irq_model(ACPI_IRQ_MODEL_GIC, domain_handle);
	return 0;
}
IRQCHIP_ACPI_DECLARE(gic_v2, ACPI_MADT_TYPE_GENERIC_DISTRIBUTOR,
		     gic_validate_dist, ACPI_MADT_GIC_VERSION_V2,
		     gic_v2_acpi_init);
IRQCHIP_ACPI_DECLARE(gic_v2_maybe, ACPI_MADT_TYPE_GENERIC_DISTRIBUTOR,
		     gic_validate_dist, ACPI_MADT_GIC_VERSION_NONE,
		     gic_v2_acpi_init);
#endif
