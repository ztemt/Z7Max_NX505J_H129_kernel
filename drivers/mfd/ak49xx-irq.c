/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ak49xx/core.h>
#include <linux/mfd/ak49xx/ak496x_registers.h>
#ifdef CONFIG_AK4960_CODEC
#include <linux/mfd/ak49xx/ak4960_registers.h>
#endif
#ifdef CONFIG_AK4961_CODEC
#include <linux/mfd/ak49xx/ak4961_registers.h>
#endif
#include <linux/mfd/ak49xx/ak49xx-slimslave.h>
#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <mach/cpuidle.h>

#define BYTE_BIT_MASK(nr)		(1UL << ((nr) % BITS_PER_BYTE))
#define BIT_BYTE(nr)			((nr) / BITS_PER_BYTE)

#ifdef CONFIG_OF
struct ak49xx_irq_drv_data {
	struct irq_domain *domain;
	int irq;
};
#endif

struct ak49xx_irq {
	bool level;
};

static struct ak49xx_irq ak49xx_irqs[AK4961_NUM_IRQS] = {
//	[0] = { .level = 1},
/* All other ak49xx interrupts are edge triggered */
};

static int virq_to_phyirq(struct ak49xx *ak49xx, int virq);
static int phyirq_to_virq(struct ak49xx *ak49xx, int irq);
static unsigned int ak49xx_irq_get_upstream_irq(struct ak49xx *ak49xx);
static void ak49xx_irq_put_upstream_irq(struct ak49xx *ak49xx);
static int ak49xx_map_irq(struct ak49xx *ak49xx, int irq);

static void ak49xx_irq_lock(struct irq_data *data)
{
	struct ak49xx *ak49xx = irq_data_get_irq_chip_data(data);
	mutex_lock(&ak49xx->irq_lock);
}

static void ak49xx_irq_sync_unlock(struct irq_data *data)
{
	struct ak49xx *ak49xx = irq_data_get_irq_chip_data(data);
	mutex_unlock(&ak49xx->irq_lock);
}

static void ak49xx_irq_enable(struct irq_data *data)
{
	struct ak49xx *ak49xx = irq_data_get_irq_chip_data(data);
	int ak49xx_irq = virq_to_phyirq(ak49xx, data->irq);
	ak49xx->irq_masks_cur[BIT_BYTE(ak49xx_irq)] &=
		~(BYTE_BIT_MASK(ak49xx_irq));
}

static void ak49xx_irq_disable(struct irq_data *data)
{
	struct ak49xx *ak49xx = irq_data_get_irq_chip_data(data);
	int ak49xx_irq = virq_to_phyirq(ak49xx, data->irq);
	ak49xx->irq_masks_cur[BIT_BYTE(ak49xx_irq)]
			|= BYTE_BIT_MASK(ak49xx_irq);
}

static void ak49xx_irq_mask(struct irq_data *d)
{
	/* do nothing but required as linux calls irq_mask without NULL check */
}

static struct irq_chip ak49xx_irq_chip = {
	.name = "ak49xx",
	.irq_bus_lock = ak49xx_irq_lock,
	.irq_bus_sync_unlock = ak49xx_irq_sync_unlock,
	.irq_disable = ak49xx_irq_disable,
	.irq_enable = ak49xx_irq_enable,
	.irq_mask = ak49xx_irq_mask,
};

enum ak49xx_pm_state ak49xx_pm_cmpxchg(struct ak49xx *ak49xx,
		enum ak49xx_pm_state o,
		enum ak49xx_pm_state n)
{
	enum ak49xx_pm_state old;
	mutex_lock(&ak49xx->pm_lock);
	old = ak49xx->pm_state;
	if (old == o)
		ak49xx->pm_state = n;
	mutex_unlock(&ak49xx->pm_lock);
	return old;
}
EXPORT_SYMBOL_GPL(ak49xx_pm_cmpxchg);

static void ak49xx_irq_dispatch(struct ak49xx *ak49xx, int irqbit)
{
	handle_nested_irq(phyirq_to_virq(ak49xx, irqbit));
//	ak49xx_reg_write(ak49xx, DETECTION_EVENT_RESET, 0x01);
}

static irqreturn_t ak49xx_irq_thread(int irq, void *data)
{
	int ret;
	struct ak49xx *ak49xx = data;
	u8 status[AK49XX_NUM_IRQ_REGS];
	int i;

	ret = ak49xx_reg_read(ak49xx, JACK_DETECTION_EVENT);
	if (ret < 0) {
		dev_err(ak49xx->dev, "Failed to read interrupt status: %d\n", ret);
		return IRQ_NONE;
	} else {
		status[0] = ret;
	}
	/* Apply masking */
	for (i = 0; i < AK49XX_NUM_IRQ_REGS; i++)
		status[i] &= ~ak49xx->irq_masks_cur[i];

	/* Find out which interrupt was triggered and call that interrupt's
	 * handler function
	 */
	if (status[BIT_BYTE(AK4961_IRQ_JDE)] &
	    BYTE_BIT_MASK(AK4961_IRQ_JDE))
		ak49xx_irq_dispatch(ak49xx, AK4961_IRQ_JDE);

	if (status[BIT_BYTE(AK4961_IRQ_RCE)] &
		BYTE_BIT_MASK(AK4961_IRQ_RCE))
		ak49xx_irq_dispatch(ak49xx, AK4961_IRQ_RCE);

#ifdef CONFIG_AK4961_CODEC
	if (status[BIT_BYTE(AK4961_IRQ_VAD)] &
		BYTE_BIT_MASK(AK4961_IRQ_VAD))
		ak49xx_irq_dispatch(ak49xx, AK4961_IRQ_VAD);
#endif

	return IRQ_HANDLED;
}

void ak49xx_free_irq(struct ak49xx *ak49xx,
				int irq, void *data)
{
	free_irq(phyirq_to_virq(ak49xx, irq), data);
}
void ak49xx_enable_irq(struct ak49xx *ak49xx, int irq)
{
	enable_irq(phyirq_to_virq(ak49xx, irq));
}
void ak49xx_disable_irq(struct ak49xx *ak49xx, int irq)
{
	disable_irq_nosync(phyirq_to_virq(ak49xx, irq));
}
void ak49xx_disable_irq_sync(struct ak49xx *ak49xx, int irq)
{
	disable_irq(phyirq_to_virq(ak49xx, irq));
}

int ak49xx_irq_init(struct ak49xx *ak49xx)
{
	int ret;
	unsigned int i, virq;

	mutex_init(&ak49xx->irq_lock);

	ak49xx->irq = ak49xx_irq_get_upstream_irq(ak49xx);
	if (!ak49xx->irq) {
		pr_warn("%s: irq driver is not yet initialized\n", __func__);
		mutex_destroy(&ak49xx->irq_lock);
		return -EPROBE_DEFER;
	}
	pr_debug("%s: probed irq %d\n", __func__, ak49xx->irq);

	/* Mask the individual interrupt sources */
	for (i = 0; i < AK4961_NUM_IRQS; i++) {
		/* Map OF irq */
		virq = ak49xx_map_irq(ak49xx, i);
		pr_debug("%s: irq %d -> %d\n", __func__, i, virq);
		if (virq == NO_IRQ) {
			pr_err("%s, No interrupt specifier for irq %d\n",
				   __func__, i);
			return NO_IRQ;
		}

		ret = irq_set_chip_data(virq, ak49xx);
		if (ret) {
			pr_err("%s: Failed to configure irq %d (%d)\n",
				   __func__, i, ret);
			return ret;
		}

		if (ak49xx_irqs[i].level)
			irq_set_chip_and_handler(virq, &ak49xx_irq_chip,
					 handle_level_irq);
		else
			irq_set_chip_and_handler(virq, &ak49xx_irq_chip,
					 handle_edge_irq);

		irq_set_nested_thread(virq, 1);

		ak49xx->irq_masks_cur[BIT_BYTE(i)] |= BYTE_BIT_MASK(i);
		ak49xx->irq_masks_cache[BIT_BYTE(i)] |= BYTE_BIT_MASK(i);
		ak49xx->irq_level[BIT_BYTE(i)] |= ak49xx_irqs[i].level <<
			(i % BITS_PER_BYTE);
	}
	for (i = 0; i < AK49XX_NUM_IRQ_REGS; i++) {
		/* Initialize interrupt mask and level registers */
	}

	ret = request_threaded_irq(ak49xx->irq, NULL, ak49xx_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"ak49xx", ak49xx);
	if (ret != 0)
		dev_err(ak49xx->dev, "Failed to request IRQ %d: %d\n",
			ak49xx->irq, ret);
	else {
		ret = enable_irq_wake(ak49xx->irq);
		if (ret)
			dev_err(ak49xx->dev, "Failed to set wake interrupt on"
				" IRQ %d: %d\n", ak49xx->irq, ret);
		if (ret)
			free_irq(ak49xx->irq, ak49xx);
	}

	if (ret) {
		pr_err("%s: Failed to init ak49xx irq\n", __func__);
		ak49xx_irq_put_upstream_irq(ak49xx);
		mutex_destroy(&ak49xx->irq_lock);
	}

	return ret;
}

int ak49xx_request_irq(struct ak49xx *ak49xx, int irq, irq_handler_t handler,
			const char *name, void *data)
{
	int virq;

	virq = phyirq_to_virq(ak49xx, irq);

	pr_debug("%s: virq = %d\n", __func__, virq);
	/*
	 * ARM needs us to explicitly flag the IRQ as valid
	 * and will set them noprobe when we do so.
	 */
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	set_irq_noprobe(virq);
#endif

	return request_threaded_irq(virq, NULL, handler, IRQF_TRIGGER_RISING,
				name, data);
}

void ak49xx_irq_exit(struct ak49xx *ak49xx)
{
	if (ak49xx->irq) {
		disable_irq_wake(ak49xx->irq);
		free_irq(ak49xx->irq, ak49xx);
		/* Release parent's of node */
		ak49xx_irq_put_upstream_irq(ak49xx);
	}
	mutex_destroy(&ak49xx->irq_lock);
}

#ifndef CONFIG_OF
static int phyirq_to_virq(struct ak49xx *ak49xx, int offset)
{
	return ak49xx->irq_base + offset;
}

static int virq_to_phyirq(struct ak49xx *ak49xx, int virq)
{
	return virq - ak49xx->irq_base;
}

static unsigned int ak49xx_irq_get_upstream_irq(struct ak49xx *ak49xx)
{
	return ak49xx->irq;
}

static void ak49xx_irq_put_upstream_irq(struct ak49xx *ak49xx)
{
	/* Do nothing */
}

static int ak49xx_map_irq(struct ak49xx *ak49xx, int irq)
{
	return phyirq_to_virq(ak49xx, irq);
}
#else
int __init ak49xx_irq_of_init(struct device_node *node,
			       struct device_node *parent)
{
	struct ak49xx_irq_drv_data *data;

	pr_debug("%s: node %s, node parent %s\n", __func__,
		 node->name, node->parent->name);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * ak49xx_intc interrupt controller supports N to N irq mapping with
	 * single cell binding with irq numbers(offsets) only.
	 * Use irq_domain_simple_ops that has irq_domain_simple_map and
	 * irq_domain_xlate_onetwocell.
	 */
	data->domain = irq_domain_add_linear(node, AK49XX_MAX_NUM_IRQS,
					     &irq_domain_simple_ops, data);
	if (!data->domain) {
		kfree(data);
		return -ENOMEM;
	}

	return 0;
}

static struct ak49xx_irq_drv_data *
ak49xx_get_irq_drv_d(const struct ak49xx *ak49xx)
{
	struct device_node *pnode;
	struct irq_domain *domain;

	pnode = of_irq_find_parent(ak49xx->dev->of_node);
	/* Shouldn't happen */
	if (unlikely(!pnode))
		return NULL;

	domain = irq_find_host(pnode);
	return (struct ak49xx_irq_drv_data *)domain->host_data;
}

static int phyirq_to_virq(struct ak49xx *ak49xx, int offset)
{
	struct ak49xx_irq_drv_data *data;

	data = ak49xx_get_irq_drv_d(ak49xx);
	if (!data) {
		pr_warn("%s: not registered to interrupt controller\n",
			__func__);
		return -EINVAL;
	}
	return irq_linear_revmap(data->domain, offset);
}

static int virq_to_phyirq(struct ak49xx *ak49xx, int virq)
{
	struct irq_data *irq_data = irq_get_irq_data(virq);
	return irq_data->hwirq;
}

static unsigned int ak49xx_irq_get_upstream_irq(struct ak49xx *ak49xx)
{
	struct ak49xx_irq_drv_data *data;

	/* Hold parent's of node */
	if (!of_node_get(of_irq_find_parent(ak49xx->dev->of_node)))
		return -EINVAL;

	data = ak49xx_get_irq_drv_d(ak49xx);
	if (!data) {
		pr_err("%s: interrupt controller is not registerd\n", __func__);
		return 0;
	}

	rmb();
	return data->irq;
}

static void ak49xx_irq_put_upstream_irq(struct ak49xx *ak49xx)
{
	/* Hold parent's of node */
	of_node_put(of_irq_find_parent(ak49xx->dev->of_node));
}

static int ak49xx_map_irq(struct ak49xx *ak49xx, int irq)
{
	return of_irq_to_resource(ak49xx->dev->of_node, irq, NULL);
}

static int __devinit ak49xx_irq_probe(struct platform_device *pdev)
{
	int irq;
	struct irq_domain *domain;
	struct ak49xx_irq_drv_data *data;
	int ret = -EINVAL;

	irq = platform_get_irq_byname(pdev, "cdc-int");
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: Couldn't find cdc-int node(%d)\n",
			__func__, irq);
		return -EINVAL;
	} else {
		dev_dbg(&pdev->dev, "%s: virq = %d\n", __func__, irq);
		domain = irq_find_host(pdev->dev.of_node);
		data = (struct ak49xx_irq_drv_data *)domain->host_data;
		data->irq = irq;
		wmb();
		ret = 0;
	}

	return ret;
}

static int ak49xx_irq_remove(struct platform_device *pdev)
{
	struct irq_domain *domain;
	struct ak49xx_irq_drv_data *data;

	domain = irq_find_host(pdev->dev.of_node);
	data = (struct ak49xx_irq_drv_data *)domain->host_data;
	data->irq = 0;
	wmb();

	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "akm,ak49xx-irq" },
	{ }
};

static struct platform_driver ak49xx_irq_driver = {
	.probe = ak49xx_irq_probe,
	.remove = ak49xx_irq_remove,
	.driver = {
		.name = "ak49xx_intc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_match),
	},
};

static int ak49xx_irq_drv_init(void)
{
	return platform_driver_register(&ak49xx_irq_driver);
}
subsys_initcall(ak49xx_irq_drv_init);

static void ak49xx_irq_drv_exit(void)
{
	platform_driver_unregister(&ak49xx_irq_driver);
}
module_exit(ak49xx_irq_drv_exit);
#endif /* CONFIG_OF */
