/*
 * TQ-Systems TQMx86 PLD GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define TQMX86_NGPIO	8
#define TQMX86_NGPIO2	12
#define TQMX86_DIR_MASK	0xf0	/* 0-3 - output, 4-7 - input */
#define TQMX86_DIR_MASK1	0xc0	/* 0-5 - output, 6-11 - input */
#define TQMX86_DIR_MASK2	0x0f	/* 0-5 - output, 6-11 - input */
#define TQMX86_GPIODD	0	/* GPIO Data Direction Register */
#define TQMX86_GPIOD	1	/* GPIO Data Register */
#define TQMX86_GPIIC	3	/* GPI Interrupt Configuration Register */
#define TQMX86_GPIIS	4	/* GPI Interrupt Status Register */
#define TQMX86_GPIODD2	5	/* GPIO Data Direction Register 2 */
#define TQMX86_GPIOD2	6	/* GPIO Data Register 2 */
#define TQMX86_GPIIC2	7	/* GPI Interrupt Configuration Register 2 */
#define TQMX86_GPIIC3	8	/* GPI Interrupt Configuration Register 3 */
#define TQMX86_GPIIS2	9	/* GPI Interrupt Status Register 2 */
#define TQMX86_SAUC	0xa	/* Sanctioned Alternate Uses Configuration */

#define TQMX86_GPII_FALLING	BIT(0)
#define TQMX86_GPII_RISING	BIT(1)
#define TQMX86_GPII_MASK	(BIT(0) | BIT(1))
#define TQMX86_GPII_BITS	2

struct tqmx86_gpio_data {
	struct gpio_chip	chip;
	struct irq_chip		irq_chip;
	void __iomem		*io_base;
	int			irq;
	raw_spinlock_t		spinlock;
	u8			irq_type[TQMX86_NGPIO2];
	unsigned int	irq_gpio_offset;
	int	bank2;
	u8	altfunc;
};

static u8 tqmx86_gpio_read(struct tqmx86_gpio_data *gd, unsigned reg)
{
	return ioread8(gd->io_base + reg);
}

static void tqmx86_gpio_write(struct tqmx86_gpio_data *gd, u8 val, unsigned reg)
{
	iowrite8(val, gd->io_base + reg);
}

static int tqmx86_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);

	if (offset < 8)
		return !!(tqmx86_gpio_read(gpio, TQMX86_GPIOD) & BIT(offset));
	else
		return !!(tqmx86_gpio_read(gpio, TQMX86_GPIOD2) & BIT(offset - 8));
}

static void tqmx86_gpio_set_reg(struct gpio_chip *chip, unsigned offset, int value, unsigned reg)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	u8 val;

	if (offset >= 8) {
		offset -= 8;
		reg += TQMX86_GPIOD2 - TQMX86_GPIOD;
	}

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	val = tqmx86_gpio_read(gpio, reg);
	if (value)
		val |= BIT(offset);
	else
		val &= ~BIT(offset);
	tqmx86_gpio_write(gpio, val, reg);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);
}

static void tqmx86_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	tqmx86_gpio_set_reg(chip, offset, value, TQMX86_GPIOD);
}

static int tqmx86_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	tqmx86_gpio_set_reg(chip, offset, 0, TQMX86_GPIODD);
	return 0;
}

static int tqmx86_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);

	/* switch the alternate function to GPIO only once for each pin */
	if (offset <= 6 && (BIT(offset) & gpio->altfunc)) {
		gpio->altfunc &= ~BIT(offset);
		tqmx86_gpio_set_reg(chip, offset, 0, TQMX86_SAUC);
	}
	tqmx86_gpio_set_reg(chip, offset, 1, TQMX86_GPIODD);
	tqmx86_gpio_set_reg(chip, offset, value, TQMX86_GPIOD);
	return 0;
}

static int tqmx86_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);
	if (offset < 8)
		return !(tqmx86_gpio_read(gpio, TQMX86_GPIODD) & BIT(offset));
	else
		return !(tqmx86_gpio_read(gpio, TQMX86_GPIODD2) & BIT(offset - 8));
}

static u32 tqmx86_gpio_read_irq_conf(struct tqmx86_gpio_data *gpio)
{
	return (u32)tqmx86_gpio_read(gpio, TQMX86_GPIIC) | 
		((u32)tqmx86_gpio_read(gpio, TQMX86_GPIIC2) << 8) |
		((u32)tqmx86_gpio_read(gpio, TQMX86_GPIIC3) << 16);
}

static void tqmx86_gpio_write_irq_conf(struct tqmx86_gpio_data *gpio, u32 val)
{
	tqmx86_gpio_write(gpio, (u8)(val), TQMX86_GPIIC);
	tqmx86_gpio_write(gpio, (u8)(val >> 8), TQMX86_GPIIC2);
	tqmx86_gpio_write(gpio, (u8)(val >> 16), TQMX86_GPIIC3);
}

static u32 tqmx86_gpio_read_irq_stat(struct tqmx86_gpio_data *gpio)
{
	if (!gpio->bank2)
		return (u32)tqmx86_gpio_read(gpio, TQMX86_GPIIS) & 0x0f;
	return ((u32)tqmx86_gpio_read(gpio, TQMX86_GPIIS) & 0x0f) | 
		((u32)tqmx86_gpio_read(gpio, TQMX86_GPIIS2) << 4);
}

static void tqmx86_gpio_write_irq_stat(struct tqmx86_gpio_data *gpio, u32 val)
{
	tqmx86_gpio_write(gpio, (u8)(val & 0xf), TQMX86_GPIIS);
	tqmx86_gpio_write(gpio, (u8)(val >> 4), TQMX86_GPIIS2);
}

static void tqmx86_gpio_irq_mask(struct irq_data *data)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned int offset = (data->hwirq - gpio->irq_gpio_offset);
	unsigned long flags;
	u32 gpiic, mask;

	mask = TQMX86_GPII_MASK << (offset * TQMX86_GPII_BITS);

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpiic = tqmx86_gpio_read_irq_conf(gpio);
	gpiic &= ~mask;
	tqmx86_gpio_write_irq_conf(gpio, gpiic);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);
}

static void tqmx86_gpio_irq_unmask(struct irq_data *data)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned int offset = (data->hwirq - gpio->irq_gpio_offset);
	unsigned long flags;
	u32 gpiic, mask;

	mask = TQMX86_GPII_MASK << (offset * TQMX86_GPII_BITS);

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpiic = tqmx86_gpio_read_irq_conf(gpio);
	gpiic &= ~mask;
	gpiic |= gpio->irq_type[offset] << (offset * TQMX86_GPII_BITS);
	tqmx86_gpio_write_irq_conf(gpio, gpiic);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);
}

static int tqmx86_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned int offset = (data->hwirq - gpio->irq_gpio_offset);
	unsigned int edge_type = type & IRQF_TRIGGER_MASK;
	unsigned long flags;
	u8 new_type;
	u32 gpiic;

	switch (edge_type) {
	case IRQ_TYPE_EDGE_RISING:
		new_type = TQMX86_GPII_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		new_type = TQMX86_GPII_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		new_type = TQMX86_GPII_FALLING | TQMX86_GPII_RISING;
		break;
	default:
		return -EINVAL; /* not supported */
	}

	gpio->irq_type[offset] = new_type;

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpiic = tqmx86_gpio_read_irq_conf(gpio);
	gpiic &= ~((TQMX86_GPII_MASK) << (offset * TQMX86_GPII_BITS));
	gpiic |= (u32)new_type << (offset * TQMX86_GPII_BITS);
	tqmx86_gpio_write_irq_conf(gpio, gpiic);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);

	return 0;
}

static void tqmx86_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned long irq_bits;
	int i = 0, child_irq;
	u8 irq_status;

	chained_irq_enter(irq_chip, desc);

	for (;;) {
		irq_status = tqmx86_gpio_read_irq_stat(gpio);
		if (!irq_status) break;
		tqmx86_gpio_write_irq_stat(gpio, irq_status);

		irq_bits = irq_status;
		for_each_set_bit(i, &irq_bits, TQMX86_NGPIO2) {
			child_irq = irq_find_mapping(gpio->chip.irq.domain,
							 i + gpio->irq_gpio_offset);
			generic_handle_irq(child_irq);
		}
	}

	chained_irq_exit(irq_chip, desc);
}

/* Minimal runtime PM is needed by the IRQ subsystem */
static int __maybe_unused tqmx86_gpio_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused tqmx86_gpio_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tqmx86_gpio_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(tqmx86_gpio_runtime_suspend,
			   tqmx86_gpio_runtime_resume, NULL)
};

static int tqmx86_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tqmx86_gpio_data *gpio;
	struct gpio_chip *chip;
	void __iomem *io_base;
	struct resource *res;
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get I/O\n");
		return -ENODEV;
	}

	io_base = devm_ioport_map(&pdev->dev, res->start, resource_size(res));
	if (!io_base)
		return -ENOMEM;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	raw_spin_lock_init(&gpio->spinlock);
	gpio->io_base = io_base;

	gpio->bank2 = !(tqmx86_gpio_read(gpio, TQMX86_SAUC) & BIT(7));

#if 0
	/* if enabled this would force all GPIOs to platfrom spec defaults. */
	/* if disabled the functions remain at PLD defaults which is preferable */
	if (gpio->bank2) {
		tqmx86_gpio_write(gpio, (u8)~TQMX86_DIR_MASK1, TQMX86_GPIODD);
		tqmx86_gpio_write(gpio, (u8)~TQMX86_DIR_MASK2, TQMX86_GPIODD2);
		tqmx86_gpio_write(gpio, (u8)0x00, TQMX86_SAUC);
	} else
		tqmx86_gpio_write(gpio, (u8)~TQMX86_DIR_MASK, TQMX86_GPIODD);
#endif

	if (gpio->bank2)
		gpio->altfunc = tqmx86_gpio_read(gpio, TQMX86_SAUC);
	else
		/* On COMexpress GPIs 0-3 are mapped to GPIOs 4-7 */
		gpio->irq_gpio_offset = 4;

	platform_set_drvdata(pdev, gpio);

	chip = &gpio->chip;
	chip->label = "gpio-tqmx86";
	chip->owner = THIS_MODULE;
	chip->can_sleep = false;
	chip->base = -1;
	chip->direction_input = tqmx86_gpio_direction_input;
	chip->direction_output = tqmx86_gpio_direction_output;
	chip->get_direction = tqmx86_gpio_get_direction;
	chip->get = tqmx86_gpio_get;
	chip->set = tqmx86_gpio_set;
	chip->ngpio = gpio->bank2?TQMX86_NGPIO2:TQMX86_NGPIO;
	chip->irq.need_valid_mask = true;
	chip->parent = pdev->dev.parent;

	pm_runtime_enable(&pdev->dev);

	ret = devm_gpiochip_add_data(dev, chip, gpio);
	if (ret) {
		dev_err(dev, "Could not register GPIO chip\n");
		goto out_pm_dis;
	}

	if (irq) {
		struct irq_chip *irq_chip = &gpio->irq_chip;
		u32 irq_status;

		irq_chip->name = chip->label;
		irq_chip->parent_device = &pdev->dev;
		irq_chip->irq_mask = tqmx86_gpio_irq_mask;
		irq_chip->irq_unmask = tqmx86_gpio_irq_unmask;
		irq_chip->irq_set_type = tqmx86_gpio_irq_set_type;

		/* Mask all interrupts */
		tqmx86_gpio_write_irq_conf(gpio, 0);

		/* Clear all pending interrupts */
		irq_status = tqmx86_gpio_read_irq_stat(gpio);
		tqmx86_gpio_write_irq_stat(gpio, irq_status);

		ret = gpiochip_irqchip_add(chip, irq_chip,
					   0, handle_simple_irq,
					   IRQ_TYPE_EDGE_BOTH);
		if (ret) {
			dev_err(dev, "Could not add irq chip\n");
			goto out_pm_dis;
		}

		gpiochip_set_chained_irqchip(chip, irq_chip,
					     irq, tqmx86_gpio_irq_handler);
	}
	
	if (!gpio->bank2) {
		/* Only GPIOs 4-7 are valid for interrupts on single bank COMexpress CPLD */
		clear_bit(0, chip->irq.valid_mask);
		clear_bit(1, chip->irq.valid_mask);
		clear_bit(2, chip->irq.valid_mask);
		clear_bit(3, chip->irq.valid_mask);
	}

	dev_info(dev, "GPIO functionality initialized with %d pins\n",
		 chip->ngpio);

	return 0;

out_pm_dis:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static struct platform_driver tqmx86_gpio_driver = {
	.driver = {
		.name = "tqmx86-gpio",
		.pm = &tqmx86_gpio_dev_pm_ops,
	},
	.probe		= tqmx86_gpio_probe,
};

module_platform_driver(tqmx86_gpio_driver);

MODULE_DESCRIPTION("TQMx86 PLD GPIO Driver");
MODULE_AUTHOR("Vadim V.Vlasov <vvlasov@dev.rtsoft.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tqmx86-gpio");
