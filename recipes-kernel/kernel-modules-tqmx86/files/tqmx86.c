/*
 * TQ-Systems PLD MFD core driver
 *
 * Copyright (c) 2015 TQ-Systems GmbH
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

#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "i2c-machxo2.h"

#define TQMX86_IOBASE	0x160
#define TQMX86_IOSIZE	0x3f
#define TQMX86_I2C_BASE	0x40
#define TQMX86_I2C_IRQ	10	/* default */
#define TQMX86_CLK	33000	/* default */

/* Registers offsets */
#define TQMX86_BID	0x20	/* Board ID */
#define TQMX86_BREV	0x21	/* Board and PLD Revisions */
#define TQMX86_IOEIC	0x26	/* I/O Extension Interrupt Configuration */
#define TQMX86_I2C_DET	0x47	/* I2C controller detection register */
#define TQMX86_I2C_IEN	0x49	/* machxo2 I2C nterrupt enable register */
#define TQMX86_SAUC	(0x2d+0xa)	/* Sanctioned Alternate Uses Configuration */


struct tqmx86_info {
	u32	board_id;
	u32	board_rev;
	u32	pld_rev;
	u32	i2c_type;
	u32	gpio_bank2;
	char	*board_name;
};

#define I2C_KIND_SOFT	1	/* Wishbone soft controller */
#define I2C_KIND_HARD	2	/* Machxo2 hard controller */

/**
 * struct tqmx86_device_data - Internal representation of the PLD device
 * @io_base:		Pointer to the IO memory
 * @pld_clock:		PLD clock frequency
 * @dev:		Pointer to kernel device structure
 */
struct tqmx86_device_data {
	void __iomem		*io_base;
	u32			pld_clock;
	struct device		*dev;
	struct tqmx86_info	info;
};

/**
 * struct tqmx86_platform_data - PLD hardware configuration structure
 * @pld_clock:			PLD clock frequency
 * @ioresource:			IO addresses of the PLD
 * @register_cells:		PLD specific register_cells callback
 */
struct tqmx86_platform_data {
	u32				pld_clock;
	struct resource			*ioresource;
	int (*register_cells)		(struct tqmx86_device_data *);
};

#define MAX_ID_LEN 5
static char force_device_id[MAX_ID_LEN + 1] = "";
module_param_string(force_device_id, force_device_id,
		    sizeof(force_device_id), 0);
MODULE_PARM_DESC(force_device_id, "Override detected product");

static uint i2c_speed[] = {
	100,
	0	/* disabled */
};

module_param_array(i2c_speed, uint, NULL, 0);
MODULE_PARM_DESC(i2c_speed, "I2C speed (KHz) - 50, 100 or 400, 0 - disable");

static uint i2c_irq[] = {
	0,
	0
};
module_param_array(i2c_irq, uint, NULL, 0);
MODULE_PARM_DESC(i2c_irq, "I2C IRQ numbers");

static u8 i2c_irq_ctl[16] = {
	[7] = 1,
	[9] = 2,
	[12] = 3
};

static uint gpio_irq = 0; /* default value for irq, 0 for disable */
module_param(gpio_irq, uint, 0);
MODULE_PARM_DESC(gpio_irq, "GPIO IRQ number (7, 9, 12)");

static __inline u8 tqmx86_readb(struct tqmx86_device_data *pld, u32 off)
{
	return ioread8((void __iomem *)((ulong)pld->io_base + off));
}

static __inline void tqmx86_writeb(struct tqmx86_device_data *pld, u8 val, u32 off)
{
	iowrite8(val, (void __iomem *)((ulong)pld->io_base + off));
}


enum tqmx86_cells {
	TQMX86_I2C = 0,
	TQMX86_I2C2,
	TQMX86_I2C_SOFT,
	TQMX86_WDT,
	TQMX86_GPIO,
	TQMX86_UART,
};

static struct resource tqmx_i2c_resources[] = {
	DEFINE_RES_IO(0x1a0,10),
	DEFINE_RES_IRQ(TQMX86_I2C_IRQ)
};

static struct resource tqmx_i2c_second_resources[] = {
	DEFINE_RES_IO(0x1aa,10),
	DEFINE_RES_IRQ(0)
};

static struct resource tqmx_i2c_soft_resources[] = {
	DEFINE_RES_IO(0x1a0,10),
	DEFINE_RES_IRQ(0)
};

static struct resource tqmx_watchdog_resources[] = {
	{
	.start = 0x18b,
	.end   = 0x18c,
	.flags = IORESOURCE_IO,
	}
};

static struct resource tqmx_gpio_resources[] = {
	/* DEFINE_RES_IRQ(0), */
	{
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_IRQ 
		/* | IORESOURCE_IRQ_HIGHEDGE */ /* ISA style */
		/* | IORESOURCE_IRQ_LOWLEVEL */ /* PCI style */
	},
	{
	.start = 0x18d,
	.end   = 0x198,
	.flags = IORESOURCE_IO,
	}
};

static struct i2c_board_info tqmx86_i2c_devices[] = {
	/* 4K EEPROM at 0x50 */
	{
		.type = "24c32",
		.addr = 0x50,
	},
};

static struct machxo2_i2c_platform_data machxo2_platfom_data[] = {
	{
		.clock_khz = TQMX86_CLK,
		.speed = 400,
		.num_devices = 1,
		.devices = tqmx86_i2c_devices,
	}, {
		.clock_khz = TQMX86_CLK,
		.speed = 0,
	}
};

static const struct mfd_cell tqmx86_devs[] = {
	[TQMX86_I2C] = {
		.name = "i2c-machxo2",
		.platform_data = &machxo2_platfom_data[0],
		.pdata_size = sizeof(machxo2_platfom_data[0]),
		.resources = tqmx_i2c_resources,
		.num_resources = ARRAY_SIZE(tqmx_i2c_resources),
	},
	[TQMX86_I2C2] = {
		.name = "i2c-machxo2",
		.platform_data = &machxo2_platfom_data[1],
		.pdata_size = sizeof(machxo2_platfom_data[1]),
		.resources = tqmx_i2c_second_resources,
		.num_resources = ARRAY_SIZE(tqmx_i2c_second_resources),
	},
	[TQMX86_I2C_SOFT] = {
		.name = "i2c-wishbone",
		.platform_data = &machxo2_platfom_data[0],
		.pdata_size = sizeof(machxo2_platfom_data[0]),
		.resources = tqmx_i2c_soft_resources,
		.num_resources = ARRAY_SIZE(tqmx_i2c_soft_resources),
	},
	[TQMX86_WDT] = {
		.name = "tqmx86-wdt",
		.resources = tqmx_watchdog_resources,
		.num_resources = 1,
		.ignore_resource_conflicts = 1,
	},
	[TQMX86_GPIO] = {
		.name = "tqmx86-gpio",
		.resources = tqmx_gpio_resources,
		.num_resources = ARRAY_SIZE(tqmx_gpio_resources),
		.ignore_resource_conflicts = 1,
	},
//	[TQMX86_UART] = {
//		.name = "tqmx86-uart",
//	},
};

#define TQMX86_MAX_DEVS	ARRAY_SIZE(tqmx86_devs)

static int tqmx86_register_cells_generic(struct tqmx86_device_data *pld)
{
	struct mfd_cell devs[TQMX86_MAX_DEVS];
	int 	i = 0;
	u8	ioeic_val = 0;

	if (i2c_speed[0]) {
		tqmx_i2c_resources[1].start = i2c_irq[0];
		machxo2_platfom_data[0].clock_khz = pld->pld_clock;
		machxo2_platfom_data[0].speed = i2c_speed[0];
		if (pld->info.i2c_type == I2C_KIND_HARD)
			devs[i++] = tqmx86_devs[TQMX86_I2C];
		else
			devs[i++] = tqmx86_devs[TQMX86_I2C_SOFT];
		ioeic_val |= i2c_irq_ctl[i2c_irq[0]] & 0x3;
	}
	if (i2c_speed[1]) {
		tqmx_i2c_second_resources[1].start = i2c_irq[1];
		machxo2_platfom_data[1].clock_khz = pld->pld_clock;
		machxo2_platfom_data[1].speed = i2c_speed[1];
		if (pld->info.i2c_type == I2C_KIND_HARD)
			devs[i++] = tqmx86_devs[TQMX86_I2C2];
		ioeic_val |= (i2c_irq_ctl[i2c_irq[0]] & 0x3) << 2;
	}
	if (gpio_irq) {
		ioeic_val |= (i2c_irq_ctl[gpio_irq] & 0x3) << 4;
		/* Assumes the IRQ resource is first. */
		tqmx_gpio_resources[0].start = gpio_irq;
	}
	tqmx86_writeb(pld, ioeic_val, TQMX86_IOEIC);
	if (tqmx86_readb(pld, TQMX86_IOEIC) != ioeic_val) {
		dev_warn(pld->dev, "i2c interrupts not supported.\n");
		tqmx_i2c_resources[1].start = tqmx_i2c_resources[1].end = 0;
		tqmx_i2c_second_resources[1].start = 
			tqmx_i2c_second_resources[1].end = 0;
		tqmx_gpio_resources[0].start =
			tqmx_gpio_resources[0].end = 0;
	}

	devs[i++] = tqmx86_devs[TQMX86_WDT];

	devs[i++] = tqmx86_devs[TQMX86_GPIO];

#if 0
	devs[i++] = tqmx86_devs[TQMX86_UART];
#endif

	return mfd_add_devices(pld->dev, -1, devs, i, NULL, 0, NULL);
}

static struct resource tqmx86_ioresource = {
	.start	= TQMX86_IOBASE,
	.end	= TQMX86_IOBASE + TQMX86_IOSIZE,
	.flags	= IORESOURCE_IO,
};

static const struct tqmx86_platform_data tqmx86_platform_data_generic = {
	.pld_clock		= TQMX86_CLK,
	.ioresource		= &tqmx86_ioresource,
	.register_cells		= tqmx86_register_cells_generic,
};

static struct platform_device *tqmx86_pdev;

static int tqmx86_create_platform_device(const struct dmi_system_id *id)
{
	struct tqmx86_platform_data *pdata = id->driver_data;
	int ret;

	tqmx86_pdev = platform_device_alloc("tqmx86", -1);
	if (!tqmx86_pdev)
		return -ENOMEM;

	ret = platform_device_add_data(tqmx86_pdev, pdata, sizeof(*pdata));
	if (ret)
		goto err;

	ret = platform_device_add_resources(tqmx86_pdev, pdata->ioresource, 1);
	if (ret)
		goto err;

	ret = platform_device_add(tqmx86_pdev);
	if (ret)
		goto err;

	return 0;
err:
	platform_device_put(tqmx86_pdev);
	return ret;
}

static struct tq_board_info {
	char *name;
	u32	pld_clock;
} tq_board_info[] = {
	{"", 0},
	{"TQMxE38M", 33000},
	{"TQMx50UC", 24000},
	{"TQMxE38C", 33000},
	{"TQMx60EB", 24000},
	{"TQMxE39M", 25000},
	{"TQMxE39C", 25000},
	{"TQMxE39x", 25000},
	{"TQMx70EB", 24000},
	{"TQMx80UC", 24000},
	{"TQMx90UC", 24000},
	{"TQMxE40M", 24000}
};

static struct tq_board_info tq_board_info2[] = {
	{"TQMxE39S", 25000}
};


static ssize_t tqmx86_board_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tqmx86_device_data *pld = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 pld->info.board_name);
}

static ssize_t tqmx86_board_rev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tqmx86_device_data *pld = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", pld->info.board_rev);
}

static ssize_t tqmx86_pld_rev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tqmx86_device_data *pld = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "PLD Revision: %d\n", pld->info.pld_rev);
}

static DEVICE_ATTR(board_id, S_IRUGO, tqmx86_board_show, NULL);
static DEVICE_ATTR(board_version, S_IRUGO, tqmx86_board_rev_show, NULL);
static DEVICE_ATTR(pld_version, S_IRUGO, tqmx86_pld_rev_show, NULL);

static struct attribute *pld_attributes[] = {
	&dev_attr_board_id.attr,
	&dev_attr_board_version.attr,
	&dev_attr_pld_version.attr,
	NULL
};

static const struct attribute_group pld_attr_group = {
	.attrs = pld_attributes,
};

/*
 * tqmx86_register_cells - register cell drivers
 *
 * This function registers cell drivers for the detected hardware by calling
 * the configured tqmx86_register_cells_XXXX function which is responsible
 * to detect and register the needed cell drivers.
 */
static int tqmx86_register_cells(struct tqmx86_device_data *pld)
{
	struct tqmx86_platform_data *pdata = dev_get_platdata(pld->dev);

	return pdata->register_cells(pld);
}


static int tqmx86_detect_device(struct tqmx86_device_data *pld)
{
	u8 board_id, rev, i2c_det, i2c_ien;
	int ret;


	board_id = tqmx86_readb(pld, TQMX86_BID);
	if (board_id == 0 || board_id > ARRAY_SIZE(tq_board_info) - 1)
		return -ENODEV;

	pld->info.gpio_bank2 = !(tqmx86_readb(pld, TQMX86_SAUC) & (1 << 7));

	pld->info.board_name = tq_board_info[board_id].name;
	pld->pld_clock = tq_board_info[board_id].pld_clock;

	if (board_id == 5 && pld->info.gpio_bank2) {
		pld->info.board_name = tq_board_info2[0].name;
		pld->pld_clock = tq_board_info2[0].pld_clock;
	}

	rev = tqmx86_readb(pld, TQMX86_BREV);
	pld->info.board_id = board_id;
	pld->info.board_rev = rev >> 4;
	pld->info.pld_rev = rev & 0xf;

	i2c_det = tqmx86_readb(pld, TQMX86_I2C_DET);
	i2c_ien = tqmx86_readb(pld, TQMX86_I2C_IEN);
	if (i2c_det == 0xa5 && (i2c_ien & 0xf0) == 0xf0)
		pld->info.i2c_type = I2C_KIND_SOFT;
	else
		pld->info.i2c_type = I2C_KIND_HARD;

	dev_info(pld->dev, "Found TQx86 PLD - Board ID %d, PCB Revision %d, PLD Revision %d\n",
		 board_id, rev >> 4, rev & 0xf);

	ret = sysfs_create_group(&pld->dev->kobj, &pld_attr_group);
	if (ret)
		return ret;
	ret = tqmx86_register_cells(pld);
	if (ret)
		sysfs_remove_group(&pld->dev->kobj, &pld_attr_group);

	return ret;
}

static int tqmx86_probe(struct platform_device *pdev)
{
	struct tqmx86_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device *dev = &pdev->dev;
	struct tqmx86_device_data *pld;
	struct resource *ioport;
	int ret;

	pld = devm_kzalloc(dev, sizeof(*pld), GFP_KERNEL);
	if (!pld)
		return -ENOMEM;

	ioport = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!ioport)
		return -EINVAL;

	pld->io_base = devm_ioport_map(dev, ioport->start,
					ioport->end - ioport->start);
	if (!pld->io_base)
		return -ENOMEM;

	pld->pld_clock = pdata->pld_clock;
	pld->dev = dev;

	platform_set_drvdata(pdev, pld);

	ret = tqmx86_detect_device(pld);
	if (ret)
		return ret;

	return 0;
}

static int tqmx86_remove(struct platform_device *pdev)
{
	struct tqmx86_device_data *pld = dev_get_drvdata(&pdev->dev);
	sysfs_remove_group(&pld->dev->kobj, &pld_attr_group);
	mfd_remove_devices(&pdev->dev);

	return 0;
}

static struct platform_driver tqmx86_driver = {
	.driver		= {
		.name	= "tqmx86",
	},
	.probe		= tqmx86_probe,
	.remove		= tqmx86_remove,
};

static struct dmi_system_id tqmx86_dmi_table[] __initdata = {
	{
		.ident = "TQMX86",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TQ-Group"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TQMx"),
		},
		.driver_data = (void *)&tqmx86_platform_data_generic,
		.callback = tqmx86_create_platform_device,
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, tqmx86_dmi_table);

static int __init tqmx86_init(void)
{
	const struct dmi_system_id *id;
	int ret;

	if (i2c_speed[0]) {
		if (i2c_speed[0] > 400) {
			printk(KERN_WARNING "tqmx86: Invalid primary i2c speed (%d)\n",
				i2c_speed[0]);
			i2c_speed[0] = 400;
		}
		if (i2c_irq[0] > 15) {
			printk(KERN_WARNING "tqmx86: Invalid primary i2c IRQ (%d)\n",
				i2c_irq[0]);
			i2c_irq[0] = 0;
		} else if (i2c_irq_ctl[i2c_irq[0]] == 0) {
			printk(KERN_WARNING "tqmx86: i2c IRQ %d not supported\n", i2c_irq[0]);
		}
	}
	if (i2c_speed[1]) {
		if (i2c_speed[1] > 400) {
			printk(KERN_WARNING "tqmx86: Invalid secondary i2c speed (%d)\n",
				i2c_speed[1]);
			i2c_speed[1] = 400;
		}
		if (i2c_irq[1] > 15) {
			printk(KERN_WARNING "tqmx86: Invalid secondary i2c IRQ (%d)\n",
				i2c_irq[1]);
			i2c_irq[1] = 0;
		} else if (i2c_irq_ctl[i2c_irq[1]] == 0) {
			printk(KERN_WARNING "tqmx86: i2c IRQ %d not supported\n", i2c_irq[1]);
		}
	}
	if (gpio_irq) {
		if (gpio_irq > 15) {
			printk(KERN_WARNING "tqmx86: Invalid GPIO IRQ (%d)\n",
				gpio_irq);
			i2c_irq[1] = 0;
		} else if (i2c_irq_ctl[gpio_irq] == 0) {
			printk(KERN_WARNING "tqmx86: GPIO IRQ %d not supported\n", gpio_irq);
		}
	}
	if (force_device_id[0]) {
		for (id = tqmx86_dmi_table;
		     id->matches[0].slot != DMI_NONE; id++)
			if (strstr(id->ident, force_device_id))
				if (id->callback && !id->callback(id))
					break;
		if (id->matches[0].slot == DMI_NONE)
			return -ENODEV;
	} else {
		if (!dmi_check_system(tqmx86_dmi_table))
			return -ENODEV;
	}

	ret = platform_driver_register(&tqmx86_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit tqmx86_exit(void)
{
	if (tqmx86_pdev)
		platform_device_unregister(tqmx86_pdev);

	platform_driver_unregister(&tqmx86_driver);
}

module_init(tqmx86_init);
module_exit(tqmx86_exit);

MODULE_DESCRIPTION("TQx86 PLD Core Driver");
MODULE_AUTHOR("Vadim V.Vlasov <vvlasov@dev.rtsoft.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tqmx86");
