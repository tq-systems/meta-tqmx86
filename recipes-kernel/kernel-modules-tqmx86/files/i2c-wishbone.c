/*
 * i2c-wishbone.c: I2C bus driver for Lattice Wishbone soft I2C controller
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include "i2c-machxo2.h"
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>

struct wishbone_i2c {
	int base;
	int clock_khz;
	int speed;
	int speed_scale;
	int flags;
	wait_queue_head_t wait;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	int pos;
	int nmsgs;
	int state; /* see STATE_ */
	int error;
};

#define I2C_FL_NOIRQ	1	/* flags above */

/* registers */
#define I2C_BR0		0 /* Clock Pre-scale, rw */
#define I2C_BR1		1 /* Clock Pre-scale, rw */
#define I2C_CR		2 /* Control, rw */
#define I2C_TXDR	3 /* Transmit Data, wo */
#define I2C_RXDR	3 /* Receive Data, ro, same as TXDR */
#define I2C_CMDR	4 /* Command, wo */
#define I2C_SR		4 /* Status, ro, same as CMDR */

#define I2C_CR_I2CEN	0x80 /* Enable I2C */
#define I2C_CR_IEN	0x40 /* Enable IRQ */
#define I2C_CMDR_STA	0x80 /* Start */
#define I2C_CMDR_STO	0x40 /* Stop */
#define I2C_CMDR_RD	0x20 /* Read */
#define I2C_CMDR_WR	0x10 /* Write */
#define I2C_CMDR_ACK	0x08 /* Send NACK if the bit is set */
#define I2C_SR_RARC	0x80 /* 0 - Received ACK */
#define I2C_SR_BUSY	0x40 /* Bus Busy */
#define I2C_SR_ARBL	0x20 /* Arbitration Lost */
#define I2C_SR_TIP	0x02 /* Transmit In Progress */
#define I2C_SR_IF	0x01 /* IRQ flag */

#define STATE_DONE		0	/* xfer complete or error */
#define STATE_START		1
#define STATE_WRITE		2
#define STATE_READ		3


#define MAX_RETRIES	400
/* delay 2-3 SCL (5-7usec for 400KHz) */
//#define DELAY(i2c)	usleep_range(3 << i2c->speed_scale, 5 << i2c->speed_scale)
#define DELAY(i2c)	udelay(3 << i2c->speed_scale)

static void setreg(struct wishbone_i2c *i2c, int reg, u8 value)
{
	outb(value, i2c->base + reg);
}

static inline u8 getreg(struct wishbone_i2c *i2c, int reg)
{
	return inb(i2c->base + reg);
}


static void wishbone_process(struct wishbone_i2c *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	u8 stat = getreg(i2c, I2C_SR);

	if (i2c->state == STATE_DONE) {
		setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
		return;
	}

	dev_dbg(&i2c->adap.dev, "wishbone_process: state %x, stat %02x\n", i2c->state, stat);

	if (i2c->state == STATE_START) {
		if (msg->flags & I2C_M_RD) {
			stat = getreg(i2c, I2C_SR);
			if (stat & I2C_SR_RARC) {
				dev_dbg(&i2c->adap.dev, 
					"wait for read start: stat %02x\n", stat);
				i2c->state = STATE_DONE;
				i2c->error = -EIO;
				setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
				return;
			}
			if (msg->len == 0) {
				i2c->state = STATE_DONE;
				setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
			}
			i2c->state = STATE_READ;
			if (msg->len == 1 && i2c->nmsgs == 1)
				setreg(i2c, I2C_CMDR, I2C_CMDR_RD|I2C_CMDR_ACK);
			else
				setreg(i2c, I2C_CMDR, I2C_CMDR_RD);
			return;
		} else {
			i2c->state = STATE_WRITE;
		}
	}

	if (i2c->state == STATE_READ) {
		msg->buf[i2c->pos++] = getreg(i2c, I2C_RXDR);
	}

	/* end of msg? */
	if (i2c->pos == msg->len) {
		i2c->nmsgs--;
		i2c->msg++;
		i2c->pos = 0;
		msg = i2c->msg;

		if (i2c->nmsgs) {	/* end? */
			/* send start? */
			if (!(msg->flags & I2C_M_NOSTART)) {
				u8 addr = (msg->addr << 1);

				if (msg->flags & I2C_M_RD)
					addr |= 1;

				i2c->state = STATE_START;

				setreg(i2c, I2C_TXDR, addr);
				setreg(i2c, I2C_CMDR, I2C_CMDR_STA|I2C_CMDR_WR);
				return;
			} else
				i2c->state = (msg->flags & I2C_M_RD)
					? STATE_READ : STATE_WRITE;
		} else {
			dev_dbg(&i2c->adap.dev, 
				"end of xfer: stat %02x, state %d\n",
				stat, i2c->state);
			if (i2c->state == STATE_READ) {
				setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
				i2c->state = STATE_DONE;
			} else {
				if (stat & I2C_SR_RARC) {
					DELAY(i2c);
					stat = getreg(i2c, I2C_SR);
					dev_dbg(&i2c->adap.dev,
						"No ACK: new stat %02x\n", stat);
					if (stat & I2C_SR_RARC)
					i2c->error = -ENXIO;
				}
				i2c->state = STATE_DONE;
				setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
			}
			return;
		}
	}

	if (i2c->state == STATE_READ) {
		/* prepare to stop, if we are waiting for the last byte */
		if (i2c->pos == (msg->len - 1) && i2c->nmsgs == 1)
			setreg(i2c, I2C_CMDR, I2C_CMDR_RD|I2C_CMDR_ACK);
		else
			setreg(i2c, I2C_CMDR, I2C_CMDR_RD);
	} else {
		setreg(i2c, I2C_TXDR, msg->buf[i2c->pos++]);
		setreg(i2c, I2C_CMDR, I2C_CMDR_WR);
	}
}

static irqreturn_t wishbone_isr(int irq, void *dev_id)
{
	struct wishbone_i2c *i2c = dev_id;
	u8 irq_stat = getreg(i2c, I2C_SR) & I2C_SR_IF;

	if (!irq_stat)
		return IRQ_NONE;

	wishbone_process(i2c);
	setreg(i2c, I2C_CMDR, irq_stat); /* clear IRQ */

	if (i2c->state == STATE_DONE)
		wake_up(&i2c->wait);

	return IRQ_HANDLED;
}


static void wishbone_wait_irq(struct wishbone_i2c *i2c)
{
	u8 stat;
	int timeout = 0;

	do {
		DELAY(i2c);
		stat = getreg(i2c, I2C_SR);
	} while ((stat & I2C_SR_TIP) && timeout++ < MAX_RETRIES);


	if (!(stat & I2C_SR_TIP)) {
		wishbone_process(i2c);
	} else {
		i2c->state = STATE_DONE;
		i2c->error = -EIO;
		dev_dbg(&i2c->adap.dev, "stat %02x, retry %d\n",
			stat, timeout);
		setreg(i2c, I2C_CMDR, I2C_CMDR_STO);
	}
}

static void wait_not_busy(struct wishbone_i2c *i2c)
{
	int timeout = 0;
	u8 stat;

	while (timeout < MAX_RETRIES) {
		stat = getreg(i2c, I2C_SR);
		if ((stat & (I2C_SR_BUSY | I2C_SR_TIP)) == 0)
			break;
		DELAY(i2c);
		timeout++;
	}
}

static int wishbone_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct wishbone_i2c *i2c = i2c_get_adapdata(adap);

	i2c->msg = msgs;
	i2c->pos = 0;
	i2c->nmsgs = num;
	i2c->state = STATE_START;
	i2c->error = 0;

	dev_dbg(&adap->dev, "new msg: addr %02x, flags %x, nmsgs %d, len %d\n",
		msgs->addr, msgs->flags, num, msgs[0].len);

	setreg(i2c, I2C_TXDR,
			(i2c->msg->addr << 1) |
			((i2c->msg->flags & I2C_M_RD) ? 1:0));

	setreg(i2c, I2C_CMDR, I2C_CMDR_STA|I2C_CMDR_WR);

	if (!(i2c->flags & I2C_FL_NOIRQ)) {
		if (!wait_event_timeout(i2c->wait, (i2c->state == STATE_DONE), HZ))
			i2c->error = -ETIMEDOUT;
	} else {
		preempt_disable();
		while (i2c->state != STATE_DONE)
			wishbone_wait_irq(i2c);
		preempt_enable();
	}
	wait_not_busy(i2c);

	if (i2c->error)
		dev_dbg(&adap->dev, "end msg: error %d, nmsgs %d, pos %d\n",
		i2c->error, i2c->nmsgs, i2c->pos);

	return i2c->error?i2c->error:num;
}

static void wishbone_init(struct wishbone_i2c *i2c)
{
	int prescale;

	/* make sure the device is disabled */
	setreg(i2c, I2C_CR, 0);

	if (i2c->speed > 200) {
		i2c->speed = 400;
		i2c->speed_scale = 0;
	} else if (i2c->speed > 75) {
		i2c->speed = 100;
		i2c->speed_scale = 2;
	} else {
		i2c->speed = 50;
		i2c->speed_scale = 3;
	}
	if (i2c->clock_khz > 0) {
		prescale = i2c->clock_khz / (5 * i2c->speed) - 1;
		if (prescale & ~0x3ff)
			dev_warn(&i2c->adap.dev,
				"i2x-wishbone: Prescale too high: %d\n", prescale);

		setreg(i2c, I2C_BR0, prescale & 0xff);
		setreg(i2c, I2C_BR1, (prescale >> 8) & 3);
	}

	/* Init the device */
	setreg(i2c, I2C_CR, I2C_CR_I2CEN);
}


static u32 wishbone_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm wishbone_algorithm = {
	.master_xfer = wishbone_xfer,
	.functionality = wishbone_func,
};

static struct i2c_adapter wishbone_adapter = {
	.owner = THIS_MODULE,
	.name = "i2c-wishbone",
	.class = I2C_CLASS_DEPRECATED,
	.algo = &wishbone_algorithm,
};

static const struct of_device_id wishbone_i2c_match[] = {
	{
		.compatible = "lattice,i2c-wishbone",
	},
	{},
};
MODULE_DEVICE_TABLE(of, wishbone_i2c_match);

static int wishbone_i2c_probe(struct platform_device *pdev)
{
	struct wishbone_i2c *i2c;
	struct machxo2_i2c_platform_data *pdata;
	struct resource *res;
	int irq;
	int ret;
	int i;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (IS_ERR(res))
		return PTR_ERR(res);

	i2c->base = res->start;
	if (!devm_request_region(&pdev->dev, res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "Can't get I/O resource.\n");
		return -EBUSY;
	}

	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		i2c->clock_khz = pdata->clock_khz;
		i2c->speed = pdata->speed;
	}

	wishbone_init(i2c);

	init_waitqueue_head(&i2c->wait);
	if (irq > 0) {
		dev_info(&pdev->dev, "Using irq %d, speed %dKHz.\n", irq, i2c->speed);
		ret = devm_request_irq(&pdev->dev, irq, wishbone_isr, 0,
				       pdev->name, i2c);
		if (ret) {
			dev_err(&pdev->dev, "Cannot claim IRQ\n");
			return ret;
		}
		setreg(i2c, I2C_CR, I2C_CR_I2CEN | 1);
	} else {
		dev_info(&pdev->dev, "Running in poll mode, speed %dKHz, clock %dKHz.\n",
			 i2c->speed, pdata->clock_khz);
		i2c->flags |= I2C_FL_NOIRQ;
	}

	/* hook up driver to tree */
	platform_set_drvdata(pdev, i2c);
	i2c->adap = wishbone_adapter;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;

	/* add i2c adapter to i2c tree */
	ret = i2c_add_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		return ret;
	}

	/* add in known devices to the bus */
	if (pdata) {
		for (i = 0; i < pdata->num_devices; i++)
			i2c_new_device(&i2c->adap, pdata->devices + i);
	}

	return 0;
}

static int wishbone_i2c_remove(struct platform_device *pdev)
{
	struct wishbone_i2c *i2c = platform_get_drvdata(pdev);

	/* disable i2c logic */
	setreg(i2c, I2C_CR, 0);

	/* remove adapter & data */
	i2c_del_adapter(&i2c->adap);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wishbone_i2c_suspend(struct device *dev)
{
	struct wishbone_i2c *i2c = dev_get_drvdata(dev);

	/* make sure the device is disabled */
	setreg(i2c, I2C_CR, 0);

	return 0;
}

static int wishbone_i2c_resume(struct device *dev)
{
	struct wishbone_i2c *i2c = dev_get_drvdata(dev);

	wishbone_init(i2c);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wishbone_i2c_pm, wishbone_i2c_suspend, wishbone_i2c_resume);
#define I2C_PM	(&wishbone_i2c_pm)
#else
#define I2C_PM	NULL
#endif

static struct platform_driver wishbone_i2c_driver = {
	.probe   = wishbone_i2c_probe,
	.remove  = wishbone_i2c_remove,
	.driver  = {
		.name = "i2c-wishbone",
		.of_match_table = wishbone_i2c_match,
		.pm = I2C_PM,
	},
};

module_platform_driver(wishbone_i2c_driver);

MODULE_AUTHOR("<vvlasov@dev.rtsoft.ru>");
MODULE_DESCRIPTION("Lattice Wishbone soft I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-wishbone");
