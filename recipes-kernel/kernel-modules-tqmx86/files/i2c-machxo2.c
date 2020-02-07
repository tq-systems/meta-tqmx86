/*
 * i2c-machxo2.c: I2C bus driver for Lattice MachxO2 I2C controller
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

struct machxo2_i2c {
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
#define I2C_CR		0 /* Control, rw */
#define I2C_CMDR	1 /* Command, rw */
#define I2C_BR0		2 /* Clock Pre-scale, rw */
#define I2C_BR1		3 /* Clock Pre-scale, rw */
#define I2C_TXDR	4 /* Transmit Data, wo */
#define I2C_SR		5 /* Status, ro */
#define I2C_RXDR	7 /* Receive Data, ro */
#define I2C_IRQ		8 /* IRQ, rw */
#define I2C_IRQEN	9 /* IRQ Enable, rw */

#define I2C_CR_I2CEN	0x80 /* Enable I2C */
#define I2C_CMDR_STA	0x80 /* Start */
#define I2C_CMDR_STO	0x40 /* Stop */
#define I2C_CMDR_RD	0x20 /* Read */
#define I2C_CMDR_WR	0x10 /* Write */
#define I2C_CMDR_ACK	0x08 /* Send NACK if the bit is set */
#define I2C_CMDR_CKSDIS	0x04 /* must be set to 1 */
#define I2C_SR_ARBL	0x08 /* Arbitration Lost */
#define I2C_SR_SRW	0x10 /* Slave Read/Write */
#define I2C_SR_RARC	0x20 /* 0 - Received ACK */
#define I2C_SR_BUSY	0x40 /* Bus Busy */
#define I2C_SR_TIP	0x80 /* Transmit In Progress */
#define I2C_SR_TRRDY	0x04 /* Transmitter/Receiver Ready */
#define I2C_SR_TROE	0x02 /* Transmitter/Receiver Overrun Error */

#define I2C_IRQ_MASK	0x0e /* Arbitration lost, Transmitter/Receiver Ready and Overrun Error */
#define I2C_SRW_DELAY	20  /* TODO: recalculate from bus freq? */

#define STATE_DONE		0	/* xfer complete or error */
#define STATE_START		1
#define STATE_WRITE		2
#define STATE_READ		3


#define MAX_RETRIES	400
/* delay 2-3 SCL (5-7usec for 400KHz) */
//#define DELAY(i2c)	usleep_range(3 << i2c->speed_scale, 5 << i2c->speed_scale)
#define DELAY(i2c)	udelay(1 << i2c->speed_scale)

static void setreg(struct machxo2_i2c *i2c, int reg, u8 value)
{
	outb(value, i2c->base + reg);
}

#define setcmdr(i2c, val)	setreg(i2c, I2C_CMDR, val | I2C_CMDR_CKSDIS)

static inline u8 getreg(struct machxo2_i2c *i2c, int reg)
{
	return inb(i2c->base + reg);
}


static void machxo2_process(struct machxo2_i2c *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	u8 stat = getreg(i2c, I2C_SR);

	if (i2c->state == STATE_DONE) {
		setcmdr(i2c, I2C_CMDR_STO);
		return;
	}

	/* error? */
	if (!(stat & I2C_SR_TRRDY) && (stat & (I2C_SR_ARBL | I2C_SR_TROE))) {
		i2c->state = STATE_DONE;
		i2c->error = -EIO;
		setcmdr(i2c, I2C_CMDR_STO);
		return;
	}

	if (i2c->state == STATE_START) {
		if (msg->flags & I2C_M_RD) {
			int i = I2C_SRW_DELAY;
			stat = getreg(i2c, I2C_SR);
			stat = getreg(i2c, I2C_SR);
			stat = getreg(i2c, I2C_SR);
			stat = getreg(i2c, I2C_SR);
			while (!(stat & I2C_SR_SRW) && i >= 0) {
				udelay(2<<i2c->speed_scale);
				stat = getreg(i2c, I2C_SR);
				i--;
			}
			if ((i < 0) || (stat & I2C_SR_RARC)) {
				dev_dbg(&i2c->adap.dev, 
					"wait for SRW: stat %02x, count %d\n",
					stat, i);
				i2c->state = STATE_DONE;
				i2c->error = -EIO;
				setcmdr(i2c, I2C_CMDR_STO);
				return;
			}
			if (msg->len == 0) {
				i2c->state = STATE_DONE;
				setcmdr(i2c, I2C_CMDR_STO);
			}
			i2c->state = STATE_READ;
			setcmdr(i2c, I2C_CMDR_RD);
			if (msg->len == 1 && i2c->nmsgs == 1) {
				DELAY(i2c);
				DELAY(i2c);
				setcmdr(i2c, I2C_CMDR_RD|I2C_CMDR_ACK|I2C_CMDR_STO);
			}
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
				setcmdr(i2c, I2C_CMDR_STA|I2C_CMDR_WR);
				return;
			} else
				i2c->state = (msg->flags & I2C_M_RD)
					? STATE_READ : STATE_WRITE;
		} else {
			dev_dbg(&i2c->adap.dev, 
				"end of xfer: stat %02x, state %d\n",
				stat, i2c->state);
			if (i2c->state == STATE_READ) {
				setcmdr(i2c, 0);
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
				setcmdr(i2c, I2C_CMDR_STO);
			}
			return;
		}
	}

	if (i2c->state == STATE_READ) {
		/* prepare to stop, if we are waiting for the last byte */
		if (i2c->pos == (msg->len - 1) && i2c->nmsgs == 1)
			setcmdr(i2c, I2C_CMDR_RD|I2C_CMDR_ACK|I2C_CMDR_STO);
	} else {
		if (i2c->pos > 0 && stat & I2C_SR_RARC) {
			dev_dbg(&i2c->adap.dev, "No ACK at %d\n", i2c->pos);
			i2c->error = -ENXIO;
			i2c->state = STATE_DONE;
			setcmdr(i2c, I2C_CMDR_STO);
			return;
		}
		setreg(i2c, I2C_TXDR, msg->buf[i2c->pos++]);
		setcmdr(i2c, I2C_CMDR_WR);
	}
}

static irqreturn_t machxo2_isr(int irq, void *dev_id)
{
	struct machxo2_i2c *i2c = dev_id;
	u8 irq_stat = getreg(i2c, I2C_IRQ);

	if (!irq_stat)
		return IRQ_NONE;

	machxo2_process(i2c);
	setreg(i2c, I2C_IRQ, irq_stat); /* clear IRQ */

	if (i2c->state == STATE_DONE)
		wake_up(&i2c->wait);

	return IRQ_HANDLED;
}


static void machxo2_wait_irq(struct machxo2_i2c *i2c)
{
	u8 irq_stat, stat;
	int timeout = 0;

	do {
		DELAY(i2c);
		irq_stat = getreg(i2c, I2C_IRQ);
		stat = getreg(i2c, I2C_SR);
	} while (!(irq_stat & I2C_SR_TRRDY) && timeout++ < MAX_RETRIES);

	stat = getreg(i2c, I2C_SR);

	setreg(i2c, I2C_IRQ, irq_stat);
	if (stat & I2C_SR_TRRDY) {
		machxo2_process(i2c);
	} else if (stat & (I2C_SR_ARBL | I2C_SR_TROE)) {
		i2c->state = STATE_DONE;
		i2c->error = -EIO;
		dev_dbg(&i2c->adap.dev,
			"irq_stat %02x, stat %02x, retry %d\n",
			irq_stat, stat, timeout);
		setcmdr(i2c, I2C_CMDR_STO);
	} else {
		i2c->state = STATE_DONE;
		i2c->error = -ETIMEDOUT;
		if (stat & I2C_SR_BUSY) {
			setcmdr(i2c, I2C_CMDR_STO);
			DELAY(i2c);
		}
		setcmdr(i2c, 0);
	}
}

static void wait_not_busy(struct machxo2_i2c *i2c)
{
	int timeout = 0;
	u8 stat, irq_stat;

	while (timeout < MAX_RETRIES) {
		stat = getreg(i2c, I2C_SR);
		if ((stat & (I2C_SR_BUSY | I2C_SR_TIP)) == 0)
			break;
		DELAY(i2c);
		timeout++;
	}
	irq_stat = getreg(i2c, I2C_IRQ);
	if (irq_stat)
		setreg(i2c, I2C_IRQ, irq_stat);

	if (stat & I2C_SR_TROE || timeout >= MAX_RETRIES) { /* reset controller */
		setreg(i2c, I2C_CR, 0);
		DELAY(i2c);
		setreg(i2c, I2C_CR, I2C_CR_I2CEN);
	}
}

static int machxo2_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct machxo2_i2c *i2c = i2c_get_adapdata(adap);

	i2c->msg = msgs;
	i2c->pos = 0;
	i2c->nmsgs = num;
	i2c->state = STATE_START;
	i2c->error = 0;

	dev_dbg(&adap->dev, "new msg: addr %02x, flags %x, nmsgs %d, len %d\n",
		msgs->addr, msgs->flags, num, msgs[0].len);

	wait_not_busy(i2c);
	setreg(i2c, I2C_TXDR,
			(i2c->msg->addr << 1) |
			((i2c->msg->flags & I2C_M_RD) ? 1:0));

	if (!(i2c->flags & I2C_FL_NOIRQ)) {
		setcmdr(i2c, I2C_CMDR_STA|I2C_CMDR_WR);
		if (!wait_event_timeout(i2c->wait, (i2c->state == STATE_DONE), HZ))
			i2c->error = -ETIMEDOUT;
	} else {
		unsigned long flags;
		preempt_disable();
		local_irq_save(flags);
		local_irq_disable();
		setcmdr(i2c, I2C_CMDR_STA|I2C_CMDR_WR);
		while (i2c->state != STATE_DONE)
			machxo2_wait_irq(i2c);
		local_irq_restore(flags);
		preempt_enable();
	}
	wait_not_busy(i2c);

	if (i2c->error)
		dev_dbg(&adap->dev, "end msg: error %d, nmsgs %d, pos %d\n",
			i2c->error, i2c->nmsgs, i2c->pos);

	return i2c->error?i2c->error:num;
}

static void machxo2_init(struct machxo2_i2c *i2c)
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
		prescale = (i2c->clock_khz / (2 * i2c->speed) + 1) >> 1;
		if (prescale & ~0x3ff)
			dev_warn(&i2c->adap.dev,
				"i2x-machxo2: Prescale too high: %d\n", prescale);

		setreg(i2c, I2C_BR0, prescale & 0xff);
		setreg(i2c, I2C_BR1, (prescale >> 8) & 3);
	}

	/* Init the device */
	setreg(i2c, I2C_IRQEN, I2C_IRQ_MASK);
	setreg(i2c, I2C_CR, I2C_CR_I2CEN);
}


static u32 machxo2_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm machxo2_algorithm = {
	.master_xfer = machxo2_xfer,
	.functionality = machxo2_func,
};

static struct i2c_adapter machxo2_adapter = {
	.owner = THIS_MODULE,
	.name = "i2c-machxo2",
	.class = I2C_CLASS_DEPRECATED,
	.algo = &machxo2_algorithm,
};

static const struct of_device_id machxo2_i2c_match[] = {
	{
		.compatible = "lattice,i2c-machxo2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, machxo2_i2c_match);

static int machxo2_i2c_probe(struct platform_device *pdev)
{
	struct machxo2_i2c *i2c;
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

	machxo2_init(i2c);

	init_waitqueue_head(&i2c->wait);
	if (irq > 0) {
		dev_info(&pdev->dev, "Using irq %d, speed %dKHz.\n", irq, i2c->speed);
		ret = devm_request_irq(&pdev->dev, irq, machxo2_isr, 0,
				       pdev->name, i2c);
		if (ret) {
			dev_err(&pdev->dev, "Cannot claim IRQ\n");
			return ret;
		}
	} else {
		dev_info(&pdev->dev, "Running in poll mode, speed %dKHz, clock %dKHz.\n",
			 i2c->speed, pdata->clock_khz);
		i2c->flags |= I2C_FL_NOIRQ;
	}

	/* hook up driver to tree */
	platform_set_drvdata(pdev, i2c);
	i2c->adap = machxo2_adapter;
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

static int machxo2_i2c_remove(struct platform_device *pdev)
{
	struct machxo2_i2c *i2c = platform_get_drvdata(pdev);

	/* disable i2c logic */
	setreg(i2c, I2C_CR, 0);
	setreg(i2c, I2C_IRQEN, 0);

	/* remove adapter & data */
	i2c_del_adapter(&i2c->adap);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int machxo2_i2c_suspend(struct device *dev)
{
	struct machxo2_i2c *i2c = dev_get_drvdata(dev);

	/* make sure the device is disabled */
	setreg(i2c, I2C_CR, 0);

	return 0;
}

static int machxo2_i2c_resume(struct device *dev)
{
	struct machxo2_i2c *i2c = dev_get_drvdata(dev);

	machxo2_init(i2c);

	return 0;
}

static SIMPLE_DEV_PM_OPS(machxo2_i2c_pm, machxo2_i2c_suspend, machxo2_i2c_resume);
#define I2C_PM	(&machxo2_i2c_pm)
#else
#define I2C_PM	NULL
#endif

static struct platform_driver machxo2_i2c_driver = {
	.probe   = machxo2_i2c_probe,
	.remove  = machxo2_i2c_remove,
	.driver  = {
		.name = "i2c-machxo2",
		.of_match_table = machxo2_i2c_match,
		.pm = I2C_PM,
	},
};

module_platform_driver(machxo2_i2c_driver);

MODULE_AUTHOR("<vvlasov@dev.rtsoft.ru>");
MODULE_DESCRIPTION("Lattice MachxO2 I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-machxo2");
