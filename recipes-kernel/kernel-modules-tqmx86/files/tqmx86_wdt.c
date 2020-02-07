/*
 * Watchdog driver for TQMx86 PLD.
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * The watchdog supports power of 2 timeouts from 1 to 4096sec.
 * Once started, it cannot be stopped.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/log2.h>


/* default timeout (secs) */
#define WDT_TIMEOUT 32

static unsigned int timeout = WDT_TIMEOUT;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout,
	"Watchdog timeout in seconds. (1<=timeout<=4096, default="
				__MODULE_STRING(WDT_TIMEOUT) ")");

static void __iomem *io_base;

#define TQMX86_WDCFG	0x00 /* Watchdog Configuration Register */
#define TQMX86_WDCS	0x01 /* Watchdog Config/Status Register */


static int tqmx86_wdt_start(struct watchdog_device *wdd)
{
	iowrite8(0x81, io_base + TQMX86_WDCS);
	return 0;
}

static int tqmx86_wdt_stop(struct watchdog_device *wdd)
{
	return -EINVAL;
}

static int tqmx86_wdt_ping(struct watchdog_device *wdd)
{
	/* user land ping */
	iowrite8(0x81, io_base + TQMX86_WDCS);

	return 0;
}

static int tqmx86_wdt_set_timeout(struct watchdog_device *wdd, unsigned int new_timeout)
{
	u8 val;

	if (new_timeout < 1 || new_timeout > 4096)
		return -EINVAL;

	timeout = roundup_pow_of_two(new_timeout);
	val = ilog2(timeout) | 0x90;
	val += 3; /* vallues 0,1,2 correspond to 0.125,0.25,0.5s timeouts */
	iowrite8(val, io_base + TQMX86_WDCFG);

	wdd->timeout = timeout;

	return 0;
}

static const struct watchdog_info tqmx86_wdt_ident = {
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING,
	.identity	= "TQMx86 Watchdog",
};

static struct watchdog_ops tqmx86_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= tqmx86_wdt_start,
	.stop		= tqmx86_wdt_stop,
	.ping		= tqmx86_wdt_ping,
	.set_timeout	= tqmx86_wdt_set_timeout,
};

static struct watchdog_device tqmx86_wdt_wdd = {
	.info		= &tqmx86_wdt_ident,
	.ops		= &tqmx86_wdt_ops,
};

static int tqmx86_wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (IS_ERR(res))
		return PTR_ERR(res);
	io_base = devm_ioport_map(&pdev->dev, res->start, res->end - res->start + 1);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	if (timeout < 1 || timeout > 4096) {
		timeout = WDT_TIMEOUT;
		dev_warn(&pdev->dev,
			"timeout value must be 1<=x<=4096, using %d\n",
			timeout);
	}
	tqmx86_wdt_set_timeout(&tqmx86_wdt_wdd, timeout);

	tqmx86_wdt_wdd.parent = &pdev->dev;

	watchdog_set_nowayout(&tqmx86_wdt_wdd, 1);

	err = watchdog_register_device(&tqmx86_wdt_wdd);
	if (err)
		return err;

	dev_info(&pdev->dev,
		"TQMx86 watchdog\n");

	return 0;
}

static int tqmx86_wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&tqmx86_wdt_wdd);
	return 0;
}

static struct platform_driver tqmx86_wdt_driver = {
	.driver		= {
		.name	= "tqmx86-wdt",
	},
	.probe		= tqmx86_wdt_probe,
	.remove		= tqmx86_wdt_remove,
};

module_platform_driver(tqmx86_wdt_driver);

MODULE_AUTHOR("Vadim V.Vlasov <vvlasov@dev.rtsoft.ru>");
MODULE_DESCRIPTION("TQMx86 Watchdog");
MODULE_ALIAS("platform:tqmx86-wdt");
MODULE_LICENSE("GPL");
