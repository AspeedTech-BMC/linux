// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ASPEED Always-On RTC (AONRTC) driver
 *
 * Reference: AST2705 Datasheet §72, V0.1
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/bcd.h>
#include <linux/pm_wakeirq.h>
#include <linux/reset.h>

/* Register offsets */
#define AONRTC_VERSION		0x00
#define AONRTC_CONFIG		0x04
#define AONRTC_CTRL		0x08
#define AONRTC_ALARM_MASK	0x0C
#define AONRTC_IRQ_CTRL		0x10
#define AONRTC_STATUS		0x14
#define AONRTC_SECONDS		0x18
#define AONRTC_MINUTES		0x1C
#define AONRTC_HOURS		0x20
#define AONRTC_WDAY		0x24
#define AONRTC_DAYS		0x28
#define AONRTC_MONTHS		0x2C
#define AONRTC_YEAR_LSB		0x30
#define AONRTC_YEAR_MSB		0x34
#define AONRTC_LOAD_SECONDS	0x38
#define AONRTC_LOAD_MINUTES	0x3C
#define AONRTC_LOAD_HOURS	0x40
#define AONRTC_LOAD_WDAYS	0x44
#define AONRTC_LOAD_DAYS	0x48
#define AONRTC_LOAD_MONTHS	0x4C
#define AONRTC_LOAD_YEAR_LSB	0x50
#define AONRTC_LOAD_YEAR_MSB	0x54
#define AONRTC_BATT_TIME1	0x58
#define AONRTC_BATT_TIME2	0x5C
#define AONRTC_BATT_TIME3	0x60
#define AONRTC_BATT_TIME4	0x64

/* CONFIG register bits */
#define AONRTC_CFG_12HR		BIT(4)
#define AONRTC_CFG_EN		BIT(0)

/* CTRL register bits */
#define AONRTC_CTRL_CLR_BATT	BIT(5)
#define AONRTC_CTRL_CLR_CTRS	BIT(4)
#define AONRTC_CTRL_LOAD_ANA	BIT(3)
#define AONRTC_CTRL_LOAD_BATT	BIT(2)
#define AONRTC_CTRL_LOAD_ALARM	BIT(1)
#define AONRTC_CTRL_LOAD_CTRS	BIT(0)

/* ALARM_MASK register bits */
#define AONRTC_AMASK_EN		BIT(6)
#define AONRTC_AMASK_MASK_ALL	0x3f
#define AONRTC_AMASK_SEL_WDAY	BIT(5)
#define AONRTC_AMASK_WDAY	BIT(4)
#define AONRTC_AMASK_DAY	BIT(3)
#define AONRTC_AMASK_HOURS	BIT(2)
#define AONRTC_AMASK_MINUTES	BIT(1)
#define AONRTC_AMASK_SECONDS	BIT(0)

/* IRQ_CTRL register fields */
#define AONRTC_IRQ_PENDING_MASK		GENMASK(5, 4)
#define AONRTC_IRQ_PENDING_SHIFT	4
#define AONRTC_IRQ_CLEAR_MASK		GENMASK(3, 2)
#define AONRTC_IRQ_CLEAR_SHIFT		2
#define AONRTC_IRQ_MASK_MASK		GENMASK(1, 0)
#define AONRTC_IRQ_BATT			BIT(1)
#define AONRTC_IRQ_DATETIME		BIT(0)
#define AONRTC_IRQ_ALL			(AONRTC_IRQ_BATT | AONRTC_IRQ_DATETIME)

/*
 * The spec says hold I_RESET for "at least 120s" but the hardware description
 * states 3 XTAL (32kHz) cycles are sufficient for a full counter reset.
 * We honour the 3-cycle minimum (~100 µs) here; if board errata require longer
 * the delay can be increased.
 */
#define AONRTC_RESET_DELAY_US	200

struct aspeed_aonrtc {
	void __iomem		*base;
	struct rtc_device	*rtc;
	spinlock_t		lock;	/* protects register access and alrm_time */
	int			irq0;
	struct rtc_time		alrm_time;	/* shadow: LOAD regs shared with set_time */
};

static inline u32 aonrtc_read(struct aspeed_aonrtc *priv, u32 reg)
{
	return readl(priv->base + reg);
}

static inline void aonrtc_write(struct aspeed_aonrtc *priv, u32 reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline void aonrtc_set_bits(struct aspeed_aonrtc *priv, u32 reg, u32 bits)
{
	aonrtc_write(priv, reg, aonrtc_read(priv, reg) | bits);
}

static inline void aonrtc_clr_bits(struct aspeed_aonrtc *priv, u32 reg, u32 bits)
{
	aonrtc_write(priv, reg, aonrtc_read(priv, reg) & ~bits);
}

/* ------------------------------------------------------------------ */
/*  get_time / set_time                                                */
/* ------------------------------------------------------------------ */

static int aspeed_aonrtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	u32 year_msb, year_lsb, sec;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	/*
	 * Read all fields then re-check SECONDS to detect a rollover that
	 * occurred between the first and last readl().  Retry if so — the
	 * window is a few microseconds so convergence is nearly immediate.
	 */
	do {
		sec         = bcd2bin(aonrtc_read(priv, AONRTC_SECONDS) & 0xFF);
		tm->tm_min  = bcd2bin(aonrtc_read(priv, AONRTC_MINUTES) & 0xFF);
		tm->tm_hour = bcd2bin(aonrtc_read(priv, AONRTC_HOURS)   & 0xFF);
		tm->tm_wday = bcd2bin(aonrtc_read(priv, AONRTC_WDAY)    & 0x07);
		tm->tm_mday = bcd2bin(aonrtc_read(priv, AONRTC_DAYS)    & 0xFF);
		tm->tm_mon  = bcd2bin(aonrtc_read(priv, AONRTC_MONTHS)  & 0xFF) - 1;
		year_lsb    = bcd2bin(aonrtc_read(priv, AONRTC_YEAR_LSB) & 0xFF);
		year_msb    = bcd2bin(aonrtc_read(priv, AONRTC_YEAR_MSB) & 0xFF);
	} while (sec != bcd2bin(aonrtc_read(priv, AONRTC_SECONDS) & 0xFF));

	tm->tm_sec = sec;

	spin_unlock_irqrestore(&priv->lock, flags);

	/* Hardware stores year as two 2-digit pairs: MSB="20", LSB="26" → 2026 */
	tm->tm_year = (year_msb * 100 + year_lsb) - 1900;

	return rtc_valid_tm(tm);
}

static int aspeed_aonrtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	unsigned long flags;
	int year;

	year = tm->tm_year + 1900;

	spin_lock_irqsave(&priv->lock, flags);

	/* Pause counters before loading */
	aonrtc_clr_bits(priv, AONRTC_CONFIG, AONRTC_CFG_EN);
	udelay(AONRTC_RESET_DELAY_US);

	/* Value are all used decimal */
	aonrtc_write(priv, AONRTC_LOAD_SECONDS,  bin2bcd(tm->tm_sec));
	aonrtc_write(priv, AONRTC_LOAD_MINUTES,  bin2bcd(tm->tm_min));
	aonrtc_write(priv, AONRTC_LOAD_HOURS,    bin2bcd(tm->tm_hour));
	aonrtc_write(priv, AONRTC_LOAD_WDAYS,    bin2bcd(tm->tm_wday));
	aonrtc_write(priv, AONRTC_LOAD_DAYS,     bin2bcd(tm->tm_mday));
	aonrtc_write(priv, AONRTC_LOAD_MONTHS,   bin2bcd(tm->tm_mon + 1));
	aonrtc_write(priv, AONRTC_LOAD_YEAR_LSB, bin2bcd(year % 100));
	aonrtc_write(priv, AONRTC_LOAD_YEAR_MSB, bin2bcd(year / 100));

	/* Activate the loaded values */
	aonrtc_set_bits(priv, AONRTC_CTRL, AONRTC_CTRL_LOAD_CTRS);

	/* Resume counters */
	aonrtc_set_bits(priv, AONRTC_CONFIG, AONRTC_CFG_EN);

	spin_unlock_irqrestore(&priv->lock, flags);

	/* Wait for counters to stabilise — no register access needed under lock */
	udelay(AONRTC_RESET_DELAY_US);

	return 0;
}

/* ------------------------------------------------------------------ */
/*  Alarm                                                              */
/* ------------------------------------------------------------------ */

static int aspeed_aonrtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	u32 mask, irq;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	/*
	 * The LOAD registers are a single staging buffer shared between
	 * set_time() and set_alarm(); reading them back after a set_time()
	 * call would return time data, not alarm data.  Use the software
	 * shadow written by set_alarm() instead.
	 */
	alrm->time = priv->alrm_time;

	mask = aonrtc_read(priv, AONRTC_ALARM_MASK);
	irq  = aonrtc_read(priv, AONRTC_IRQ_CTRL);

	spin_unlock_irqrestore(&priv->lock, flags);

	alrm->enabled = !!(mask & AONRTC_AMASK_EN);
	alrm->pending = !!((irq & AONRTC_IRQ_PENDING_MASK) >> AONRTC_IRQ_PENDING_SHIFT
			   & AONRTC_IRQ_DATETIME);

	return 0;
}

static int aspeed_aonrtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	unsigned long flags;
	u32 mask;

	spin_lock_irqsave(&priv->lock, flags);

	priv->alrm_time = alrm->time;

	/* Value are all used decimal */
	aonrtc_write(priv, AONRTC_LOAD_SECONDS,  bin2bcd(alrm->time.tm_sec));
	aonrtc_write(priv, AONRTC_LOAD_MINUTES,  bin2bcd(alrm->time.tm_min));
	aonrtc_write(priv, AONRTC_LOAD_HOURS,    bin2bcd(alrm->time.tm_hour));
	aonrtc_write(priv, AONRTC_LOAD_DAYS,     bin2bcd(alrm->time.tm_mday));
	aonrtc_write(priv, AONRTC_LOAD_WDAYS,    bin2bcd(alrm->time.tm_wday));
	aonrtc_write(priv, AONRTC_LOAD_MONTHS,   bin2bcd(alrm->time.tm_mon + 1));
	aonrtc_write(priv, AONRTC_LOAD_YEAR_LSB, bin2bcd((alrm->time.tm_year + 1900) % 100));
	aonrtc_write(priv, AONRTC_LOAD_YEAR_MSB, bin2bcd((alrm->time.tm_year + 1900) / 100));

	/*
	 * Build alarm mask: set bits for fields to IGNORE (mask=1 means skip),
	 * clear bits for fields to MATCH.
	 * select_wday=0 → day-of-month mode (rtc_wkalrm uses mday not wday).
	 * mask = AONRTC_AMASK_WDAY;
	 */
	mask = AONRTC_AMASK_WDAY;	/* Not period trigger */
	if (alrm->enabled)
		mask |= AONRTC_AMASK_EN;

	aonrtc_write(priv, AONRTC_ALARM_MASK, mask);

	/* Keep IRQ_CTRL mask consistent with alarm enabled state */
	if (alrm->enabled)
		aonrtc_clr_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_DATETIME);
	else
		aonrtc_set_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_DATETIME);
	aonrtc_set_bits(priv, AONRTC_CTRL, AONRTC_CTRL_LOAD_ALARM);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int aspeed_aonrtc_alarm_irq_enable(struct device *dev, unsigned int en)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (en) {
		aonrtc_set_bits(priv, AONRTC_ALARM_MASK, AONRTC_AMASK_EN);
		aonrtc_clr_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_DATETIME);
	} else {
		aonrtc_clr_bits(priv, AONRTC_ALARM_MASK, AONRTC_AMASK_EN);
		aonrtc_set_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_DATETIME);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

/* ------------------------------------------------------------------ */
/*  IRQ handler                                                        */
/* ------------------------------------------------------------------ */

static irqreturn_t aspeed_aonrtc_irq(int irq, void *data)
{
	struct aspeed_aonrtc *priv = data;
	u32 pending;

	spin_lock(&priv->lock);

	pending = (aonrtc_read(priv, AONRTC_IRQ_CTRL) & AONRTC_IRQ_PENDING_MASK)
		  >> AONRTC_IRQ_PENDING_SHIFT;

	if (pending & AONRTC_IRQ_DATETIME)
		aonrtc_clr_bits(priv, AONRTC_ALARM_MASK, AONRTC_AMASK_EN);

	if (pending & AONRTC_IRQ_BATT)
		aonrtc_set_bits(priv, AONRTC_CTRL, AONRTC_CTRL_CLR_BATT);

	/* Clear all fired IRQs */
	if (pending)
		aonrtc_set_bits(priv, AONRTC_IRQ_CTRL,
				(pending << AONRTC_IRQ_CLEAR_SHIFT) & AONRTC_IRQ_CLEAR_MASK);

	spin_unlock(&priv->lock);

	if (!pending)
		return IRQ_NONE;

	if (pending & AONRTC_IRQ_DATETIME)
		rtc_update_irq(priv->rtc, 1, RTC_IRQF | RTC_AF);

	if (pending & AONRTC_IRQ_BATT)
		dev_warn(&priv->rtc->dev, "low battery event\n");

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  Sysfs: battery boot time                                           */
/* ------------------------------------------------------------------ */

static ssize_t battery_boot_time_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aspeed_aonrtc *priv = dev_get_drvdata(dev);
	unsigned long flags;
	u64 btime;

	spin_lock_irqsave(&priv->lock, flags);
	btime  = (u64)(aonrtc_read(priv, AONRTC_BATT_TIME1) & 0xFF);
	btime |= (u64)(aonrtc_read(priv, AONRTC_BATT_TIME2) & 0xFF) << 8;
	btime |= (u64)(aonrtc_read(priv, AONRTC_BATT_TIME3) & 0xFF) << 16;
	btime |= (u64)(aonrtc_read(priv, AONRTC_BATT_TIME4) & 0x3F) << 24;
	spin_unlock_irqrestore(&priv->lock, flags);

	return sysfs_emit(buf, "%llu\n", btime);
}
static DEVICE_ATTR_RO(battery_boot_time);

static struct attribute *aspeed_aonrtc_attrs[] = {
	&dev_attr_battery_boot_time.attr,
	NULL,
};

static const struct attribute_group aspeed_aonrtc_group = {
	.attrs = aspeed_aonrtc_attrs,
};

/* ------------------------------------------------------------------ */
/*  RTC ops                                                            */
/* ------------------------------------------------------------------ */

static const struct rtc_class_ops aspeed_aonrtc_ops = {
	.read_time		= aspeed_aonrtc_read_time,
	.set_time		= aspeed_aonrtc_set_time,
	.read_alarm		= aspeed_aonrtc_read_alarm,
	.set_alarm		= aspeed_aonrtc_set_alarm,
	.alarm_irq_enable	= aspeed_aonrtc_alarm_irq_enable,
};

/* ------------------------------------------------------------------ */
/*  Platform driver                                                    */
/* ------------------------------------------------------------------ */

static int aspeed_aonrtc_probe(struct platform_device *pdev)
{
	struct aspeed_aonrtc *priv;
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	u32 cfg;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return dev_err_probe(dev, PTR_ERR(priv->base), "Can't map resource\n");

	rst = devm_reset_control_get_shared_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "Missing reset ctrl\n");

	spin_lock_init(&priv->lock);

	priv->irq0 = platform_get_irq(pdev, 0);
	if (priv->irq0 < 0)
		return priv->irq0;

	cfg = aonrtc_read(priv, AONRTC_CONFIG);

	/* Force 24-hour mode — hardware provides no way to write AM/PM */
	if (cfg & AONRTC_CFG_12HR)
		aonrtc_clr_bits(priv, AONRTC_CONFIG, AONRTC_CFG_12HR);

	/* Enable the RTC if not already running */
	if (!(cfg & AONRTC_CFG_EN))
		aonrtc_set_bits(priv, AONRTC_CONFIG, AONRTC_CFG_EN);

	/*
	 * Disable any alarm left armed by a previous boot.  The alarm time
	 * shadow (priv->alrm_time) is zero-initialised and only populated by
	 * set_alarm(), so leaving AONRTC_AMASK_EN set would cause read_alarm()
	 * to report enabled=true with an invalid zeroed time.
	 */
	aonrtc_clr_bits(priv, AONRTC_ALARM_MASK, AONRTC_AMASK_EN);
	aonrtc_set_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_DATETIME);

	platform_set_drvdata(pdev, priv);

	ret = devm_device_add_group(dev, &aspeed_aonrtc_group);
	if (ret)
		return ret;

	priv->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(priv->rtc))
		return dev_err_probe(dev, PTR_ERR(priv->rtc), "Can't allocate device\n");

	priv->rtc->ops        = &aspeed_aonrtc_ops;
	priv->rtc->range_min  = RTC_TIMESTAMP_BEGIN_2000;
	priv->rtc->range_max  = RTC_TIMESTAMP_END_2099;
	set_bit(RTC_FEATURE_ALARM, priv->rtc->features);

	ret = devm_request_irq(dev, priv->irq0, aspeed_aonrtc_irq,
			       0, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d: %d\n", priv->irq0, ret);
		return ret;
	}

	/* Unmask battery IRQ now that the handler is registered */
	aonrtc_clr_bits(priv, AONRTC_IRQ_CTRL, AONRTC_IRQ_BATT);

	dev_info(dev, "AONRTC version 0x%02x\n",
		 aonrtc_read(priv, AONRTC_VERSION) & 0xFF);

	ret = devm_rtc_register_device(priv->rtc);
	if (ret)
		return ret;

	device_init_wakeup(dev, true);
	dev_pm_set_wake_irq(dev, priv->irq0);

	return 0;
}

static void aspeed_aonrtc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_pm_clear_wake_irq(dev);
	device_init_wakeup(dev, false);
}

static const struct of_device_id aspeed_aonrtc_dt_ids[] = {
	{ .compatible = "aspeed,ast2705-aonrtc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aspeed_aonrtc_dt_ids);

static struct platform_driver aspeed_aonrtc_driver = {
	.probe	= aspeed_aonrtc_probe,
	.remove	= aspeed_aonrtc_remove,
	.driver	= {
		.name		= "aspeed-aonrtc",
		.of_match_table	= aspeed_aonrtc_dt_ids,
	},
};
module_platform_driver(aspeed_aonrtc_driver);

MODULE_DESCRIPTION("ASPEED AST2705 Always-On RTC driver");
MODULE_AUTHOR("Tommy Huang <tommy_huang@aspeedtech.com>");
MODULE_LICENSE("GPL");
