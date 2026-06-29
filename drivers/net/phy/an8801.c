// SPDX-License-Identifier: GPL-2.0
/* FILE NAME:  an8801.c
 * PURPOSE:
 *      Airoha phy driver for Linux
 * NOTES:
 *
 */

/* INCLUDE FILE DECLARATIONS
 */

#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/debugfs.h>

#include "an8801.h"

MODULE_DESCRIPTION("Airoha AN8801 PHY drivers");
MODULE_AUTHOR("Airoha");
MODULE_LICENSE("GPL");

#define phydev_mdiobus(_dev)        ((_dev)->mdio.bus)
#define phydev_phy_addr(_dev) ((_dev)->mdio.addr)
#define phydev_dev(_dev) (&(_dev)->mdio.dev)

#define phydev_mdiobus_lock(phy)   (phydev_mdiobus(phy)->mdio_lock)
#define phydev_cfg(phy)            ((struct an8801r_priv *)(phy)->priv)

#define mdiobus_lock(phy)          (mutex_lock(&phydev_mdiobus_lock(phy)))
#define mdiobus_unlock(phy)        (mutex_unlock(&phydev_mdiobus_lock(phy)))

#ifdef AN8801R_DEBUGFS
#define AN8801R_DEBUGFS_PBUS_HELP_STRING \
	"\nUsage: echo w [pbus_reg] [value] > /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/pbus_reg_op" \
	"\n       echo r [pbus_reg] > /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/pbus_op" \
	"\nRead example: 0x10000054" \
	"\necho r 0x10000054> /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/pbus_reg_op" \
	"\nWrite example: Register 0x10000054 0x0" \
	"\necho w 0x10000054 0x0> /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/pbus_op" \
	"\n"
#define AN8801R_DEBUGFS_MDIO_HELP_STRING \
	"\nUsage: echo cl22 w [phy_reg] [value]> /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/mdio" \
	"\n       echo cl22 r [phy_reg] > /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/mdio" \
	"\nUsage: echo cl45 w [devad] [phy_reg] [value]> /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/mdio" \
	"\n       echo cl45 r [devad] [phy_reg] > /sys/" \
	"kernel/debug/mdio-bus\':[phy_addr]/mdio" \
	"\n"
#endif

/* For reference only
 *	GPIO1    <-> LED0,
 *	GPIO2    <-> LED1,
 *	GPIO3    <-> LED2,
 */
/* User-defined.B */
#define R50_SHIFT (-7)
static const u16 r50ohm_table[] = {
127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
127, 127, 127, 127, 127, 127, 127, 127, 127, 124,
120, 116, 112, 108, 104, 100,  96,  93,  90,  86,
84,   80,  77,  74,  72,  68,  65,  64,  61,  59,
56,   54,  52,  48,  48,  45,  43,  40,  39,  36,
35,   32,  32,  30,  28,  26,  24,  23,  21,  20,
18,   16,  16,  14
};

static const u16 r50ohm_table_size = sizeof(r50ohm_table) / sizeof(u16);
static const struct AIR_LED_CFG_T led_cfg_dlt[MAX_LED_SIZE] = {
//   LED Enable,          GPIO,    LED Polarity,      LED ON,    LED Blink
	/* LED0 */
	{LED_ENABLE, AIR_LED_GPIO1, AIR_ACTIVE_LOW,  AIR_LED0_ON, AIR_LED0_BLK},
	/* LED1 */
	{LED_ENABLE, AIR_LED_GPIO2, AIR_ACTIVE_HIGH, AIR_LED1_ON, AIR_LED1_BLK},
	/* LED2 */
	{LED_ENABLE, AIR_LED_GPIO3, AIR_ACTIVE_HIGH, AIR_LED2_ON, AIR_LED2_BLK},
};

static const u16 led_blink_cfg_dlt = AIR_LED_BLK_DUR_64M;
/* RGMII delay */
static const u8 rxdelay_force = FALSE;
static const u8 txdelay_force = FALSE;
static const u16 rxdelay_step = AIR_RGMII_DELAY_NOSTEP;
static const u8 rxdelay_align = FALSE;
static const u16 txdelay_step = AIR_RGMII_DELAY_NOSTEP;
/* User-defined.E */

/************************************************************************
 *                  F U N C T I O N S
 ************************************************************************/
static int __air_buckpbus_reg_write(struct phy_device *phydev, u32 addr,
				    u32 data)
{
	int err = 0;

	err = __phy_write(phydev, 0x1F, 4);
	if (err)
		return err;

	err |= __phy_write(phydev, 0x10, 0);
	err |= __phy_write(phydev, 0x11, (u16)(addr >> 16));
	err |= __phy_write(phydev, 0x12, (u16)(addr & 0xffff));
	err |= __phy_write(phydev, 0x13, (u16)(data >> 16));
	err |= __phy_write(phydev, 0x14, (u16)(data & 0xffff));
	err |= __phy_write(phydev, 0x1F, 0);

	return err;
}

static int __air_buckpbus_reg_read(struct phy_device *phydev, u32 addr,
				   u32 *data)
{
	int err = 0;
	u32 data_h, data_l;

	err = __phy_write(phydev, 0x1F, 4);
	if (err)
		return err;

	err |= __phy_write(phydev, 0x10, 0);
	err |= __phy_write(phydev, 0x15, (u16)(addr >> 16));
	err |= __phy_write(phydev, 0x16, (u16)(addr & 0xffff));
	data_h = __phy_read(phydev, 0x17);
	data_l = __phy_read(phydev, 0x18);
	err |= __phy_write(phydev, 0x1F, 0);
	if (err)
		return err;

	*data = ((data_h & 0xffff) << 16) | (data_l & 0xffff);
	return 0;
}

static int __air_buckpbus_reg_modify(struct phy_device *phydev, u32 addr,
				     u32 mask, u32 set)
{
	int err = 0;
	u32 data_h, data_l, data_old, data_new;

	err = __phy_write(phydev, 0x1F, 4);
	if (err)
		return err;
	err |= __phy_write(phydev, 0x10, 0);
	err |= __phy_write(phydev, 0x15, (u16)(addr >> 16));
	err |= __phy_write(phydev, 0x16, (u16)(addr & 0xffff));
	data_h = __phy_read(phydev, 0x17);
	data_l = __phy_read(phydev, 0x18);
	if (err) {
		__phy_write(phydev, 0x1F, 0);
		return err;
	}

	data_old = data_l | (data_h << 16);
	data_new = (data_old & ~mask) | set;
	if (data_new == data_old)
		return __phy_write(phydev, 0x1F, 0);

	err |= __phy_write(phydev, 0x11, (u16)(addr >> 16));
	err |= __phy_write(phydev, 0x12, (u16)(addr & 0xffff));
	err |= __phy_write(phydev, 0x13, (u16)(data_new >> 16));
	err |= __phy_write(phydev, 0x14, (u16)(data_new & 0xffff));
	err |= __phy_write(phydev, 0x1F, 0);

	return err;
}

static int air_buckpbus_reg_write(struct phy_device *phydev, u32 addr, u32 data)
{
	int err = 0;

	mdiobus_lock(phydev);
	err = __air_buckpbus_reg_write(phydev, addr, data);
	mdiobus_unlock(phydev);

	return err;
}

static int air_buckpbus_reg_read(struct phy_device *phydev, u32 addr, u32 *data)
{
	int err;

	mdiobus_lock(phydev);
	err = __air_buckpbus_reg_read(phydev, addr, data);
	mdiobus_unlock(phydev);

	return err;
}

static int air_buckpbus_reg_modify(struct phy_device *phydev, u32 addr,
				   u32 mask, u32 set)
{
	int err = 0;

	mdiobus_lock(phydev);
	err = __air_buckpbus_reg_modify(phydev, addr, mask, set);
	mdiobus_unlock(phydev);

	return err;
}

static int __air_read_mmd(struct phy_device *phydev, int devad, u32 regnum)
{
	int val;
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;

	__mdiobus_write(bus, phy_addr, MII_MMD_CTRL, devad);
	__mdiobus_write(bus, phy_addr, MII_MMD_DATA, regnum);
	__mdiobus_write(bus, phy_addr, MII_MMD_CTRL,
			devad | MII_MMD_CTRL_NOINCR);

	val = __mdiobus_read(bus, phy_addr, MII_MMD_DATA);

	return val;
}

static int __air_write_mmd(struct phy_device *phydev, int devad, u32 regnum, u16 val)
{
	int ret;
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;

	__mdiobus_write(bus, phy_addr, MII_MMD_CTRL, devad);
	__mdiobus_write(bus, phy_addr, MII_MMD_DATA, regnum);
	__mdiobus_write(bus, phy_addr, MII_MMD_CTRL,
			devad | MII_MMD_CTRL_NOINCR);

	__mdiobus_write(bus, phy_addr, MII_MMD_DATA, val);

	ret = 0;

	return ret;
}

static int __air_modify_mmd(struct phy_device *phydev, int devad, u32 regnum,
			    u16 mask, u16 set)
{
	int new, ret;

	ret = __air_read_mmd(phydev, devad, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = __air_write_mmd(phydev, devad, regnum, new);

	return ret < 0 ? ret : 0;
}

static int air_efuse_read(struct phy_device *phydev, u32 addr, u32 *data)
{
	int ret = 0;

	ret |= air_buckpbus_reg_write(phydev, 0x10004034, 0xde7502bc);
	ret |= air_buckpbus_reg_write(phydev, 0x1000408c, 0x78d39bf1);
	ret |= air_buckpbus_reg_write(phydev, 0x10004004, addr);
	ret |= air_buckpbus_reg_write(phydev, 0x10004000, 1);
	mdelay(1);
	air_buckpbus_reg_read(phydev, 0x10004008, data);
	air_buckpbus_reg_read(phydev, 0x10004014, data);
	ret |= air_buckpbus_reg_write(phydev, 0x10004034, 0x0);
	ret |= air_buckpbus_reg_write(phydev, 0x1000408c, 0x0);

	return ret;
}

static int air_modify_mmd(struct phy_device *phydev, int devad, u32 regnum,
			  u16 mask, u16 set)
{
	int ret;

	mdiobus_lock(phydev);
	ret = __air_modify_mmd(phydev, devad, regnum, mask, set);
	mdiobus_unlock(phydev);

	return ret;
}

static int an8801r_led_set_usr_def(struct phy_device *phydev, u8 entity,
				   u16 polar, u16 on_evt, u16 blk_evt)
{
	int err;

	if (polar == AIR_ACTIVE_HIGH)
		on_evt |= LED_ON_POL;
	else
		on_evt &= ~LED_ON_POL;

	on_evt |= LED_ON_EN;

	err = phy_write_mmd(phydev, 0x1f, LED_ON_CTRL(entity), on_evt);
	if (err)
		return -1;

	return phy_write_mmd(phydev, 0x1f, LED_BLK_CTRL(entity), blk_evt);
}

static int an8801r_led_set_blink(struct phy_device *phydev, u16 blink)
{
	int err;

	err = phy_write_mmd(phydev, 0x1f, LED_BLK_DUR,
			    LED_BLINK_DURATION(blink));
	if (err)
		return err;

	return phy_write_mmd(phydev, 0x1f, LED_ON_DUR,
			     (LED_BLINK_DURATION(blink) >> 1));
}

static int an8801r_led_set_mode(struct phy_device *phydev, u8 mode)
{
	switch (mode) {
	case AIR_LED_MODE_DISABLE:
		return air_modify_mmd(phydev, 0x1f, LED_BCR,
				      (LED_BCR_EXT_CTRL | LED_BCR_CLK_EN),
				      0x0);
	case AIR_LED_MODE_USER_DEFINE:
		return air_modify_mmd(phydev, 0x1f, LED_BCR,
				      (LED_BCR_EXT_CTRL | LED_BCR_CLK_EN),
				      (LED_BCR_EXT_CTRL | LED_BCR_CLK_EN));
	default:
		break;
	}
	dev_err(phydev_dev(phydev),
		"LED mode %d is not supported\n", mode);
	return -EINVAL;
}

static int an8801r_led_set_state(struct phy_device *phydev, u8 entity, u8 state)
{
	return air_modify_mmd(phydev, 0x1f, LED_ON_CTRL(entity), LED_ON_EN,
			      (state) ? LED_ON_EN : 0x0);
}

static int an8801r_led_init(struct phy_device *phydev)
{
	struct an8801r_priv *priv = phydev_cfg(phydev);
	struct AIR_LED_CFG_T *led_cfg = priv->led_cfg;
	int ret, led_id;
	u32 data;
	u16 led_blink_cfg = priv->led_blink_cfg;

	ret = an8801r_led_set_blink(phydev, led_blink_cfg);
	if (ret != 0)
		return ret;

	ret = an8801r_led_set_mode(phydev, AIR_LED_MODE_USER_DEFINE);
	if (ret != 0) {
		dev_err(phydev_dev(phydev),
			"LED fail to set mode, ret %d !\n", ret);
		return ret;
	}

	for (led_id = AIR_LED0; led_id < MAX_LED_SIZE; led_id++) {
		ret = an8801r_led_set_state(phydev, led_id, led_cfg[led_id].en);
		if (ret != 0) {
			dev_err(phydev_dev(phydev),
				"LED fail to set LED(%d) state, ret %d !\n",
				led_id, ret);
			return ret;
		}
		if (led_cfg[led_id].en == LED_ENABLE) {
			data = BIT(led_cfg[led_id].gpio);
			ret |= air_buckpbus_reg_modify(phydev, 0x10000054, data, data);

			data = LED_GPIO_SEL(led_id, led_cfg[led_id].gpio);
			ret |= air_buckpbus_reg_modify(phydev, 0x10000058, data, data);

			data = BIT(led_cfg[led_id].gpio);
			ret |= air_buckpbus_reg_modify(phydev, 0x10000070, data, 0);

			ret |= an8801r_led_set_usr_def(phydev, led_id,
				led_cfg[led_id].pol,
				led_cfg[led_id].on_cfg,
				led_cfg[led_id].blk_cfg);
			if (ret != 0) {
				dev_err(phydev_dev(phydev),
					"Fail to set LED(%d) usr def, ret %d !\n",
					led_id, ret);
				return ret;
			}
		}
	}
	dev_info(phydev_dev(phydev), "LED initialize OK !\n");
	return 0;
}

static int an8801r_ack_interrupt(struct phy_device *phydev)
{
	u32 reg_val = 0;

	/* Reset WOL status */
	air_buckpbus_reg_write(phydev, 0x10285404, 0x102);
	air_buckpbus_reg_read(phydev, 0x10285400, &reg_val);
	air_buckpbus_reg_write(phydev, 0x10285400, 0x0);
	air_buckpbus_reg_write(phydev, 0x10285400, reg_val | 0x10);
	air_buckpbus_reg_write(phydev, 0x10285404, 0x12);
	/* Clear the interrupts by writing the reg */
	air_buckpbus_reg_write(phydev, 0x10285704, 0x1f);
	return 0;
}

static int an8801r_config_intr(struct phy_device *phydev)
{
	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		air_buckpbus_reg_write(phydev, 0x1000007c, 0x10000);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x1, 0x1);
	} else {
		air_buckpbus_reg_write(phydev, 0x1000007c, 0x0);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x1, 0);
	}
	an8801r_ack_interrupt(phydev);
	return 0;
}

static int an8801r_did_interrupt(struct phy_device *phydev)
{
	int err;
	u32 intr_cfg = 0, reg_val = 0;

	err = air_buckpbus_reg_read(phydev, 0x10285700, &intr_cfg);
	if (err)
		return err;

	err = air_buckpbus_reg_read(phydev, 0x10285704, &reg_val);
	if (err)
		return err;

	if (reg_val & 0x10)
		return 1;

	if ((intr_cfg & 0x1) && (reg_val & 0x1))
		return 1;

	return 0;
}

static irqreturn_t an8801r_handle_interrupt(struct phy_device *phydev)
{
	if (!an8801r_did_interrupt(phydev))
		return IRQ_NONE;

	an8801r_ack_interrupt(phydev);

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static void an8801r_get_wol(struct phy_device *phydev,
			    struct ethtool_wolinfo *wol)
{
	u32 reg_val = 0;

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	air_buckpbus_reg_read(phydev, 0x10285400, &reg_val);

	wol->wolopts = (reg_val & 0xE) ? WAKE_MAGIC : 0;
}

static int an8801r_set_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	struct net_device *attach_dev = phydev->attached_dev;
	u32 reg_val;

	if (wol->wolopts & WAKE_MAGIC) {
		reg_val = (attach_dev->dev_addr[2] << 24) |
			(attach_dev->dev_addr[3] << 16) |
			(attach_dev->dev_addr[4] << 8) |
			(attach_dev->dev_addr[5]);
		air_buckpbus_reg_write(phydev, 0x10285114, reg_val);
		reg_val = (attach_dev->dev_addr[0] << 8) |
			(attach_dev->dev_addr[1]);
		air_buckpbus_reg_write(phydev, 0x10285118, reg_val);
		air_buckpbus_reg_modify(phydev, 0x10285400, 0xE, 0xE);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x10, 0x10);
	} else {
		air_buckpbus_reg_modify(phydev, 0x10285400, 0xE, 0x0);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x10, 0x0);
	}
	an8801r_ack_interrupt(phydev);
	return 0;
}

static int an8801r_of_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	struct an8801r_priv *priv = phydev_cfg(phydev);
	u32 val = 0;

	if (of_find_property(of_node, "airoha,rxclk-delay", NULL)) {
		if (of_property_read_u32(of_node, "airoha,rxclk-delay",
					 &val) != 0) {
			dev_err(phydev_dev(phydev), "airoha,rxclk-delay value is invalid.");
			return -1;
		}
		if (val < AIR_RGMII_DELAY_NOSTEP ||
		    val > AIR_RGMII_DELAY_STEP_7) {
			dev_err(phydev_dev(phydev),
				"airoha,rxclk-delay value %u out of range.",
				val);
			return -1;
		}
		priv->rxdelay_force = TRUE;
		priv->rxdelay_step = val;
		priv->rxdelay_align = of_property_read_bool(of_node,
							    "airoha,rxclk-delay-align");
	}

	if (of_find_property(of_node, "airoha,txclk-delay", NULL)) {
		if (of_property_read_u32(of_node, "airoha,txclk-delay",
					 &val) != 0) {
			dev_err(phydev_dev(phydev),
				"airoha,txclk-delay value is invalid.");
			return -1;
		}
		if (val < AIR_RGMII_DELAY_NOSTEP ||
		    val > AIR_RGMII_DELAY_STEP_7) {
			dev_err(phydev_dev(phydev),
				"airoha,txclk-delay value %u out of range.",
				val);
			return -1;
		}
		priv->txdelay_force = TRUE;
		priv->txdelay_step = val;
	}

	if (of_find_property(of_node, "airoha,cko-output", NULL)) {
		if (of_property_read_u32(of_node, "airoha,cko-output",
					 &val) != 0) {
			dev_err(phydev_dev(phydev),
				"airoha,cko-output value is invalid.");
			return -1;
		}
		if (val < AIR_CKO_OUTPUT_RATE_25M ||
		    val > AIR_CKO_OUTPUT_RATE_125M) {
			dev_err(phydev_dev(phydev),
				"airoha,cko-output value %u out of range.",
				val);
			return -1;
		}
		priv->cko_output_en = TRUE;
		priv->cko_output_rate = val;
	}
	if (of_find_property(of_node, "airoha,surge", NULL)) {
		if (of_property_read_u32(of_node, "airoha,surge",
					 &val) != 0) {
			dev_err(phydev_dev(phydev), "airoha,surge value is invalid.");
			return -1;
		}
		if (val < AIR_SURGE_0R ||
		    val > AIR_SURGE_5R) {
			dev_err(phydev_dev(phydev),
				"airoha,surge value %u out of range.",
				val);
			return -1;
		}
		priv->surge = val;
	} else {
		priv->surge = AIR_SURGE_0R;
	}

	return 0;
}

static int an8801r_rgmii_rxdelay(struct phy_device *phydev, u16 delay, u8 align)
{
	u32 reg_val = delay & RGMII_DELAY_STEP_MASK;

	/* align */
	if (align) {
		reg_val |= RGMII_RXDELAY_ALIGN;
		dev_info(phydev_dev(phydev), "Rxdelay align\n");
	}
	reg_val |= RGMII_RXDELAY_FORCE_MODE;
	air_buckpbus_reg_write(phydev, 0x1021C02C, reg_val);
	reg_val = 0;
	air_buckpbus_reg_read(phydev, 0x1021C02C, &reg_val);
	dev_info(phydev_dev(phydev),
		 "Force rxdelay = %d(0x%x)\n", delay, reg_val);
	return 0;
}

static int an8801r_rgmii_txdelay(struct phy_device *phydev, u16 delay)
{
	u32 reg_val = delay & RGMII_DELAY_STEP_MASK;

	reg_val |= RGMII_TXDELAY_FORCE_MODE;
	air_buckpbus_reg_write(phydev, 0x1021C024, reg_val);
	reg_val = 0;
	air_buckpbus_reg_read(phydev, 0x1021C024, &reg_val);
	dev_info(phydev_dev(phydev),
		 "Force txdelay = %d(0x%x)\n", delay, reg_val);
	return 0;
}

static int an8801r_rgmii_delay_config(struct phy_device *phydev)
{
	struct an8801r_priv *priv = phydev_cfg(phydev);

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_RGMII_TXID:
		an8801r_rgmii_txdelay(phydev, AIR_RGMII_DELAY_STEP_4);
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		an8801r_rgmii_rxdelay(phydev, AIR_RGMII_DELAY_NOSTEP, TRUE);
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		an8801r_rgmii_txdelay(phydev, AIR_RGMII_DELAY_STEP_4);
		an8801r_rgmii_rxdelay(phydev, AIR_RGMII_DELAY_NOSTEP, TRUE);
		break;
	case PHY_INTERFACE_MODE_RGMII:
	default:
		if (priv->rxdelay_force)
			an8801r_rgmii_rxdelay(phydev, priv->rxdelay_step,
					      priv->rxdelay_align);
		if (priv->txdelay_force)
			an8801r_rgmii_txdelay(phydev, priv->txdelay_step);
		break;
	}
	return 0;
}

static int an8801r_cko_config(struct phy_device *phydev)
{
	struct an8801r_priv *priv = phydev_cfg(phydev);

	if (priv->cko_output_en) {
		if (priv->cko_output_rate == AIR_CKO_OUTPUT_RATE_125M) {
			air_buckpbus_reg_write(phydev, 0x10000194, 0x80);
			air_buckpbus_reg_write(phydev, 0x100001A4, (0x3 << 10) |
					       ((AIR_CKO_OUT_DRV & 0xF) << 4) |
					       (0x2 << 2));
		} else if (priv->cko_output_rate == AIR_CKO_OUTPUT_RATE_25M) {
			air_buckpbus_reg_write(phydev, 0x10000194, 0x0);
			air_buckpbus_reg_write(phydev, 0x100001A4, (0x0 << 10) |
					       ((AIR_CKO_OUT_DRV & 0xF) << 4) |
					       (0x2 << 2));
		}
	} else {
		air_buckpbus_reg_write(phydev, 0x100001A4, 0x3);
	}

	return 0;
}

static int findClosestNumber(const u16 *arr, u16 size, u16 target)
{
	int left = 0, right = size - 1;

	while (left <= right) {
		int mid = left + ((right - left) >> 2);

		if (arr[mid] == target)
			return mid;

		if (arr[mid] < target)
			right = mid - 1;
		else
			left = mid + 1;
	}

	if (left > size - 1)
		return (size - 1);
	else
		return ((left - 1) >= 0 ? (left - 1) : 0);
}

static int an8801r_i2mpb_config(struct phy_device *phydev)
{
	int ret = 0;
	u32 efuse_data0 = 0, efuse_data1 = 0, efuse_data2 = 0;
	u16 cl45_value = 0;
	u16 mask = 0;

	air_efuse_read(phydev, 0, &efuse_data0);
	air_efuse_read(phydev, 1, &efuse_data1);
	air_efuse_read(phydev, 2, &efuse_data2);
	dev_dbg(phydev_dev(phydev), "%s:%d efuse data0 0x%x!\n", __func__, __LINE__, efuse_data0);
	dev_dbg(phydev_dev(phydev), "%s:%d efuse data1 0x%x!\n", __func__, __LINE__, efuse_data1);
	dev_dbg(phydev_dev(phydev), "%s:%d efuse data2 0x%x!\n", __func__, __LINE__, efuse_data2);

	cl45_value = ((efuse_data0 & GENMASK(5, 0)) + 6) << 10;
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x12, GENMASK(15, 10), cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = (efuse_data1 & GENMASK(5, 0)) + 6;
	cl45_value = cl45_value | (((efuse_data2 & GENMASK(5, 0)) + 9) << 10);
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	mask = GENMASK(15, 10) | GENMASK(5, 0);
	ret = phy_modify_mmd(phydev, 0x1e, 0x16, mask, cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = (efuse_data0 & GENMASK(13, 8)) + (6 << 8);
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x17, GENMASK(13, 8), cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = ((efuse_data1 & GENMASK(15, 10)) >> 10) + 6;
	cl45_value = cl45_value | (((efuse_data2 & GENMASK(15, 10)) >> 2) + (9 << 8));
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	mask = GENMASK(13, 8) | GENMASK(5, 0);
	ret = phy_modify_mmd(phydev, 0x1e, 0x18, mask, cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = ((efuse_data0 & GENMASK(21, 16)) >> 8) + (6 << 8);
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x19, GENMASK(13, 8), cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = ((efuse_data1 & GENMASK(21, 16)) >> 16) + 6;
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x20, GENMASK(5, 0), cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = ((efuse_data0 & GENMASK(29, 24)) >> 16) + (6 << 8);
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x21, GENMASK(13, 8), cl45_value);
	if (ret < 0)
		return ret;
	cl45_value = ((efuse_data1 & GENMASK(31, 26)) >> 26) + 6;
	dev_dbg(phydev_dev(phydev), "%s:%d cl45_value 0x%x!\n", __func__, __LINE__, cl45_value);
	ret = phy_modify_mmd(phydev, 0x1e, 0x22, GENMASK(5, 0), cl45_value);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, 0x1e, 0x23, 0x883);
	ret |= phy_write_mmd(phydev, 0x1e, 0x24, 0x883);
	ret |= phy_write_mmd(phydev, 0x1e, 0x25, 0x883);
	ret |= phy_write_mmd(phydev, 0x1e, 0x26, 0x883);
	ret |= phy_write_mmd(phydev, 0x1e, 0x0, 0x100);
	ret |= phy_write_mmd(phydev, 0x1e, 0x1, 0x1bc);
	ret |= phy_write_mmd(phydev, 0x1e, 0x2, 0x1d0);
	ret |= phy_write_mmd(phydev, 0x1e, 0x3, 0x186);
	ret |= phy_write_mmd(phydev, 0x1e, 0x4, 0x202);
	ret |= phy_write_mmd(phydev, 0x1e, 0x5, 0x20e);
	ret |= phy_write_mmd(phydev, 0x1e, 0x6, 0x300);
	ret |= phy_write_mmd(phydev, 0x1e, 0x7, 0x3c0);
	ret |= phy_write_mmd(phydev, 0x1e, 0x8, 0x3d0);
	ret |= phy_write_mmd(phydev, 0x1e, 0x9, 0x317);
	ret |= phy_write_mmd(phydev, 0x1e, 0xa, 0x206);
	ret |= phy_write_mmd(phydev, 0x1e, 0xb, 0xe);
	if (ret < 0)
		return ret;

	dev_info(phydev_dev(phydev), "I2MPB Initialize OK\n");
	return ret;
}

static void update_r50_value(struct phy_device *phydev,
		      u16 *cl45_value, int pos1, int pos2)
{
	*cl45_value &= ~(0x007f << 8);
	*cl45_value |= ((r50ohm_table[pos1]) & 0x007f) << 8;
	*cl45_value &= ~(0x007f);
	*cl45_value |= (r50ohm_table[pos2]) & 0x007f;
	dev_dbg(phydev_dev(phydev), "Read: r50ohm_tx_1=%d r50ohm_tx_2=%d\n",
		r50ohm_table[pos1], r50ohm_table[pos2]);
}

static int calculate_position(int pos, int shift, int table_size)
{
	if (shift > 0)
		return (pos + shift < table_size) ? (pos + shift) : (table_size - 1);
	else
		return (pos + shift > 0) ? (pos + shift) : 0;
}

static int process_r50(struct phy_device *phydev, int reg,
		u16 *cl45_value, u16 *r50ohm_tx_a, u16 *r50ohm_tx_b)
{
	int pos1 = findClosestNumber(r50ohm_table, r50ohm_table_size, *r50ohm_tx_a);
	int pos2 = findClosestNumber(r50ohm_table, r50ohm_table_size, *r50ohm_tx_b);

	if (pos1 != -1 && pos2 != -1) {
		pos1 = calculate_position(pos1, R50_SHIFT, r50ohm_table_size);
		pos2 = calculate_position(pos2, R50_SHIFT, r50ohm_table_size);

		update_r50_value(phydev, cl45_value, pos1, pos2);
		return phy_write_mmd(phydev, 0x1e, reg, *cl45_value);
	}
	return 0;
}

static int an8801r_surge_protect_cfg(struct phy_device *phydev)
{
	int ret = 0;
	struct device *dev = phydev_dev(phydev);
	struct an8801r_priv *priv = phydev->priv;
	u16 r50ohm_tx_a = 0, r50ohm_tx_b = 0, r50ohm_tx_c = 0, r50ohm_tx_d = 0;
	u16 cl45_value = 0;
	u32 efuse_data4;

	if (priv->surge) {
		air_efuse_read(phydev, 4, &efuse_data4);
		dev_dbg(phydev_dev(phydev), "%s:%d efuse data4 0x%x!\n",
			__func__, __LINE__, efuse_data4);
		cl45_value = phy_read_mmd(phydev, 0x1e, 0x174);
		r50ohm_tx_a = efuse_data4 & 0x007f;
		r50ohm_tx_b = (efuse_data4 >> 8) & 0x007f;
		dev_dbg(phydev_dev(phydev), "Read: (0x174) value=0x%04x r50ohm_tx_a=%d r50ohm_tx_b=%d\n",
			cl45_value, r50ohm_tx_a, r50ohm_tx_b);
		ret = process_r50(phydev, 0x174, &cl45_value, &r50ohm_tx_a, &r50ohm_tx_b);
		if (ret < 0)
			return ret;
		cl45_value = phy_read_mmd(phydev, 0x1e, 0x175);
		r50ohm_tx_c = (efuse_data4 >> 16) & 0x007f;
		r50ohm_tx_d = (efuse_data4 >> 24) & 0x007f;
		dev_dbg(phydev_dev(phydev), "Read: (0x175) value=0x%04x r50ohm_tx_c=%d r50ohm_tx_d=%d\n",
			cl45_value, r50ohm_tx_c, r50ohm_tx_d);
		ret = process_r50(phydev, 0x175, &cl45_value, &r50ohm_tx_c, &r50ohm_tx_d);
		if (ret < 0)
			return ret;
		ret = an8801r_i2mpb_config(phydev);
		if (ret < 0) {
			dev_err(dev, "an8801r_i2mpb_config fail\n");
			return ret;
		}
		dev_info(dev, "surge protection mode - 5R\n");
	} else {
		dev_info(dev, "surge protection mode - 0R\n");
	}
	return ret;
}

static int an8801r_config_init(struct phy_device *phydev)
{
	int ret;

	air_buckpbus_reg_write(phydev, 0x100000C8, 0x7);
	ret = an8801r_of_init(phydev);
	if (ret < 0)
		return ret;

	phy_write_mmd(phydev, 0x1f, 0x600, 0x1e);
	phy_write_mmd(phydev, 0x1f, 0x601, 0x2);
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0x0);
	mdiobus_lock(phydev);
	__phy_write(phydev, 0x1f, 0x1);
	__phy_write(phydev, 0x14, 0x3a14);
	__phy_write(phydev, 0x1f, 0x0);
	mdiobus_unlock(phydev);

	air_buckpbus_reg_write(phydev, 0x11F808D0, 0x180);

	air_buckpbus_reg_write(phydev, 0x1021c004, 0x1);
	air_buckpbus_reg_write(phydev, 0x10270004, 0x3f);
	air_buckpbus_reg_write(phydev, 0x10270104, 0xff);
	air_buckpbus_reg_write(phydev, 0x10270204, 0xff);

	phy_write_mmd(phydev, 0x1e, 0x13, 0x4040);
	phy_write_mmd(phydev, 0x1e, 0xD8, 0x1010);
	phy_write_mmd(phydev, 0x1e, 0xD9, 0x100);
	phy_write_mmd(phydev, 0x1e, 0xDA, 0x100);

	an8801r_rgmii_delay_config(phydev);
	an8801r_cko_config(phydev);
	ret = an8801r_surge_protect_cfg(phydev);
	if (ret < 0) {
		dev_err(phydev_dev(phydev),
			"an8801r_surge_protect_cfg fail. (ret=%d)\n", ret);
		return ret;
	}

	ret = an8801r_led_init(phydev);
	if (ret != 0) {
		dev_err(phydev_dev(phydev),
			"LED initialize fail, ret %d !\n", ret);
		return ret;
	}
	dev_info(phydev_dev(phydev), "AN8801R Initialize OK ! (%s)\n",
		 AN8801R_DRIVER_VERSION);
	return 0;
}

#ifdef AN8801R_DEBUGFS
static ssize_t an8801r_mdio_write(struct file *file, const char __user *ptr,
				  size_t len, loff_t *off)
{
	struct phy_device *phydev = file->private_data;
	char buf[64], param1[32], param2[32];
	int count = len, ret = 0;
	unsigned int reg, devad, val;
	u16 reg_val;

	memset(buf, 0, 64);
	memset(param1, 0, 32);
	memset(param2, 0, 32);

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ptr, len))
		return -EFAULT;

	ret = sscanf(buf, "%s %s", param1, param2);
	if (ret < 0)
		return ret;

	if (!strncmp("cl22", param1, strlen("cl22"))) {
		if (!strncmp("w", param2, strlen("w"))) {
			if (sscanf(buf, "cl22 w %x %x", &reg, &val) == -1)
				return -EFAULT;
			pr_notice("\nphy=0x%x, reg=0x%x, val=0x%x\n",
				  phydev_phy_addr(phydev), reg, val);

			ret = phy_write(phydev, reg, val);
			if (ret < 0)
				return ret;
			pr_notice("\nphy=0x%x, reg=0x%x, val=0x%x confirm..\n",
				  phydev_phy_addr(phydev), reg,
				  phy_read(phydev, reg));
		} else if (!strncmp("r", param2, strlen("r"))) {
			if (sscanf(buf, "cl22 r %x", &reg) == -1)
				return -EFAULT;
			pr_notice("\nphy=0x%x, reg=0x%x, val=0x%x\n",
				  phydev_phy_addr(phydev), reg,
				  phy_read(phydev, reg));
		} else {
			pr_notice(AN8801R_DEBUGFS_MDIO_HELP_STRING);
			return -EINVAL;
		}
	} else if (!strncmp("cl45", param1, strlen("cl45"))) {
		if (!strncmp("w", param2, strlen("w"))) {
			if (sscanf(buf, "cl45 w %x %x %x", &devad, &reg, &val) == -1)
				return -EFAULT;
			pr_notice("\nphy=0x%x, devad=0x%x, reg=0x%x, val=0x%x\n",
				  phydev_phy_addr(phydev), devad, reg, val);

			ret = phy_write_mmd(phydev, devad, reg, val);
			if (ret < 0)
				return ret;
			reg_val = phy_read_mmd(phydev, devad, reg);
			pr_notice("\nphy=0x%x, devad=0x%x, reg=0x%x, val=0x%x confirm..\n",
				  phydev_phy_addr(phydev), devad, reg, reg_val);
		} else if (!strncmp("r", param2, strlen("r"))) {
			if (sscanf(buf, "cl45 r %x %x", &devad, &reg) == -1)
				return -EFAULT;
			reg_val = phy_read_mmd(phydev, devad, reg);
			pr_notice("\nphy=0x%x, devad=0x%x, reg=0x%x, val=0x%x\n",
				  phydev_phy_addr(phydev), devad, reg, reg_val);
		} else {
			pr_notice(AN8801R_DEBUGFS_MDIO_HELP_STRING);
			return -EINVAL;
		}
	} else {
		pr_notice(AN8801R_DEBUGFS_MDIO_HELP_STRING);
		return -EINVAL;
	}

	return count;
}

static int an8801r_counter_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;
	int ret = 0;
	u32 pkt_cnt = 0;

	seq_puts(seq, "|\t<<EFIFO COUNTER>>\n");
	seq_puts(seq, "| Rx from Line side_S     :");
	air_buckpbus_reg_read(phydev, 0x10270030, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from Line side_E     :");
	air_buckpbus_reg_read(phydev, 0x10270034, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to System side_S     :");
	air_buckpbus_reg_read(phydev, 0x10270038, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to System side_E     :");
	air_buckpbus_reg_read(phydev, 0x1027003C, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from System side_S   :");
	air_buckpbus_reg_read(phydev, 0x10270020, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from System side_E   :");
	air_buckpbus_reg_read(phydev, 0x10270024, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to Line side_S       :");
	air_buckpbus_reg_read(phydev, 0x10270028, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to Line side_E       :");
	air_buckpbus_reg_read(phydev, 0x1027002C, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);

	ret = air_buckpbus_reg_write(phydev, 0x1027001C, 0x3);
	if (ret < 0)
		return ret;

	seq_puts(seq, "|\t<<LS Counter>>\n");
	ret = phy_write(phydev, 0x1f, 1);
	if (ret < 0)
		return ret;
	seq_puts(seq, "| Rx from Line side       :");
	pkt_cnt = phy_read(phydev, 0x12) & 0x7fff;
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx Error from Line side :");
	pkt_cnt = phy_read(phydev, 0x17) & 0xff;
	seq_printf(seq, "%010u |\n", pkt_cnt);

	ret = phy_write(phydev, 0x1f, 0);
	if (ret < 0)
		return ret;
	ret = phy_write(phydev, 0x1f, 0x52B5);
	if (ret < 0)
		return ret;
	ret = phy_write(phydev, 0x10, 0xBF92);
	if (ret < 0)
		return ret;

	seq_puts(seq, "| Tx to Line side         :");
	pkt_cnt = (phy_read(phydev, 0x11) & 0x7ffe) >> 1;
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx Error to Line side   :");
	pkt_cnt = phy_read(phydev, 0x12);
	pkt_cnt &= 0x7f;
	seq_printf(seq, "%010u |\n\n", pkt_cnt);
	ret = phy_write(phydev, 0x1f, 0);
	if (ret < 0)
		return ret;

	return ret;
}

static int an8801r_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, an8801r_counter_show, inode->i_private);
}

static int an8801r_debugfs_pbus_help(void)
{
	pr_notice(AN8801R_DEBUGFS_PBUS_HELP_STRING);
	return 0;
}

static ssize_t an8801r_debugfs_pbus(struct file *file,
				    const char __user *buffer, size_t count,
				    loff_t *data)
{
	struct phy_device *phydev = file->private_data;
	char buf[64];
	int ret = 0;
	unsigned int reg;
	u32 val = 0;

	memset(buf, 0, 64);

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (buf[0] == 'w') {
		if (sscanf(buf, "w %x %x", &reg, &val) == -1)
			return -EFAULT;

		pr_notice("\nphy=0x%x, reg=0x%x, val=0x%x\n",
			  phydev_phy_addr(phydev), reg, val);

		ret = air_buckpbus_reg_write(phydev, reg, val);
		if (ret < 0)
			return ret;

		val = 0;
		air_buckpbus_reg_read(phydev, reg, &val);
		pr_notice("\nphy=%d, reg=0x%x, val=0x%x confirm..\n",
			  phydev_phy_addr(phydev), reg, val);
	} else if (buf[0] == 'r') {
		if (sscanf(buf, "r %x", &reg) == -1)
			return -EFAULT;

		air_buckpbus_reg_read(phydev, reg, &val);
		pr_notice("\nphy=0x%x, reg=0x%x, val=0x%x\n",
			  phydev_phy_addr(phydev), reg, val);
	} else if (buf[0] == 'h') {
		an8801r_debugfs_pbus_help();
	}

	return count;
}

static int an8801r_info_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;
	u32 pbus_data = 0;
	int reg = 0;

	seq_puts(seq, "\t<<AIR AN8801R Info>>\n");
	air_buckpbus_reg_read(phydev, 0x10005004, &pbus_data);
	seq_printf(seq, "| Product Version : E%d\n", pbus_data);
	seq_printf(seq, "| Driver Version  : %s\n", AN8801R_DRIVER_VERSION);
	air_buckpbus_reg_read(phydev, 0x10000094, &pbus_data);
	seq_printf(seq, "| RG_HW_STRAP     : 0x%08x\n", pbus_data);
	for (reg = MII_BMCR; reg <= MII_STAT1000; reg++) {
		if (reg <= MII_LPA || reg >= MII_CTRL1000)
			seq_printf(seq, "| RG_MII 0x%02x     : 0x%08x\n",
				   reg, phy_read(phydev, reg));
	}
	seq_puts(seq, "\n");
	return 0;
}

static int an8801r_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, an8801r_info_show, inode->i_private);
}

static const struct file_operations an8801r_info_fops = {
	.owner = THIS_MODULE,
	.open = an8801r_info_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations an8801r_counter_fops = {
	.owner = THIS_MODULE,
	.open = an8801r_counter_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations an8801r_debugfs_pbus_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = an8801r_debugfs_pbus,
	.llseek = noop_llseek,
};

static const struct file_operations an8801r_mdio_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = an8801r_mdio_write,
	.llseek = noop_llseek,
};

static int an8801r_debugfs_init(struct phy_device *phydev)
{
	int ret = 0;
	struct an8801r_priv *priv = phydev->priv;

	dev_info(phydev_dev(phydev), "Debugfs init start\n");
	priv->debugfs_root =
		debugfs_create_dir(dev_name(phydev_dev(phydev)), NULL);
	if (!priv->debugfs_root) {
		dev_err(phydev_dev(phydev), "Debugfs init err\n");
		ret = -ENOMEM;
	}
	debugfs_create_file(DEBUGFS_INFO, 0444,
			    priv->debugfs_root, phydev,
			    &an8801r_info_fops);
	debugfs_create_file(DEBUGFS_COUNTER, 0644,
			    priv->debugfs_root, phydev,
			    &an8801r_counter_fops);
	debugfs_create_file(DEBUGFS_PBUS_OP, S_IFREG | 0200,
			    priv->debugfs_root, phydev,
			    &an8801r_debugfs_pbus_fops);
	debugfs_create_file(DEBUGFS_MDIO, S_IFREG | 0200,
			    priv->debugfs_root, phydev,
			    &an8801r_mdio_fops);
	return ret;
}

static void air_debugfs_remove(struct phy_device *phydev)
{
	struct an8801r_priv *priv = phydev->priv;

	debugfs_remove_recursive(priv->debugfs_root);
	priv->debugfs_root = NULL;
}
#endif /*AN8801R_DEBUGFS*/

static int an8801r_phy_probe(struct phy_device *phydev)
{
	u32 reg_val, phy_id, led_id;
	struct device *dev = &phydev->mdio.dev;
	struct an8801r_priv *priv = NULL;
#ifdef AN8801R_DEBUGFS
	int ret = 0;
#endif

	reg_val = phy_read(phydev, 2);
	phy_id = reg_val << 16;
	reg_val = phy_read(phydev, 3);
	phy_id |= reg_val;
	dev_info(phydev_dev(phydev), "PHY-ID = %x\n", phy_id);

	if (phy_id != AN8801R_PHY_ID) {
		dev_err(phydev_dev(phydev),
			"AN8801R can't be detected.\n");
		return -1;
	}

	priv = devm_kzalloc(dev, sizeof(struct an8801r_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	for (led_id = AIR_LED0; led_id < MAX_LED_SIZE; led_id++)
		priv->led_cfg[led_id] = led_cfg_dlt[led_id];

	priv->led_blink_cfg  = led_blink_cfg_dlt;
	priv->rxdelay_force  = rxdelay_force;
	priv->txdelay_force  = txdelay_force;
	priv->rxdelay_step   = rxdelay_step;
	priv->rxdelay_align  = rxdelay_align;
	priv->txdelay_step   = txdelay_step;

	phydev->priv = priv;

#ifdef AN8801R_DEBUGFS
	ret = an8801r_debugfs_init(phydev);
	if (ret < 0) {
		dev_info(phydev_dev(phydev), "AN8801R debugfs init failed\n");
		air_debugfs_remove(phydev);
		kfree(priv);
		return ret;
	}
#endif
	return 0;
}

static void an8801r_phy_remove(struct phy_device *phydev)
{
	struct an8801r_priv *priv = (struct an8801r_priv *)phydev->priv;

#ifdef AN8801R_DEBUGFS
	air_debugfs_remove(phydev);
#endif
	kfree(priv);
	phydev->priv = NULL;
}

static int an8801r_read_status(struct phy_device *phydev)
{
	int ret, prespeed = phydev->speed;

	ret = genphy_read_status(phydev);
	if (phydev->link == LINK_DOWN) {
		prespeed = 0;
		phydev->speed = 0;
	}
	if (prespeed != phydev->speed && phydev->link == LINK_UP) {
		prespeed = phydev->speed;
		dev_dbg(phydev_dev(phydev), "AN8801R SPEED %d\n", prespeed);
		if (prespeed == SPEED_1000)
			air_buckpbus_reg_modify(phydev, 0x10005054, BIT(0), BIT(0));
		else
			air_buckpbus_reg_modify(phydev, 0x10005054, BIT(0), 0);
	}
	return ret;
}

static struct phy_driver airoha_driver[] = {
	{
		.phy_id         = AN8801R_PHY_ID,
		.name           = "Airoha AN8801R",
		.phy_id_mask    = 0x0ffffff0,
		.features       = PHY_GBIT_FEATURES,
		.config_init    = an8801r_config_init,
		.config_aneg    = genphy_config_aneg,
		.probe          = an8801r_phy_probe,
		.remove         = an8801r_phy_remove,
		.read_status    = an8801r_read_status,
		.config_intr    = an8801r_config_intr,
		.handle_interrupt = an8801r_handle_interrupt,
		.set_wol        = an8801r_set_wol,
		.get_wol        = an8801r_get_wol,
		.suspend        = genphy_suspend,
		.resume         = genphy_resume,
	}
};

module_phy_driver(airoha_driver);

static struct mdio_device_id __maybe_unused airoha_tbl[] = {
	{ AN8801R_PHY_ID, 0x0ffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, airoha_tbl);
