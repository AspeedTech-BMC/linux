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
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#include "an8801.h"

MODULE_DESCRIPTION("Airoha AN8801 PHY drivers");
MODULE_AUTHOR("Airoha");
MODULE_LICENSE("GPL");

#define phydev_mdiobus(phy)        ((phy)->mdio.bus)
#define phydev_mdiobus_lock(phy)   (phydev_mdiobus(phy)->mdio_lock)
#define phydev_cfg(phy)            ((struct an8801r_priv *)(phy)->priv)

#define mdiobus_lock(phy)          (mutex_lock(&phydev_mdiobus_lock(phy)))
#define mdiobus_unlock(phy)        (mutex_unlock(&phydev_mdiobus_lock(phy)))

/* For reference only
 *	GPIO1    <-> LED0,
 *	GPIO2    <-> LED1,
 *	GPIO3    <-> LED2,
 */
/* User-defined.B */
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

static int __an8801r_cl45_write(struct phy_device *phydev, int devad, u16 reg,
				u16 val)
{
	u32 addr = (AN8801R_EPHY_ADDR | AN8801R_CL22 | (devad << 18) |
				(reg << 2));

	return __air_buckpbus_reg_write(phydev, addr, val);
}

static int __an8801r_cl45_read(struct phy_device *phydev, int devad, u16 reg)
{
	u32 addr = (AN8801R_EPHY_ADDR | AN8801R_CL22 | (devad << 18) |
				(reg << 2));
	u32 data = 0;
	int err;

	err = __air_buckpbus_reg_read(phydev, addr, &data);

	return (err) ? -EINVAL : data;
}

static int an8801r_cl45_write(struct phy_device *phydev, int devad, u16 reg,
			      u16 val)
{
	int err = 0;

	mdiobus_lock(phydev);
	err = __an8801r_cl45_write(phydev, devad, reg, val);
	mdiobus_unlock(phydev);

	return err;
}

static int an8801r_cl45_read(struct phy_device *phydev, int devad, u16 reg,
			     u16 *read_data)
{
	int data = 0;

	mdiobus_lock(phydev);
	data = __an8801r_cl45_read(phydev, devad, reg);
	mdiobus_unlock(phydev);

	if (data < 0)
		return data;

	*read_data = data;

	return 0;
}

static int air_sw_reset(struct phy_device *phydev)
{
	u32 reg_value;
	u8 retry = MAX_RETRY;

	/* Software Reset PHY */
	reg_value = phy_read(phydev, MII_BMCR);
	reg_value |= BMCR_RESET;
	phy_write(phydev, MII_BMCR, reg_value);
	do {
		mdelay(10);
		reg_value = phy_read(phydev, MII_BMCR);
		retry--;
		if (retry == 0) {
			phydev_err(phydev, "Reset fail !\n");
			return -1;
		}
	} while (reg_value & BMCR_RESET);

	return 0;
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

	err = an8801r_cl45_write(phydev, 0x1f, LED_ON_CTRL(entity), on_evt);
	if (err)
		return -1;

	return an8801r_cl45_write(phydev, 0x1f, LED_BLK_CTRL(entity), blk_evt);
}

static int an8801r_led_set_blink(struct phy_device *phydev, u16 blink)
{
	int err;

	err = an8801r_cl45_write(phydev, 0x1f, LED_BLK_DUR,
				 LED_BLINK_DURATION(blink));
	if (err)
		return err;

	return an8801r_cl45_write(phydev, 0x1f, LED_ON_DUR,
				 (LED_BLINK_DURATION(blink) >> 1));
}

static int an8801r_led_set_mode(struct phy_device *phydev, u8 mode)
{
	int err;
	u16 data;

	err = an8801r_cl45_read(phydev, 0x1f, LED_BCR, &data);
	if (err)
		return -1;

	switch (mode) {
	case AIR_LED_MODE_DISABLE:
		data &= ~LED_BCR_EXT_CTRL;
		data &= ~LED_BCR_MODE_MASK;
		data |= LED_BCR_MODE_DISABLE;
		break;
	case AIR_LED_MODE_USER_DEFINE:
		data |= (LED_BCR_EXT_CTRL | LED_BCR_CLK_EN);
		break;
	}
	return an8801r_cl45_write(phydev, 0x1f, LED_BCR, data);
}

static int an8801r_led_set_state(struct phy_device *phydev, u8 entity, u8 state)
{
	u16 data;
	int err;

	err = an8801r_cl45_read(phydev, 0x1f, LED_ON_CTRL(entity), &data);
	if (err)
		return err;

	if (state)
		data |= LED_ON_EN;
	else
		data &= ~LED_ON_EN;

	return an8801r_cl45_write(phydev, 0x1f, LED_ON_CTRL(entity), data);
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
		phydev_err(phydev, "LED fail to set mode, ret %d !\n", ret);
		return ret;
	}

	for (led_id = AIR_LED0; led_id < MAX_LED_SIZE; led_id++) {
		ret = an8801r_led_set_state(phydev, led_id, led_cfg[led_id].en);
		if (ret != 0) {
			phydev_err(phydev,
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
				phydev_err(phydev,
					   "Fail to set LED(%d) usr def, ret %d !\n",
					   led_id, ret);
				return ret;
			}
		}
	}
	phydev_info(phydev, "LED initialize OK !\n");
	return 0;
}

static int an8801r_ack_interrupt(struct phy_device *phydev)
{
	u32 reg_val = 0;

	/* Reset WOL status */
	air_buckpbus_reg_write(phydev, 0x100050c4, 0x200000);
	air_buckpbus_reg_write(phydev, 0x100050c4, 0x0);
	air_buckpbus_reg_write(phydev, 0x10285404, 0x102);
	air_buckpbus_reg_read(phydev, 0x10285400, &reg_val);
	if (reg_val & 0x1E)
		air_buckpbus_reg_write(phydev, 0x10285404, 0x12);
	else
		air_buckpbus_reg_write(phydev, 0x10285404, 0x2);
	/* Clear the interrupts by writing the reg */
	air_buckpbus_reg_write(phydev, 0x10285704, 0x1f);
	return 0;
}

static int an8801r_config_intr(struct phy_device *phydev)
{
	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		air_buckpbus_reg_modify(phydev, 0x10285400, 0x10, 0x10);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x1, 0x1);
	} else {
		air_buckpbus_reg_modify(phydev, 0x10285400, 0x10, 0);
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
		air_buckpbus_reg_write(phydev, 0x1000007c, 0x10000);
		air_buckpbus_reg_modify(phydev, 0x10285400, 0xE, 0xE);
		air_buckpbus_reg_modify(phydev, 0x10285700, 0x10, 0x10);
	} else {
		air_buckpbus_reg_write(phydev, 0x1000007c, 0x0);
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
			phydev_err(phydev, "airoha,rxclk-delay value is invalid.");
			return -1;
		}
		if (val < AIR_RGMII_DELAY_NOSTEP ||
		    val > AIR_RGMII_DELAY_STEP_7) {
			phydev_err(phydev,
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
			phydev_err(phydev,
				   "airoha,txclk-delay value is invalid.");
			return -1;
		}
		if (val < AIR_RGMII_DELAY_NOSTEP ||
		    val > AIR_RGMII_DELAY_STEP_7) {
			phydev_err(phydev,
				   "airoha,txclk-delay value %u out of range.",
				   val);
			return -1;
		}
		priv->txdelay_force = TRUE;
		priv->txdelay_step = val;
	}

	return 0;
}

static int an8801r_rgmii_rxdelay(struct phy_device *phydev, u16 delay, u8 align)
{
	u32 reg_val = delay & RGMII_DELAY_STEP_MASK;

	/* align */
	if (align) {
		reg_val |= RGMII_RXDELAY_ALIGN;
		phydev_info(phydev, "Rxdelay align\n");
	}
	reg_val |= RGMII_RXDELAY_FORCE_MODE;
	air_buckpbus_reg_write(phydev, 0x1021C02C, reg_val);
	reg_val = 0;
	air_buckpbus_reg_read(phydev, 0x1021C02C, &reg_val);
	phydev_info(phydev, "Force rxdelay = %d(0x%x)\n", delay, reg_val);
	return 0;
}

static int an8801r_rgmii_txdelay(struct phy_device *phydev, u16 delay)
{
	u32 reg_val = delay & RGMII_DELAY_STEP_MASK;

	reg_val |= RGMII_TXDELAY_FORCE_MODE;
	air_buckpbus_reg_write(phydev, 0x1021C024, reg_val);
	reg_val = 0;
	air_buckpbus_reg_read(phydev, 0x1021C024, &reg_val);
	phydev_info(phydev, "Force txdelay = %d(0x%x)\n", delay, reg_val);
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

static int an8801r_config_init(struct phy_device *phydev)
{
	int ret;

	ret = an8801r_of_init(phydev);
	if (ret < 0)
		return ret;

	ret = air_sw_reset(phydev);
	if (ret < 0)
		return ret;

	an8801r_cl45_write(phydev, 0x1f, 0x600, 0x1e);
	an8801r_cl45_write(phydev, 0x1f, 0x601, 0x2);
	an8801r_cl45_write(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0x0);
	mdiobus_lock(phydev);
	__phy_write(phydev, 0x1f, 0x1);
	__phy_write(phydev, 0x14, 0x3a14);
	__phy_write(phydev, 0x19, 0x3f);
	__phy_write(phydev, 0x1f, 0x0);
	mdiobus_unlock(phydev);

	air_buckpbus_reg_write(phydev, 0x11F808D0, 0x180);

	air_buckpbus_reg_write(phydev, 0x1021c004, 0x1);
	air_buckpbus_reg_write(phydev, 0x10270004, 0x3f);
	air_buckpbus_reg_write(phydev, 0x10270104, 0xff);
	air_buckpbus_reg_write(phydev, 0x10270204, 0xff);

	air_buckpbus_reg_write(phydev, 0x100001A4, 0x3);

	an8801r_cl45_write(phydev, 0x1e, 0x13, 0x4040);
	an8801r_cl45_write(phydev, 0x1e, 0xD8, 0x1010);
	an8801r_cl45_write(phydev, 0x1e, 0xD9, 0x100);
	an8801r_cl45_write(phydev, 0x1e, 0xDA, 0x100);

	an8801r_rgmii_delay_config(phydev);

	ret = an8801r_led_init(phydev);
	if (ret != 0) {
		phydev_err(phydev, "LED initialize fail, ret %d !\n", ret);
		return ret;
	}
	phydev_info(phydev, "Initialize OK ! (%s)\n", AN8801R_DRIVER_VERSION);
	return 0;
}

static int an8801r_phy_probe(struct phy_device *phydev)
{
	u32 reg_val, phy_id, led_id;
	struct device *dev = &phydev->mdio.dev;
	struct an8801r_priv *priv = NULL;

	reg_val = phy_read(phydev, 2);
	phy_id = reg_val << 16;
	reg_val = phy_read(phydev, 3);
	phy_id |= reg_val;
	phydev_info(phydev, "PHY-ID = %x\n", phy_id);

	if (phy_id != AN8801R_PHY_ID) {
		phydev_err(phydev, "AN8801R can't be detected.\n");
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
	return 0;
}

static void an8801r_phy_remove(struct phy_device *phydev)
{
	struct an8801r_priv *priv = (struct an8801r_priv *)phydev->priv;

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
		phydev_dbg(phydev, "SPEED %d\n", prespeed);
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
		.read_mmd       = __an8801r_cl45_read,
		.write_mmd      = __an8801r_cl45_write,
	}
};

module_phy_driver(airoha_driver);

static struct mdio_device_id __maybe_unused airoha_tbl[] = {
	{ AN8801R_PHY_ID, 0x0ffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, airoha_tbl);
