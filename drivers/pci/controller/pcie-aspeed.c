// SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe host controller driver for ASPEED PCIe Bridge
 *
 */
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/kernel.h>
#include <linux/msi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/bitfield.h>

/*	PCI Host Controller registers */
#define ASPEED_PCIE_CLASS_CODE		0x04
#define ASPEED_PCIE_GLOBAL			0x30
#define ASPEED_PCIE_CFG_DIN			0x50
#define ASPEED_PCIE_CFG3			0x58
#define ASPEED_PCIE_LOCK			0x7C
#define ASPEED_PCIE_LINK			0xC0
#define ASPEED_PCIE_INT				0xC4
#define ASPEED_PCIE_LINK_STS		0xD0
/*	AST_PCIE_CFG2			0x04 */
#define PCIE_CFG_CLASS_CODE(x)	((x) << 8)
#define PCIE_CFG_REV_ID(x)		(x)
/*	PEHR10: Miscellaneous Control 10H Register */
#define DATALINK_REPORT_CAPABLE	BIT(4)
/*	PEHR14: Miscellaneous Control 14H Register */
#define HOTPLUG_CAPABLE_ENABLE	BIT(6)
#define HOTPLUG_SURPRISE_ENABLE	BIT(5)
#define ATTENTION_BUTTON_ENALBE	BIT(0)
/*	PEHR30: Miscellaneous Control 30H Register */
/* Disable RC synchronous reset when link up to link down*/
#define RC_SYNC_RESET_DISABLE	BIT(20)
#define ROOT_COMPLEX_ID(x)		((x) << 4)
#define PCIE_RC_SLOT_ENABLE		BIT(1)
/*	AST_PCIE_LOCK			0x7C */
#define PCIE_UNLOCK				0xa8
/*	AST_PCIE_LINK			0xC0 */
#define PCIE_LINK_STS			BIT(5)
/*  ASPEED_PCIE_LINK_STS	0xD0 */
#define PCIE_LINK_5G			BIT(17)
#define PCIE_LINK_2_5G			BIT(16)

/*	H2X Controller registers */
/* reg 0x08 */
#define PCIE_TX_IDLE_CLEAR		BIT(0)

/* reg 0x24 */
#define PCIE_TX_IDLE			BIT(31)

#define PCIE_STATUS_OF_TX		GENMASK(25, 24)
#define	PCIE_RC_TX_COMPLETE		0
#define	PCIE_RC_L_TX_COMPLETE	BIT(24)
#define	PCIE_RC_H_TX_COMPLETE	BIT(25)

#define PCIE_TRIGGER_TX			BIT(0)

/* reg 0x80, 0xC0 */
#define PCIE_RX_TAG_MASK		GENMASK(23, 16)
#define PCIE_RX_DMA_EN			BIT(9)
#define PCIE_RX_LINEAR			BIT(8)
#define PCIE_RX_MSI_SEL			BIT(7)
#define PCIE_RX_MSI_EN			BIT(6)
#define PCIE_1M_ADDRESS_EN		BIT(5)
#define PCIE_UNLOCK_RX_BUFF		BIT(4)
#define PCIE_RX_TLP_TAG_MATCH	BIT(3)
#define PCIE_Wait_RX_TLP_CLR	BIT(2)
#define PCIE_RC_RX_ENABLE		BIT(1)
#define PCIE_RC_ENABLE			BIT(0)

/* reg 0x88, 0xC8 : RC ISR */
#define PCIE_RC_CPLCA_ISR		BIT(6)
#define PCIE_RC_CPLUR_ISR		BIT(5)
#define PCIE_RC_RX_DONE_ISR		BIT(4)

#define PCIE_RC_INTD_ISR		BIT(3)
#define PCIE_RC_INTC_ISR		BIT(2)
#define PCIE_RC_INTB_ISR		BIT(1)
#define PCIE_RC_INTA_ISR		BIT(0)

#define MAX_MSI_HOST_IRQS		64

/* AST2700 H2X */
#define H2X_CTRL		0x00
#define H2X_BRIDGE_EN			BIT(0)
#define H2X_BRIDGE_DIRECT_EN		BIT(1)
#define H2X_CFGE_INT_STS	0x08
#define CFGE_TX_IDLE			BIT(0)
#define CFGE_RX_IDLE			BIT(1)
#define H2X_CFGI_TLP		0x20
#define H2X_CFGI_WR_DATA	0x24
#define H2X_CFGI_CTRL		0x28
#define CFGI_TLP_FIRE			BIT(0)
#define H2X_CFGI_RET_DATA	0x2C
#define H2X_CFGE_TLP_1ST	0x30
#define H2X_CFGE_TLP_NEXT	0x34
#define H2X_CFGE_CTRL		0x38
#define CFGE_TLP_FIRE			BIT(0)
#define H2X_CFGE_RET_DATA	0x3C
#define H2X_REMAP_DIRECT_ADDR	0x78

/* AST2700 PEHR */
#define PEHR_GEN_CAPABILITY	0x60
#define PORT_TPYE			GENMASK(7, 4)
#define PORT_TYPE_ROOT			BIT(2)

/* TLP configuration type 0 and type 1 */
#define CRG_READ_FMTTYPE(type)		(0x04000000 | (type << 24))
#define CRG_WRITE_FMTTYPE(type)		(0x44000000 | (type << 24))
#define CRG_PAYLOAD_SIZE		0x01 /* 1 DWORD */
#define TLP_COMP_STATUS(s)		(((s) >> 13) & 7)

struct aspeed_pcie_rc_platform {
	int (*setup)(struct platform_device *pdev);
	/* Interrupt Register Offset */
	int reg_intx_en;
	int reg_intx_sts;
	int reg_msi_en;
	int reg_msi_sts;
};

struct aspeed_pcie {
	struct pci_host_bridge *host;
	struct device *dev;
	void __iomem *reg;	//rc slot base
	struct regmap *ahbc;
	struct regmap *device;
	int domain;
	char name[10];
	u32 msi_address;
	int irq;
	u8 tx_tag;
	struct regmap *cfg;	//pciecfg
	struct regmap *pciephy; //pcie_phy
	struct reset_control *h2xrst;
	struct reset_control *perst;
	/* INTx */
	struct irq_domain *irq_domain;	//irq_domain
	// msi
	struct irq_domain *dev_domain;	//inner_domain
	struct irq_domain *msi_domain;
	struct mutex lock;  /* protect bitmap variable */
	int hotplug_event;
	struct gpio_desc *perst_ep_in;
	struct gpio_desc *perst_rc_out;
	struct gpio_desc *perst_owner;
	struct delayed_work rst_dwork;
	DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_HOST_IRQS);

	const struct aspeed_pcie_rc_platform *platform;
	bool support_msi;
};

static void aspeed_pcie_intx_ack_irq(struct irq_data *d)
{
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(d);
	int intx_en = pcie->platform->reg_intx_en;

	writel(readl(pcie->reg + intx_en) | BIT(d->hwirq), pcie->reg + intx_en);
}

static void aspeed_pcie_intx_mask_irq(struct irq_data *d)
{
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(d);
	int intx_en = pcie->platform->reg_intx_en;

	writel(readl(pcie->reg + intx_en) & ~BIT(d->hwirq), pcie->reg + intx_en);
}

static void aspeed_pcie_intx_unmask_irq(struct irq_data *d)
{
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(d);
	int intx_en = pcie->platform->reg_intx_en;

	writel(readl(pcie->reg + intx_en) | BIT(d->hwirq), pcie->reg + intx_en);
}

static struct irq_chip aspeed_intx_irq_chip = {
	.name = "ASPEED:IntX",
	.irq_ack = aspeed_pcie_intx_ack_irq,
	.irq_mask = aspeed_pcie_intx_mask_irq,
	.irq_unmask = aspeed_pcie_intx_unmask_irq,
};

static int aspeed_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_intx_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops aspeed_intx_domain_ops = {
	.map = aspeed_pcie_intx_map,
};

static void aspeed_pcie_intr_handler(struct irq_desc *desc)
{
	struct aspeed_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	const struct aspeed_pcie_rc_platform *platform = pcie->platform;
	unsigned long status;
	unsigned long intx;
	u32 bit;
	int i;

	chained_irq_enter(irqchip, desc);

	intx = readl(pcie->reg + platform->reg_intx_sts) & 0xf;
	if (intx) {
		for_each_set_bit(bit, &intx, PCI_NUM_INTX)
			generic_handle_domain_irq(pcie->irq_domain, bit);
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		for (i = 0; i < 2; i++) {
			status = readl(pcie->reg + platform->reg_msi_sts + (i * 4));
			writel(status, pcie->reg + platform->reg_msi_sts + (i * 4));
			if (!status)
				continue;

			for_each_set_bit(bit, &status, 32) {
				if (i)
					bit += 32;
				generic_handle_domain_irq(pcie->dev_domain, bit);
			}
		}
	}
	chained_irq_exit(irqchip, desc);
}

//optional : set_slot_power_limit
void aspeed_pcie_set_slot_power_limit(struct aspeed_pcie *pcie)
{
	u32 cfg_val, isr;
	int ret;

	writel(BIT(4) | readl(pcie->reg), pcie->reg);

	pcie->tx_tag %= 0x7;
	regmap_write(pcie->cfg, 0x10, 0x74000001);
	switch (pcie->domain) {
	case 0: //write for 0.8.0
		regmap_write(pcie->cfg, 0x14, 0x00400050 | (pcie->tx_tag << 8));
		break;
	case 1: //write for 0.4.0
		regmap_write(pcie->cfg, 0x14, 0x00200050 | (pcie->tx_tag << 8));
		break;
	}

	regmap_write(pcie->cfg, 0x18, 0);
	regmap_write(pcie->cfg, 0x1C, 0);
	regmap_write(pcie->cfg, 0x20, 0x1a);

	//trigger tx
	regmap_write_bits(pcie->cfg, 0x24, PCIE_TRIGGER_TX, PCIE_TRIGGER_TX);

	//wait tx idle
	ret = regmap_read_poll_timeout(pcie->cfg, 0x24, cfg_val,
				       (cfg_val & PCIE_TX_IDLE), 0, 10);
	if (ret)
		goto out;

	//write clr tx idle
	regmap_write_bits(pcie->cfg, 0x08, PCIE_TX_IDLE_CLEAR,
			  PCIE_TX_IDLE_CLEAR);

	//check tx status
	regmap_read(pcie->cfg, 0x24, &cfg_val);
	switch (cfg_val & PCIE_STATUS_OF_TX) {
	case PCIE_RC_L_TX_COMPLETE:
	case PCIE_RC_H_TX_COMPLETE:
		ret = readl_poll_timeout(pcie->reg + 0x08, isr,
					 (isr & PCIE_RC_RX_DONE_ISR), 0, 10);
		if (ret)
			dev_err(pcie->dev, "[%d] : tx timeout [%x]\n",
				pcie->domain, isr);

		writel(readl(pcie->reg + 0x08), pcie->reg + 0x08);
		break;
	}
out:
	pcie->tx_tag++;
}

static int aspeed_ast2600_rd_conf(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 *val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 bdf_offset;
	int rx_done_fail = 0;
	u32 cfg_val, isr, type = 0;
	u32 link_sts = 0;
	int ret;

	//H2X80[4] (unlock) is write-only.
	//Driver may set H2X80[4]=1 before triggering next TX config.
	writel(BIT(4) | readl(pcie->reg), pcie->reg);

	switch (pcie->domain) {
	case 0:
		if (!bus->number) {
			switch (PCI_SLOT(devfn)) {
			case 0:
			case 4:
				break;
			default:
				*val = 0xffffffff;
				return PCIBIOS_SUCCESSFUL;
			}
		}

		if (bus->number)
			type = 1;
		else
			type = 0;
		break;
	case 1:
		if (bus->number == 128) {
			switch (PCI_SLOT(devfn)) {
			case 0:
			case 8:
				break;
			default:
				*val = 0xffffffff;
				return PCIBIOS_SUCCESSFUL;
			}
		}

		if (bus->number > 128)
			type = 1;
		else
			type = 0;
		break;
	}

	dev_dbg(pcie->dev, "[%d]R:b d f [%d:%d:%d] devfn %x\n",
		pcie->domain, bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), devfn);

	if (type) {
		regmap_read(pcie->pciephy, ASPEED_PCIE_LINK, &link_sts);
		if (!(link_sts & PCIE_LINK_STS)) {
			*val = 0xffffffff;
			return PCIBIOS_SUCCESSFUL;
		}
	}

	bdf_offset = ((bus->number) << 24) | (PCI_SLOT(devfn) << 19) |
		     (PCI_FUNC(devfn) << 16) | (where & ~3);

	pcie->tx_tag %= 0x7;

	regmap_write(pcie->cfg, 0x10, 0x04000001 | (type << 24));
	regmap_write(pcie->cfg, 0x14, 0x0000200f | (pcie->tx_tag << 8));
	regmap_write(pcie->cfg, 0x18, bdf_offset);
	regmap_write(pcie->cfg, 0x1C, 0x00000000);

	//trigger tx
	regmap_write_bits(pcie->cfg, 0x24, PCIE_TRIGGER_TX, PCIE_TRIGGER_TX);

	//wait tx idle
	//todo find timeout and time period
	ret = regmap_read_poll_timeout(pcie->cfg, 0x24, cfg_val,
				       (cfg_val & PCIE_TX_IDLE), 0, 10);
	if (ret) {
		dev_err(pcie->dev, "[%d] : tx idle timeout [%x]\n",
			pcie->domain, cfg_val);
		*val = 0xffffffff;
		goto out;
	}

	//write clr tx idle
	regmap_write_bits(pcie->cfg, 0x08, PCIE_TX_IDLE_CLEAR,
			  PCIE_TX_IDLE_CLEAR);

	//check tx status
	regmap_read(pcie->cfg, 0x24, &cfg_val);

	switch (cfg_val & PCIE_STATUS_OF_TX) {
	case PCIE_RC_L_TX_COMPLETE: //domain 0
		if (pcie->domain != 0)
			dev_err(pcie->dev, "[%d] : tx complete no correct\n",
				pcie->domain);
		fallthrough;
	case PCIE_RC_H_TX_COMPLETE: //domain 1
		ret = readl_poll_timeout(pcie->reg + 0x08, isr,
					 (isr & PCIE_RC_RX_DONE_ISR), 0, 10);
		if (ret) {
			dev_err(pcie->dev, "[%d] : rx done timeout\n",
				pcie->domain);
			rx_done_fail = 1;
			*val = 0xffffffff;
		}
		if (!rx_done_fail) {
			if (readl(pcie->reg + 0x14) & BIT(13))
				*val = 0xffffffff;
			else
				*val = readl(pcie->reg + 0x0C);
		}

		writel(BIT(4) | readl(pcie->reg), pcie->reg);
		writel(readl(pcie->reg + 0x08), pcie->reg + 0x08);
		break;
	case PCIE_STATUS_OF_TX:
		*val = 0xffffffff;
		break;
	default: //read rc data
		regmap_read(pcie->cfg, 0x0C, &cfg_val);
		*val = cfg_val;
		break;
	}

	switch (size) {
	case 1:
		*val = (*val >> ((where & 3) * 8)) & 0xff;
		break;
	case 2:
		*val = (*val >> ((where & 2) * 8)) & 0xffff;
		break;
	}

	dev_dbg(pcie->dev, "R:b d f [%d:%d:%d] where:%x : %x\n",
		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), where, *val);

#ifdef CONFIG_HOTPLUG_PCI
	switch (pcie->domain) {
	case 0:
		if (where == 0x9a && bus->number == 0x0 &&
		    (PCI_SLOT(devfn) == 0x4) && (PCI_FUNC(devfn) == 0x0) &&
		    pcie->hotplug_event)
			*val |= PCI_EXP_SLTSTA_ABP;
		break;
	case 1:
		if (where == 0x9a && bus->number == 128 &&
		    (PCI_SLOT(devfn) == 0x8) && (PCI_FUNC(devfn) == 0x0) &&
		    pcie->hotplug_event)
			*val |= PCI_EXP_SLTSTA_ABP;
		break;
	}
#endif
out:
	pcie->tx_tag++;
	return PCIBIOS_SUCCESSFUL;
}

static int aspeed_ast2600_wr_conf(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 val)
{
	u32 type = 0;
	u32 shift = 8 * (where & 3);
	u32 bdf_offset;
	u8 byte_en = 0;
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 isr, cfg_val;
	int ret;

#ifdef CONFIG_HOTPLUG_PCI
	switch (pcie->domain) {
	case 0:
		if (where == 0x9a && bus->number == 0x0 &&
		    (PCI_SLOT(devfn) == 0x4) && (PCI_FUNC(devfn) == 0x0) &&
		    pcie->hotplug_event && (val & PCI_EXP_SLTSTA_ABP)) {
			pcie->hotplug_event = 0;
			return PCIBIOS_SUCCESSFUL;
		}
		break;
	case 1:
		if (where == 0x9a && bus->number == 128 &&
		    (PCI_SLOT(devfn) == 0x8) && (PCI_FUNC(devfn) == 0x0) &&
		    pcie->hotplug_event && (val & PCI_EXP_SLTSTA_ABP)) {
			pcie->hotplug_event = 0;
			return PCIBIOS_SUCCESSFUL;
		}
		break;
	}
#endif

	dev_dbg(pcie->dev, "W b d f [%d:%d:%d] : where %x : val %x\n",
		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), where, val);

	//H2X80[4] (unlock) is write-only.
	//Driver may set H2X80[4]=1 before triggering next TX config.
	writel(BIT(4) | readl(pcie->reg), pcie->reg);

	switch (size) {
	case 1:
		switch (where % 4) {
		case 0:
			byte_en = 0x1;
			break;
		case 1:
			byte_en = 0x2;
			break;
		case 2:
			byte_en = 0x4;
			break;
		case 3:
			byte_en = 0x8;
			break;
		}
		val = (val & 0xff) << shift;
		break;
	case 2:
		switch ((where >> 1) % 2) {
		case 0:
			byte_en = 0x3;
			break;
		case 1:
			byte_en = 0xc;
			break;
		}
		val = (val & 0xffff) << shift;
		break;
	default:
		byte_en = 0xf;
		break;
	}

	switch (pcie->domain) {
	case 0:
		if (bus->number)
			type = 1;
		else
			type = 0;
		break;
	case 1:
		if (bus->number > 128)
			type = 1;
		else
			type = 0;
		break;
	}

	bdf_offset = (bus->number << 24) | (PCI_SLOT(devfn) << 19) |
		     (PCI_FUNC(devfn) << 16) | (where & ~3);
	pcie->tx_tag %= 0x7;

	regmap_write(pcie->cfg, 0x10, 0x44000001 | (type << 24));
	regmap_write(pcie->cfg, 0x14,
		     0x00002000 | (pcie->tx_tag << 8) | byte_en);
	regmap_write(pcie->cfg, 0x18, bdf_offset);
	regmap_write(pcie->cfg, 0x1C, 0x00000000);
	regmap_write(pcie->cfg, 0x20, val);

	//trigger tx
	regmap_write_bits(pcie->cfg, 0x24, PCIE_TRIGGER_TX, PCIE_TRIGGER_TX);

	//wait tx idle
	//todo find timeout and time period
	ret = regmap_read_poll_timeout(pcie->cfg, 0x24, cfg_val,
				       (cfg_val & PCIE_TX_IDLE), 0, 10);
	if (ret) {
		dev_err(pcie->dev, "[%d] : tx idle timeout [%x]\n",
			pcie->domain, cfg_val);
		goto out;
	}

	//write clr tx idle
	regmap_write_bits(pcie->cfg, 0x08, PCIE_TX_IDLE_CLEAR,
			  PCIE_TX_IDLE_CLEAR);

	//check tx status
	regmap_read(pcie->cfg, 0x24, &cfg_val);

	switch (cfg_val & PCIE_STATUS_OF_TX) {
	case PCIE_RC_L_TX_COMPLETE:
	case PCIE_RC_H_TX_COMPLETE:
		ret = readl_poll_timeout(pcie->reg + 0x08, isr,
					 (isr & PCIE_RC_RX_DONE_ISR), 0, 10);
		if (ret)
			dev_err(pcie->dev, "[%d] : tx timeout\n", pcie->domain);

		writel(readl(pcie->reg + 0x08), pcie->reg + 0x08);
		break;
	}

out:
	pcie->tx_tag++;
	return PCIBIOS_SUCCESSFUL;
}

static int aspeed_ast2700_rd_conf(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 *val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 bdf_offset, status;
	u8 type;
	int ret;

	if (bus->number == 0 && devfn != 0) {
		*val = 0xffffffff;
		return PCIBIOS_SUCCESSFUL;
	}

	if (bus->number == 0) {
		/* Internal access to bridge */
		writel(0xF << 16 | (where & ~3), pcie->reg + H2X_CFGI_TLP);
		writel(CFGI_TLP_FIRE, pcie->reg + H2X_CFGI_CTRL);
		*val = readl(pcie->reg + H2X_CFGI_RET_DATA);
	} else {
		bdf_offset = ((bus->number) << 24) | (PCI_SLOT(devfn) << 19) |
			     (PCI_FUNC(devfn) << 16) | (where & ~3);

		pcie->tx_tag %= 0xF;

		type = (bus->number == 1) ? PCI_HEADER_TYPE_NORMAL : PCI_HEADER_TYPE_BRIDGE;

		/* Prepare TLP */
		writel(CRG_READ_FMTTYPE(type) | CRG_PAYLOAD_SIZE, pcie->reg + H2X_CFGE_TLP_1ST);
		writel(0x40100F | (pcie->tx_tag << 8), pcie->reg + H2X_CFGE_TLP_NEXT);
		writel(bdf_offset, pcie->reg + H2X_CFGE_TLP_NEXT);
		/* Clear TX/RX status */
		writel(CFGE_TX_IDLE | CFGE_RX_IDLE, pcie->reg + H2X_CFGE_INT_STS);
		/* Issue command */
		writel(CFGE_TLP_FIRE, pcie->reg + H2X_CFGE_CTRL);

		pcie->tx_tag++;

		ret = readl_poll_timeout(pcie->reg + H2X_CFGE_INT_STS, status,
					 (status & CFGE_RX_IDLE), 0, 50000);
		writel(status, pcie->reg + H2X_CFGE_INT_STS);
		if (ret) {
			dev_err(pcie->dev,
				"RC [%04X:%02X:%02X.%02X] : RX Conf. timeout, sts: %x\n",
				pcie->domain, bus->number, PCI_SLOT(devfn),
				PCI_FUNC(devfn), status);
			*val = 0xffffffff;
			return PCIBIOS_SUCCESSFUL;
		}
		*val = readl(pcie->reg + H2X_CFGE_RET_DATA);
	}

	switch (size) {
	case 1:
		*val = (*val >> ((where & 3) * 8)) & 0xff;
		break;
	case 2:
		*val = (*val >> ((where & 2) * 8)) & 0xffff;
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int aspeed_ast2700_wr_conf(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 shift = 8 * (where & 3);
	u8 byte_en;
	u32 bdf_offset, status, type;
	int ret;

	if (bus->number == 0 && devfn != 0)
		return PCIBIOS_SUCCESSFUL;

	switch (size) {
	case 1:
		byte_en = 1 << (where % 4);
		val = (val & 0xff) << shift;
		break;
	case 2:
		byte_en = (((where >> 1) % 2) == 0) ? 0x3 : 0xc;
		val = (val & 0xffff) << shift;
		break;
	default:
		byte_en = 0xf;
		break;
	}

	if (bus->number == 0) {
		/* Internal access to bridge */
		writel(0x100000 | byte_en << 16 | (where & ~3), pcie->reg + H2X_CFGI_TLP);
		writel(val, pcie->reg + H2X_CFGI_WR_DATA);
		writel(CFGI_TLP_FIRE, pcie->reg + H2X_CFGI_CTRL);
	} else {
		bdf_offset = (bus->number << 24) | (PCI_SLOT(devfn) << 19) |
			     (PCI_FUNC(devfn) << 16) | (where & ~3);
		pcie->tx_tag %= 0xF;

		type = (bus->number == 1) ? PCI_HEADER_TYPE_NORMAL : PCI_HEADER_TYPE_BRIDGE;

		/* Prepare TLP */
		writel(CRG_WRITE_FMTTYPE(type) | CRG_PAYLOAD_SIZE, pcie->reg + H2X_CFGE_TLP_1ST);
		writel(0x401000 | (pcie->tx_tag << 8) | byte_en, pcie->reg + H2X_CFGE_TLP_NEXT);
		writel(bdf_offset, pcie->reg + H2X_CFGE_TLP_NEXT);
		writel(val, pcie->reg + H2X_CFGE_TLP_NEXT);
		/* Clear TX/RX idle status */
		writel(CFGE_TX_IDLE | CFGE_RX_IDLE, pcie->reg + H2X_CFGE_INT_STS);
		/* Issue command */
		writel(CFGE_TLP_FIRE, pcie->reg + H2X_CFGE_CTRL);

		pcie->tx_tag++;

		ret = readl_poll_timeout(pcie->reg + H2X_CFGE_INT_STS, status,
					 (status & CFGE_RX_IDLE), 0, 50000);
		writel(status, pcie->reg + H2X_CFGE_INT_STS);
		if (ret)
			dev_err(pcie->dev,
				"RC [%04X:%02X:%02X.%02X] : TX Conf. timeout, sts: %x\n",
				pcie->domain, bus->number, PCI_SLOT(devfn),
				PCI_FUNC(devfn), status);

		(void)readl(pcie->reg + H2X_CFGE_RET_DATA);
	}

	return PCIBIOS_SUCCESSFUL;
}

/* PCIe operations */
static struct pci_ops aspeed_ast2600_pcie_ops = {
	.read = aspeed_ast2600_rd_conf,
	.write = aspeed_ast2600_wr_conf,
};

static struct pci_ops aspeed_ast2700_pcie_ops = {
	.read = aspeed_ast2700_rd_conf,
	.write = aspeed_ast2700_wr_conf,
};

#ifdef CONFIG_PCI_MSI
static void aspeed_msi_compose_msi_msg(struct irq_data *data,
				       struct msi_msg *msg)
{
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(data);

	msg->address_hi = 0;
	msg->address_lo = pcie->msi_address;
	msg->data = data->hwirq;
}

static int aspeed_msi_set_affinity(struct irq_data *irq_data,
				   const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static struct irq_chip aspeed_msi_bottom_irq_chip = {
	.name = "ASPEED MSI",
	.irq_compose_msi_msg = aspeed_msi_compose_msi_msg,
	.irq_set_affinity = aspeed_msi_set_affinity,
};

static int aspeed_irq_msi_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *args)
{
	struct aspeed_pcie *pcie = domain->host_data;
	int bit;
	int i;

	mutex_lock(&pcie->lock);

	bit = bitmap_find_free_region(pcie->msi_irq_in_use, MAX_MSI_HOST_IRQS,
				      get_count_order(nr_irqs));

	mutex_unlock(&pcie->lock);

	if (bit < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i,
				    &aspeed_msi_bottom_irq_chip,
				    domain->host_data, handle_simple_irq, NULL,
				    NULL);
	}

	return 0;
}

static void aspeed_irq_msi_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(data);

	mutex_lock(&pcie->lock);

	bitmap_release_region(pcie->msi_irq_in_use, data->hwirq,
			      get_count_order(nr_irqs));

	mutex_unlock(&pcie->lock);
}

static void aspeed_pcie_msi_enable(struct aspeed_pcie *pcie)
{
	writel(0xffffffff, pcie->reg + pcie->platform->reg_msi_en);
	writel(0xffffffff, pcie->reg + pcie->platform->reg_msi_en + 0x04);
}

static const struct irq_domain_ops aspeed_msi_domain_ops = {
	.alloc = aspeed_irq_msi_domain_alloc,
	.free = aspeed_irq_msi_domain_free,
};

static struct irq_chip aspeed_msi_irq_chip = {
	.name = "PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static struct msi_domain_info aspeed_msi_domain_info = {
	.flags = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_MULTI_PCI_MSI),
	.chip = &aspeed_msi_irq_chip,
};
#endif

static int aspeed_pcie_init_irq_domain(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pcie_intc_node;
#ifdef CONFIG_PCI_MSI
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct irq_domain *parent;
#endif

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return -ENODEV;
	}

	pcie->irq_domain =
		irq_domain_add_linear(pcie_intc_node, PCI_NUM_INTX, &aspeed_intx_domain_ops, pcie);

	if (!pcie->irq_domain) {
		dev_err(dev, "failed to get an INTx IRQ domain\n");
		return -ENOMEM;
	}

	of_node_put(pcie_intc_node);

	if (!pcie->support_msi)
		return 0;

#ifdef CONFIG_PCI_MSI
	pcie->dev_domain =
		irq_domain_add_linear(NULL, MAX_MSI_HOST_IRQS, &aspeed_msi_domain_ops, pcie);
	if (!pcie->dev_domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	pcie->msi_domain =
		pci_msi_create_irq_domain(fwnode, &aspeed_msi_domain_info, pcie->dev_domain);
	if (!pcie->msi_domain) {
		dev_err(pcie->dev, "failed to create MSI domain\n");
		irq_domain_remove(parent);
		return -ENOMEM;
	}
	aspeed_pcie_msi_enable(pcie);
#endif

	return 0;
}

static void aspeed_pcie_port_init(struct aspeed_pcie *pcie)
{
	u32 link_sts = 0;

	//plda init
	regmap_write(pcie->pciephy, ASPEED_PCIE_LOCK, PCIE_UNLOCK);
#ifdef CONFIG_HOTPLUG_PCI
	regmap_write(pcie->pciephy, ASPEED_PCIE_GLOBAL,
		     RC_SYNC_RESET_DISABLE | ROOT_COMPLEX_ID(0x3) |
			     PCIE_RC_SLOT_ENABLE);
	regmap_write(pcie->pciephy, 0x10, 0xd7040022 | DATALINK_REPORT_CAPABLE);
	regmap_write(pcie->pciephy, 0x14,
		     HOTPLUG_CAPABLE_ENABLE | HOTPLUG_SURPRISE_ENABLE |
			     ATTENTION_BUTTON_ENALBE);
#else
	regmap_write(pcie->pciephy, ASPEED_PCIE_GLOBAL, ROOT_COMPLEX_ID(0x3));
#endif
	/* Toggle the gpio to reset the devices on RC bus */
	if (pcie->perst_rc_out) {
		mdelay(100);
		gpiod_set_value(pcie->perst_rc_out, 1);
	}

	reset_control_deassert(pcie->perst);
	mdelay(500);

	//clr intx isr
	writel(0x0, pcie->reg + 0x04);

	//clr msi isr
	writel(0xFFFFFFFF, pcie->reg + 0x28);
	writel(0xFFFFFFFF, pcie->reg + 0x2c);

	//rc_l
	//	0x80: 040 set bit7 0
	//	0xC0: 080 set bit7 1
	if (pcie->domain)
		writel(PCIE_RX_DMA_EN | PCIE_RX_LINEAR | PCIE_RX_MSI_SEL |
			       PCIE_RX_MSI_EN | PCIE_Wait_RX_TLP_CLR |
			       PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
		       pcie->reg);
	else
		writel(PCIE_RX_DMA_EN | PCIE_RX_LINEAR | PCIE_RX_MSI_EN |
			       PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE |
			       PCIE_RC_ENABLE,
		       pcie->reg);

	//assign debug tx tag
	writel(0x28, pcie->reg + 0x3C);

	regmap_read(pcie->pciephy, ASPEED_PCIE_LINK, &link_sts);
	if (link_sts & PCIE_LINK_STS) {
		//		aspeed_pcie_set_slot_power_limit(pcie);
		dev_info(pcie->dev, "PCIE- Link up\n");
		//		if (readl(pcie->pciereg_base
		//				+ ASPEED_PCIE_LINK_STS) & PCIE_LINK_2_5G)
		//			dev_info(pcie->dev, "PCIE- Link up : 2.5G\n");
	} else {
		dev_info(pcie->dev, "PCIE- Link down\n");
	}
}


static ssize_t hotplug_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct aspeed_pcie *pcie = dev_get_drvdata(dev);

	pcie->hotplug_event = 1;

	return len;
}

static DEVICE_ATTR_WO(hotplug);

static void aspeed_pcie_reset_work(struct work_struct *work)
{
	struct aspeed_pcie *pcie =
		container_of(work, typeof(*pcie), rst_dwork.work);
	struct pci_host_bridge *host = pci_host_bridge_from_priv(pcie);
	struct pci_bus *parent = host->bus;
	struct pci_dev *dev, *temp;
	u32 link_sts = 0;
	u16 command;

	pci_lock_rescan_remove();

	list_for_each_entry_safe_reverse(dev, temp, &parent->devices,
					 bus_list) {
		pci_dev_get(dev);
		pci_stop_and_remove_bus_device(dev);
		/*
		 * Ensure that no new Requests will be generated from
		 * the device.
		 */
		pci_read_config_word(dev, PCI_COMMAND, &command);
		command &= ~(PCI_COMMAND_MASTER | PCI_COMMAND_SERR);
		command |= PCI_COMMAND_INTX_DISABLE;
		pci_write_config_word(dev, PCI_COMMAND, command);
		pci_dev_put(dev);
	}

	/*
	 * With perst_rc_out GPIO, the perst will only affect our PCIe controller, so it only
	 * needs to stay low for 1ms.
	 * Without perst_rc_out GPIO, the perst will affect external devices, so it needs to
	 * follow the spec and stay low for at least 100ms.
	 */
	reset_control_assert(pcie->perst);
	if (pcie->perst_rc_out) {
		gpiod_set_value(pcie->perst_rc_out, 0);
		mdelay(1);
	} else {
		mdelay(100);
	}
	reset_control_deassert(pcie->perst);
	if (pcie->perst_rc_out) {
		mdelay(100);
		gpiod_set_value(pcie->perst_rc_out, 1);
	}
	mdelay(10);

	regmap_read(pcie->pciephy, ASPEED_PCIE_LINK, &link_sts);
	if (link_sts & PCIE_LINK_STS)
		dev_info(pcie->dev, "PCIE- Link up\n");
	else
		dev_info(pcie->dev, "PCIE- Link down\n");

	pci_rescan_bus(host->bus);
	pci_unlock_rescan_remove();
}

static irqreturn_t pcie_rst_irq_handler(int irq, void *dev_id)
{
	struct aspeed_pcie *pcie = dev_id;

	schedule_delayed_work(&pcie->rst_dwork, 0);

	return IRQ_HANDLED;
}

#define AHBC_UNLOCK	0xAEED1A03
static int aspeed_ast2600_setup(struct platform_device *pdev)
{
	struct aspeed_pcie *pcie = platform_get_drvdata(pdev);
	struct device_node *cfg_node;
	int err;

	pcie->perst_rc_out =
		devm_gpiod_get_optional(pcie->dev, "perst-rc-out",
					GPIOD_OUT_LOW |
					GPIOD_FLAGS_BIT_NONEXCLUSIVE);

	pcie->perst = devm_reset_control_get_exclusive(pcie->dev, NULL);
	if (IS_ERR(pcie->perst)) {
		dev_err(&pdev->dev, "can't get pcie phy reset\n");
		return PTR_ERR(pcie->perst);
	}
	reset_control_assert(pcie->perst);

	pcie->ahbc = syscon_regmap_lookup_by_compatible("aspeed,aspeed-ahbc");
	if (IS_ERR(pcie->ahbc))
		return IS_ERR(pcie->ahbc);

	cfg_node =
		of_find_compatible_node(NULL, NULL, "aspeed,ast2600-pciecfg");
	if (cfg_node) {
		pcie->cfg = syscon_node_to_regmap(cfg_node);
		if (IS_ERR(pcie->cfg))
			return PTR_ERR(pcie->cfg);
	}

	//workaround : Send vender define message for avoid when PCIE RESET send unknown message out
	regmap_write(pcie->cfg, 0x10, 0x34000000);
	regmap_write(pcie->cfg, 0x14, 0x0000007f);
	regmap_write(pcie->cfg, 0x18, 0x00001a03);
	regmap_write(pcie->cfg, 0x1c, 0x00000000);

	regmap_write(pcie->ahbc, 0x00, AHBC_UNLOCK);
	regmap_update_bits(pcie->ahbc, 0x8C, BIT(5), BIT(5));
	regmap_write(pcie->ahbc, 0x00, 0x1);

	//ahb to pcie rc
	regmap_write(pcie->cfg, 0x60, 0xe0006000);
	regmap_write(pcie->cfg, 0x64, 0x00000000);
	regmap_write(pcie->cfg, 0x68, 0xFFFFFFFF);

	//PCIe Host Enable
	regmap_write(pcie->cfg, 0x00, BIT(0));

	//080 can't config for msi
	pcie->support_msi = (pcie->domain) ? false : true;

	aspeed_pcie_port_init(pcie);

	pcie->host->ops = &aspeed_ast2600_pcie_ops;

	err = sysfs_create_file(&pdev->dev.kobj, &dev_attr_hotplug.attr);
	if (err) {
		dev_err(&pdev->dev, "unable to create sysfs interface\n");
		return err;
	}

	if (pcie->domain) {
		pcie->perst_ep_in =
			devm_gpiod_get_optional(pcie->dev, "perst-ep-in", GPIOD_IN);
		if (pcie->perst_ep_in) {
			gpiod_set_debounce(pcie->perst_ep_in, 100);
			irq_set_irq_type(gpiod_to_irq(pcie->perst_ep_in),
					 IRQ_TYPE_EDGE_BOTH);
			err = devm_request_irq(pcie->dev,
					       gpiod_to_irq(pcie->perst_ep_in),
					       pcie_rst_irq_handler,
					       IRQF_SHARED, "PERST monitor",
					       pcie);
			if (err) {
				dev_err(pcie->dev,
					"Failed to request gpio irq %d\n", err);
				return err;
			}
			INIT_DELAYED_WORK(&pcie->rst_dwork,
					  aspeed_pcie_reset_work);
		}
		pcie->perst_owner =
			devm_gpiod_get_optional(pcie->dev, "perst-owner", GPIOD_OUT_HIGH);
	}

	return 0;
}

static int aspeed_ast2700_setup(struct platform_device *pdev)
{
	struct aspeed_pcie *pcie = platform_get_drvdata(pdev);
	struct device *dev = pcie->dev;
	u32 cfg_val;

	pcie->h2xrst = devm_reset_control_get(dev, "h2x");
	if (IS_ERR(pcie->h2xrst))
		return dev_err_probe(dev, PTR_ERR(pcie->h2xrst), "failed to get h2x reset\n");

	pcie->perst = devm_reset_control_get(dev, "perst");
	if (IS_ERR(pcie->perst))
		return dev_err_probe(dev, PTR_ERR(pcie->perst), "failed to get perst reset\n");

	pcie->device = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,device");
	if (IS_ERR(pcie->device))
		return dev_err_probe(dev, PTR_ERR(pcie->device), "failed to map device base\n");

	reset_control_assert(pcie->perst);

	regmap_write(pcie->pciephy, 0x00, 0x11501a02);
	regmap_write(pcie->pciephy, 0x70, 0xa00c0);
	regmap_write(pcie->pciephy, 0x78, 0x80030);
	regmap_write(pcie->pciephy, 0x58, 0x1);

	regmap_write(pcie->device, 0x60, 0xf0001);
	regmap_write(pcie->device, 0x64, 0xff00ff00);
	regmap_write(pcie->device, 0x70, 0);
	regmap_write(pcie->device, 0x78, (pcie->domain == 1) ? BIT(31) : 0);

	reset_control_assert(pcie->h2xrst);
	mdelay(10);
	reset_control_deassert(pcie->h2xrst);

	regmap_write(pcie->pciephy, 0x5C, 0x40000000);
	/* Configure to Root port */
	regmap_read(pcie->pciephy, PEHR_GEN_CAPABILITY, &cfg_val);
	regmap_write(pcie->pciephy, PEHR_GEN_CAPABILITY,
		     (cfg_val & ~PORT_TPYE) | FIELD_PREP(PORT_TPYE, PORT_TYPE_ROOT));

	/* PCIe Host Enable */
	writel(0, pcie->reg + H2X_CTRL);
	writel(H2X_BRIDGE_EN | H2X_BRIDGE_DIRECT_EN, pcie->reg + H2X_CTRL);

	/* The BAR mapping:
	 * CPU Node0: 0x60000000
	 * CPU Node1: 0x80000000
	 * IO       : 0xa0000000
	 */
	writel(0x60000000 + (0x20000000 * pcie->domain), pcie->reg + H2X_REMAP_DIRECT_ADDR);

	reset_control_deassert(pcie->perst);
	mdelay(1000);

	/* Clear INTx isr */
	writel(0, pcie->reg + pcie->platform->reg_intx_sts);

	/* Clear MSI/MSI-X isr */
	writel(~0, pcie->reg + pcie->platform->reg_msi_sts);
	writel(~0, pcie->reg + pcie->platform->reg_msi_sts + 0x04);

	pcie->host->ops = &aspeed_ast2700_pcie_ops;

	aspeed_msi_domain_info.flags |= MSI_FLAG_PCI_MSIX;
	pcie->support_msi = true;

	return 0;
}

static int aspeed_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *host;
	struct aspeed_pcie *pcie;
	struct device_node *node = dev->of_node;
	const void *md = of_device_get_match_data(dev);
	int err;

	if (!md)
		return -ENODEV;

	host = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!host)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(host);
	pcie->dev = dev;
	pcie->tx_tag = 0;
	platform_set_drvdata(pdev, pcie);

	pcie->platform = md;
	pcie->host = host;

	pcie->reg = devm_platform_ioremap_resource(pdev, 0);

	of_property_read_u32(node, "msi_address", &pcie->msi_address);
	of_property_read_u32(node, "linux,pci-domain", &pcie->domain);

	pcie->pciephy = syscon_regmap_lookup_by_phandle(node, "pciephy");
	if (IS_ERR(pcie->pciephy))
		return dev_err_probe(dev, PTR_ERR(pcie->pciephy), "failed to map pciephy base\n");

	err = pcie->platform->setup(pdev);
	if (err) {
		dev_err(dev, "Setup PCIe RC failed\n");
		return err;
	}

	host->sysdata = pcie;

	pcie->irq = irq_of_parse_and_map(node, 0);
	if (pcie->irq < 0) {
		dev_err(dev, "Mapping IRQ failed\n");
		return pcie->irq;
	}

	err = aspeed_pcie_init_irq_domain(pcie);
	if (err) {
		dev_err(dev, "failed to init PCIe IRQ domain\n");
		return err;
	}

	irq_set_chained_handler_and_data(pcie->irq, aspeed_pcie_intr_handler,
					 pcie);

	return pci_host_probe(host);
}

static struct aspeed_pcie_rc_platform pcie_rc_ast2600 = {
	.setup = aspeed_ast2600_setup,
	.reg_intx_en = 0x04,
	.reg_intx_sts = 0x08,
	.reg_msi_en = 0x20,
	.reg_msi_sts = 0x28,
};

static struct aspeed_pcie_rc_platform pcie_rc_ast2700 = {
	.setup = aspeed_ast2700_setup,
	.reg_intx_en = 0x40,
	.reg_intx_sts = 0x48,
	.reg_msi_en = 0x50,
	.reg_msi_sts = 0x58,
};

static const struct of_device_id aspeed_pcie_of_match[] = {
	{ .compatible = "aspeed,ast2600-pcie", .data = &pcie_rc_ast2600 },
	{ .compatible = "aspeed,ast2700-pcie", .data = &pcie_rc_ast2700 },
	{}
};

static struct platform_driver aspeed_pcie_driver = {
	.driver = {
		.name = "aspeed-pcie",
		.suppress_bind_attrs = true,
		.of_match_table = aspeed_pcie_of_match,
	},
	.probe = aspeed_pcie_probe,
};

module_platform_driver(aspeed_pcie_driver);
