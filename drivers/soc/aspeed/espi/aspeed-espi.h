/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Unified eSPI driver header file and data structures
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_ESPI_H__
#define __ASPEED_ESPI_H__

#include "linux/platform_device.h"
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/irqreturn.h>

#define PERIF_MMBI_INST_NUM	8
#define DEVICE_NAME		"aspeed-espi"

enum aspeed_espi_platform_id {
	ASPEED_ESPI_PLATFORM_ID_AST2500 = 0x2500,
	ASPEED_ESPI_PLATFORM_ID_AST2600 = 0x2600,
	ASPEED_ESPI_PLATFORM_ID_AST2700 = 0x2700,
};

/* consistent with DTS property "flash-safs-mode" */
enum aspeed_edaf_mode {
	EDAF_MODE_MIX = 0x0,
	EDAF_MODE_SW,
	EDAF_MODE_HW,
	EDAF_MODES,
};

struct aspeed_espi_perif;

struct aspeed_espi_perif_mmbi {
	void *b2h_virt;
	void *h2b_virt;
	dma_addr_t b2h_addr;
	dma_addr_t h2b_addr;
	struct miscdevice b2h_mdev;
	struct miscdevice h2b_mdev;
	bool host_rwp_update;
	wait_queue_head_t wq;
	struct aspeed_espi_perif *perif;
};

struct aspeed_espi_perif {
	struct {
		bool enable;
		int irq;
		void *virt;
		dma_addr_t taddr;
		dma_addr_t saddr;
		resource_size_t size;
		u32 inst_num;
		u32 inst_size;
		irqreturn_t (*mmbi_isr)(int irq, void *espi);
		struct aspeed_espi_perif_mmbi inst[PERIF_MMBI_INST_NUM];
	} mmbi;

	struct {
		bool enable;
		void *virt;
		dma_addr_t taddr;
		dma_addr_t saddr;
		resource_size_t size;
	} mcyc;

	struct {
		bool enable;
		void *np_tx_virt;
		dma_addr_t np_tx_addr;
		void *pc_tx_virt;
		dma_addr_t pc_tx_addr;
		void *pc_rx_virt;
		dma_addr_t pc_rx_addr;
	} dma;

	bool rtc_enable;
	bool rx_ready;
	wait_queue_head_t wq;

	spinlock_t lock; // protects rx_ready
	struct mutex np_tx_mtx; // protects np_tx_virt/addr
	struct mutex pc_tx_mtx; // protects pc_tx_virt/addr
	struct mutex pc_rx_mtx; // protects pc_rx_virt/addr

	struct miscdevice mdev;
};

struct aspeed_espi_vw {
	struct {
		bool hw_mode;
		u32 grp;
		u32 dir0;
		u32 dir1;
		u32 val0;
		u32 val1;
	} gpio;

	struct {
		u8 pch_generic;
	} platform;

	struct {
		bool enabled;
		spinlock_t pltrst_lock; // protects pltrst_status
		wait_queue_head_t pltrst_wq;
		char pltrst_status;
		bool pltrst_avail;
	} pltrst;

	struct miscdevice pltrst_mdev;
	struct miscdevice mdev;
};

struct ast2700_espi_oob_dma_tx_desc {
	u32 data_addrl;
	u32 data_addrh;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 msg_type : 3;
	u8 raz0 : 1;
	u8 pec : 1;
	u8 int_en : 1;
	u8 pause : 1;
	u8 raz1 : 1;
	u32 raz2;
	u32 raz3;
	u32 pad[3];
} __packed;

struct ast2700_espi_oob_dma_rx_desc {
	u32 data_addrl;
	u32 data_addrh;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 raz : 7;
	u8 dirty : 1;
	u32 pad;
} __packed;

struct ast2600_espi_oob_dma_tx_desc {
	u32 data_addr;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 msg_type : 3;
	u8 raz0 : 1;
	u8 pec : 1;
	u8 int_en : 1;
	u8 pause : 1;
	u8 raz1 : 1;
	u32 raz2;
	u32 raz3;
} __packed;

struct ast2600_espi_oob_dma_rx_desc {
	u32 data_addr;
	u8 cyc;
	u16 tag : 4;
	u16 len : 12;
	u8 raz : 7;
	u8 dirty : 1;
} __packed;

struct aspeed_espi_oob {
	struct {
		bool enable;
		void *txd_virt;
		dma_addr_t txd_addr;
		void *rxd_virt;
		dma_addr_t rxd_addr;
		void *tx_virt;
		dma_addr_t tx_addr;
		void *rx_virt;
		dma_addr_t rx_addr;
	} dma;

	bool rx_ready;
	wait_queue_head_t wq;

	spinlock_t lock; // protects rx_ready
	struct mutex tx_mtx; // protects tx_virt/addr
	struct mutex rx_mtx; // protects rx_virt/addr

	struct miscdevice mdev;
};

struct aspeed_espi_flash {
	struct {
		u32 mode;
		phys_addr_t taddr;
		resource_size_t size;
	} edaf;

	struct {
		bool enable;
		void *tx_virt;
		dma_addr_t tx_addr;
		void *rx_virt;
		dma_addr_t rx_addr;
	} dma;

	bool rx_ready;
	wait_queue_head_t wq;

	spinlock_t lock; // protects rx_ready
	struct mutex rx_mtx; // protects rx_virt/addr
	struct mutex tx_mtx; // protects tx_virt/addr

	struct miscdevice mdev;
};

struct aspeed_espi {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *regs;
	struct reset_control *rst;
	struct clk *clk;
	int dev_id;
	int irq;

	struct aspeed_espi_perif perif;
	struct aspeed_espi_vw vw;
	struct aspeed_espi_oob oob;
	struct aspeed_espi_flash flash;
	const struct aspeed_espi_ops *ops;
};

#endif // __ASPEED_ESPI_H__
