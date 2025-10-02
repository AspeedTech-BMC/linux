/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 Aspeed Technology Inc.
 */
#ifndef __ASPEED_PCIE_MMBI_H__
#define __ASPEED_PCIE_MMBI_H__

#define MMBI_SIGNATURE "#MMBI$"

//This definitions are as per MMBI specification.
#define MMBI_PROTOCOL_IPMI 1
#define MMBI_PROTOCOL_SEAMLESS 2
#define MMBI_PROTOCOL_RAS_OFFLOAD 3
#define MMBI_PROTOCOL_MCTP 4
#define MMBI_PROTOCOL_NODE_MANAGER 5

#define MMBI_HRWS0(x)	readl((x)->hrws_vmem)
#define MMBI_HRWS1(x)	readl((x)->hrws_vmem + 4)
#define MMBI_HROS0(x)	readl((x)->hros_vmem)
#define MMBI_HROS1(x)	readl((x)->hros_vmem + 4)

#define H2B_WRITE_POINTER_MASK GENMASK(31, 2)
#define H2B_READ_POINTER_MASK GENMASK(31, 2)
#define B2H_WRITE_POINTER_MASK GENMASK(31, 2)
#define B2H_READ_POINTER_MASK GENMASK(31, 2)

#define GET_H2B_WRITE_POINTER(x) (MMBI_HRWS0(x) & H2B_WRITE_POINTER_MASK)
#define GET_H2B_READ_POINTER(x) (MMBI_HROS1(x) & H2B_READ_POINTER_MASK)
#define GET_B2H_WRITE_POINTER(x) (MMBI_HROS0(x) & B2H_WRITE_POINTER_MASK)
#define GET_B2H_READ_POINTER(x) (MMBI_HRWS1(x) & B2H_READ_POINTER_MASK)

#define GET_HOST_READY_BIT(x) (MMBI_HRWS1(x) & 0x01)
#define GET_BMC_READY_BIT(x) (MMBI_HROS1(x) & 0x01)

typedef u8 protocol_type;

enum mmbi_state {				/* B_U B_R H_U H_R */
	INIT_IN_PROGRESS	= 0x00,		/* 0   0   0   0   */
	INIT_COMPLETED		= 0x08,		/* 1   0   0   0   */
	NORMAL_RUNTIME		= 0x0A,		/* 1   0   1   0   */
	RESET_REQ_BY_BMC	= 0x0E,		/* 1   1   1   0   */
	RESET_REQ_BY_HOST	= 0x0B,		/* 1   0   1   1   */
	RESET_ACKED		= 0x0F,		/* 1   1   1   1   */
	TRANS_TO_INIT		= 0x07,		/* 0   1   1   1   */
	INIT_MISMATCH		= 0x09,		/* 1   0   0   1   */
	POWER_UP_OR_ERROR	= 0x80000000,
};

struct mmbi_header {
	u32 pkt_pad	: 2;
	u32 pkt_len	: 22;
	u32 pkt_type	: 4;
	u32 reserved	: 4;
};

struct host_ros {
	u32 b_rst	: 1;		/* BMC Reset Request */
	u32 b_up	: 1;		/* BMC Interface Up */
	u32 b2h_wp	: 30;		/* B2H Write Pointer */
	u32 b_rdy	: 1;		/* BMC Ready */
	u32 reserved1	: 1;
	u32 h2b_rp	: 30;		/* H2B Read Pointer */
};

struct host_rws {
	u32 h_rst	: 1;		/* Host Reset Request */
	u32 h_up	: 1;		/* Host Interface Up */
	u32 h2b_wp	: 30;		/* H2B Write Pointer */
	u32 h_rdy	: 1;		/* Host Ready */
	u32 reserved1	: 1;
	u32 b2h_rp	: 30;		/* B2H Read Pointer */
};

struct buffer_type_desc {
	u32 h_ros_p;			/* Host Read-Only Structure Pointer */
	u32 h_rws_p;			/* Host Read-Write Structure Pointer */
	u8 h_int_t;			/* Host Interrupt Type */
	u8 h_int_l;			/* Host Interrupt Location */
	u8 reserved1[3];
	u8 h_int_v;			/* Host Interrupt Value */
	u8 bmc_int_t;			/* BMC Interrupt Type */
	u32 bmc_int_l;			/* BMC Interrupt Location */
	u8 reserved2[4];
	u8 bmc_int_v;			/* BMC Interrupt Value */
} __packed;

struct mmbi_cap_desc {
	u8 signature[6];
	u8 version;
	u8 os_use;
	u32 b2h_ba;			/* B2H Buffer Base Address */
	u32 h2b_ba;			/* H2B Buffer Base Address */
	u32 b2h_l;			/* B2H Buffer Length */
	u32 h2b_l;			/* H2B Buffer Length */
	u8 buffer_type;
	u8 reserved1[7];
	struct buffer_type_desc bt_desc;
	u8 reserved2[8];
} __packed;

struct aspeed_pcie_mmbi;

#define MCTP_MMBI_MTU		65536
#define MCTP_MMBI_MTU_MIN	68	/* base mtu (64) + mctp header */
#define MCTP_MMBI_MTU_MAX	65536

struct aspeed_mmbi_mctp {
	struct aspeed_pcie_mmbi *mmbi;
	struct net_device *ndev;
};

struct aspeed_mmbi_channel {
	struct aspeed_pcie_mmbi *mmbi;
	struct device *dev;

	u32 b2h_cb_size;
	u32 h2b_cb_size;
	u8 __iomem *desc_vmem;
	u8 __iomem *hros_vmem;
	u8 __iomem *b2h_cb_vmem;
	u8 __iomem *hrws_vmem;
	u8 __iomem *h2b_cb_vmem;

	bool bmc_int_en;
	u8 bmc_int_value;
	u32 bmc_int_location;
	u8 __iomem *bmc_int_vmem;

	bool host_int_en;
	u8 host_int_location;
	u8 host_int_value;

	enum mmbi_state state;

	/* MCTP */
	struct net_device *ndev;

	struct work_struct work;
};

#endif
