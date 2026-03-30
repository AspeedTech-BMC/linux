/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Aspeed eSPI protocol definitions and IOCTL methods
 * Copyright 2023 Aspeed Technology Inc.
 */
#ifndef __ASPEED_ESPI_COMM_H__
#define __ASPEED_ESPI_COMM_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * eSPI cycle type encoding
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
#define ESPI_PERIF_MEMRD32		0x00
#define ESPI_PERIF_MEMRD64		0x02
#define ESPI_PERIF_MEMWR32		0x01
#define ESPI_PERIF_MEMWR64		0x03
#define ESPI_PERIF_MSG			0x10
#define ESPI_PERIF_MSG_D		0x11
#define ESPI_PERIF_SUC_CMPLT		0x06
#define ESPI_PERIF_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_PERIF_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_PERIF_SUC_CMPLT_D_LAST	0x0d
#define ESPI_PERIF_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_PERIF_UNSUC_CMPLT		0x0c
#define ESPI_OOB_MSG			0x21
#define ESPI_FLASH_READ			0x00
#define ESPI_FLASH_WRITE		0x01
#define ESPI_FLASH_ERASE		0x02
#define ESPI_FLASH_SUC_CMPLT		0x06
#define ESPI_FLASH_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_FLASH_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_FLASH_SUC_CMPLT_D_LAST	0x0d
#define ESPI_FLASH_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_FLASH_UNSUC_CMPLT		0x0c

/*
 * eSPI packet format structure
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
struct espi_comm_hdr {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
};

struct espi_perif_mem32 {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u32 addr_be;
	u8 data[];
} __packed;

struct espi_perif_mem64 {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u32 addr_be;
	u8 data[];
} __packed;

struct espi_perif_msg {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u8 msg_code;
	u8 msg_byte[4];
	u8 data[];
} __packed;

struct espi_perif_cmplt {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u8 data[];
} __packed;

struct espi_oob_msg {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u8 data[];
};

struct espi_flash_rwe {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u32 addr_be;
	u8 data[];
} __packed;

struct espi_flash_cmplt {
	u8 cyc;
	u8 len_h : 4;
	u8 tag : 4;
	u8 len_l;
	u8 data[];
} __packed;

#define ESPI_MAX_PLD_LEN	BIT(12)

/*
 * Aspeed IOCTL for eSPI raw packet send/receive
 *
 * This IOCTL interface works in the eSPI packet in/out paradigm.
 *
 * Only the virtual wire IOCTL is a special case which does not send
 * or receive an eSPI packet. However, to keep a more consisten use from
 * userspace, we make all of the four channel drivers serve through the
 * IOCTL interface.
 *
 * For the eSPI packet format, refer to
 *   Section 5.1 Cycle Types and Packet Format,
 *   Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 *
 * For the example user apps using these IOCTL, refer to
 *   https://github.com/AspeedTech-BMC/aspeed_app/tree/master/espi_test
 */
#define __ASPEED_ESPI_IOCTL_MAGIC	0xb8

/*
 * we choose the longest header and the max payload size
 * based on the Intel specification to define the maximum
 * eSPI packet length
 */
#define ESPI_MAX_PKT_LEN	(sizeof(struct espi_perif_msg) + ESPI_MAX_PLD_LEN)

struct aspeed_espi_ioc {
	u32 pkt_len;
	u8 *pkt;
};

/*
 * Peripheral Channel (CH0)
 *  - ASPEED_ESPI_PERIF_PC_GET_RX
 *      Receive an eSPI Posted/Completion packet
 *  - ASPEED_ESPI_PERIF_PC_PUT_TX
 *      Transmit an eSPI Posted/Completion packet
 *  - ASPEED_ESPI_PERIF_NP_PUT_TX
 *      Transmit an eSPI Non-Posted packet
 */
#define ASPEED_ESPI_PERIF_PC_GET_RX	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x00, struct aspeed_espi_ioc)
#define ASPEED_ESPI_PERIF_PC_PUT_TX	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x01, struct aspeed_espi_ioc)
#define ASPEED_ESPI_PERIF_NP_PUT_TX	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x02, struct aspeed_espi_ioc)
/*
 * Virtual Wire Channel (CH1)
 *  - ASPEED_ESPI_VW_GET_GPIO_VAL
 *      Read the input value of GPIO over the VW channel
 *  - ASPEED_ESPI_VW_PUT_GPIO_VAL
 *      Write the output value of GPIO over the VW channel
 *  - ASPEED_ESPI_VW_GET_GPIO_VAL1 (new feature in AST2700)
 *      Read the input value1 of GPIO over the VW channel
 *  - ASPEED_ESPI_VW_PUT_GPIO_VAL1 (new feature in AST2700)
 *      Write the output value1 of GPIO over the VW channel
 *  - ASPEED_ESPI_VW_GET_PCH_GENERIC
 *      Read the input value of PCH Generic Indexes over the VW channel
 *  - ASPEED_ESPI_VW_PUT_PCH_GENERIC
 *      Write the output value of PCH Generic Indexes over the VW channel
 */
#define ASPEED_ESPI_VW_GET_GPIO_VAL	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x10, u32)
#define ASPEED_ESPI_VW_PUT_GPIO_VAL	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x11, u32)
#ifdef CONFIG_ARM64
#define ASPEED_ESPI_VW_GET_GPIO_VAL1	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x12, u32)
#define ASPEED_ESPI_VW_PUT_GPIO_VAL1	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x13, u32)
#endif
#define ASPEED_ESPI_VW_GET_PCH_GENERIC	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x14, u32)
#define ASPEED_ESPI_VW_PUT_PCH_GENERIC	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x15, u32)
/*
 * Out-of-band Channel (CH2)
 *  - ASPEED_ESPI_OOB_GET_RX
 *      Receive an eSPI OOB packet
 *  - ASPEED_ESPI_OOB_PUT_TX
 *      Transmit an eSPI OOB packet
 */
#define ASPEED_ESPI_OOB_GET_RX		_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x20, struct aspeed_espi_ioc)
#define ASPEED_ESPI_OOB_PUT_TX		_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x21, struct aspeed_espi_ioc)
/*
 * Flash Channel (CH3)
 *  - ASPEED_ESPI_FLASH_GET_RX
 *      Receive an eSPI flash packet
 *  - ASPEED_ESPI_FLASH_PUT_TX
 *      Transmit an eSPI flash packet
 */
#define ASPEED_ESPI_FLASH_GET_RX	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x30, struct aspeed_espi_ioc)
#define ASPEED_ESPI_FLASH_PUT_TX	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x31, struct aspeed_espi_ioc)

#endif
