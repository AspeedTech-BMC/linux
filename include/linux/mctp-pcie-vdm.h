/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mctp-pcie-vdm.h - MCTP-over-PCIe-VDM (DMTF DSP0238) transport binding Interface
 * for PCIe VDM devices to register and implement.
 *
 */

#ifndef __LINUX_MCTP_PCIE_VDM_H
#define __LINUX_MCTP_PCIE_VDM_H

#include <linux/device.h>
#include <linux/notifier.h>

/**
 * @send_packet: referenced to send packets with PCIe VDM header packed.
 * @recv_packet: referenced multiple times until no RX packet to be handled.
 *               received pointer shall start from the PCIe VDM header.
 * @free_packet: referenced when the packet is processed and okay to be freed.
 * @uninit: uninitialize the device.
 */
struct mctp_pcie_vdm_ops {
	int (*send_packet)(struct device *dev, u8 *data, size_t len);
	u8 *(*recv_packet)(struct device *dev);
	void (*free_packet)(void *packet);
	void (*uninit)(struct device *dev);
};

struct net_device *mctp_pcie_vdm_add_dev(struct device *dev,
					 const struct mctp_pcie_vdm_ops *ops);
void mctp_pcie_vdm_receive_packet(struct net_device *ndev);
void mctp_pcie_vdm_remove_dev(struct net_device *ndev);

#endif	 /* __LINUX_MCTP_PCIE_VDM_H */
