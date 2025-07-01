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

struct mctp_pcie_vdm_dev;

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

struct mctp_pcie_vdm_dev *mctp_pcie_vdm_add_dev(struct device *dev,
						const struct mctp_pcie_vdm_ops *ops);
void mctp_pcie_vdm_remove_dev(struct mctp_pcie_vdm_dev *vdm_dev);

/**
 * Notify mctp-pcie-vdm that packets are received and ready to be processed.
 * After notified, mctp-pcie-vdm will call recv_packet() multiple times
 * until no more packets to be processed. The lower layer driver can keep receiving
 * packets while the upper layer is processing the received packets.
 */
void mctp_pcie_vdm_notify_rx(struct mctp_pcie_vdm_dev *vdm_dev);

#endif	 /* __LINUX_MCTP_PCIE_VDM_H */
