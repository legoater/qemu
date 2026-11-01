/*
 * vfio PCI device - VGA support
 *
 * Copyright Red Hat, Inc. 2012-2026
 *
 * Authors:
 *  Alex Williamson <alex.williamson@redhat.com>
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include <linux/vfio.h>

#include "hw/core/hw-error.h"
#include "hw/pci/pci_device.h"
#include "qemu/error-report.h"
#include "pci.h"
#include "trace.h"
#include "qapi/error.h"
#include "vfio-helpers.h"

void vfio_vga_write(void *opaque, hwaddr addr,
                           uint64_t data, unsigned size)
{
    VFIOVGARegion *region = opaque;
    VFIOVGA *vga = container_of(region, VFIOVGA, region[region->nr]);
    union {
        uint8_t byte;
        uint16_t word;
        uint32_t dword;
        uint64_t qword;
    } buf;
    off_t offset = vga->fd_offset + region->offset + addr;

    switch (size) {
    case 1:
        buf.byte = data;
        break;
    case 2:
        buf.word = cpu_to_le16(data);
        break;
    case 4:
        buf.dword = cpu_to_le32(data);
        break;
    default:
        hw_error("vfio: unsupported write size, %d bytes", size);
        break;
    }

    if (pwrite(vga->fd, &buf, size, offset) != size) {
        error_report("%s(,0x%"HWADDR_PRIx", 0x%"PRIx64", %d) failed: %m",
                     __func__, region->offset + addr, data, size);
    }

    trace_vfio_vga_write(region->offset + addr, data, size);
}

uint64_t vfio_vga_read(void *opaque, hwaddr addr, unsigned size)
{
    VFIOVGARegion *region = opaque;
    VFIOVGA *vga = container_of(region, VFIOVGA, region[region->nr]);
    union {
        uint8_t byte;
        uint16_t word;
        uint32_t dword;
        uint64_t qword;
    } buf;
    uint64_t data = 0;
    off_t offset = vga->fd_offset + region->offset + addr;

    if (pread(vga->fd, &buf, size, offset) != size) {
        error_report("%s(,0x%"HWADDR_PRIx", %d) failed: %m",
                     __func__, region->offset + addr, size);
        return (uint64_t)-1;
    }

    switch (size) {
    case 1:
        data = buf.byte;
        break;
    case 2:
        data = le16_to_cpu(buf.word);
        break;
    case 4:
        data = le32_to_cpu(buf.dword);
        break;
    default:
        hw_error("vfio: unsupported read size, %d bytes", size);
        break;
    }

    trace_vfio_vga_read(region->offset + addr, size, data);

    return data;
}

static const MemoryRegionOps vfio_vga_ops = {
    .read = vfio_vga_read,
    .write = vfio_vga_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

bool vfio_populate_vga(VFIOPCIDevice *vdev, Error **errp)
{
    VFIODevice *vbasedev = &vdev->vbasedev;
    struct vfio_region_info *reg_info = NULL;
    int ret;

    ret = vfio_device_get_region_info(vbasedev, VFIO_PCI_VGA_REGION_INDEX, &reg_info);
    if (ret) {
        error_setg_errno(errp, -ret,
                         "failed getting region info for VGA region index %d",
                         VFIO_PCI_VGA_REGION_INDEX);
        return false;
    }

    if (!(reg_info->flags & VFIO_REGION_INFO_FLAG_READ) ||
        !(reg_info->flags & VFIO_REGION_INFO_FLAG_WRITE) ||
        reg_info->size < 0xbffff + 1) {
        error_setg(errp, "unexpected VGA info, flags 0x%lx, size 0x%lx",
                   (unsigned long)reg_info->flags,
                   (unsigned long)reg_info->size);
        return false;
    }

    vdev->vga = g_new0(VFIOVGA, 1);

    vdev->vga->fd_offset = reg_info->offset;
    vdev->vga->fd = vdev->vbasedev.fd;

    vdev->vga->region[QEMU_PCI_VGA_MEM].offset = QEMU_PCI_VGA_MEM_BASE;
    vdev->vga->region[QEMU_PCI_VGA_MEM].nr = QEMU_PCI_VGA_MEM;
    QLIST_INIT(&vdev->vga->region[QEMU_PCI_VGA_MEM].quirks);

    memory_region_init_io(&vdev->vga->region[QEMU_PCI_VGA_MEM].mem,
                          OBJECT(vdev), &vfio_vga_ops,
                          &vdev->vga->region[QEMU_PCI_VGA_MEM],
                          "vfio-vga-mmio@0xa0000",
                          QEMU_PCI_VGA_MEM_SIZE);

    vdev->vga->region[QEMU_PCI_VGA_IO_LO].offset = QEMU_PCI_VGA_IO_LO_BASE;
    vdev->vga->region[QEMU_PCI_VGA_IO_LO].nr = QEMU_PCI_VGA_IO_LO;
    QLIST_INIT(&vdev->vga->region[QEMU_PCI_VGA_IO_LO].quirks);

    memory_region_init_io(&vdev->vga->region[QEMU_PCI_VGA_IO_LO].mem,
                          OBJECT(vdev), &vfio_vga_ops,
                          &vdev->vga->region[QEMU_PCI_VGA_IO_LO],
                          "vfio-vga-io@0x3b0",
                          QEMU_PCI_VGA_IO_LO_SIZE);

    vdev->vga->region[QEMU_PCI_VGA_IO_HI].offset = QEMU_PCI_VGA_IO_HI_BASE;
    vdev->vga->region[QEMU_PCI_VGA_IO_HI].nr = QEMU_PCI_VGA_IO_HI;
    QLIST_INIT(&vdev->vga->region[QEMU_PCI_VGA_IO_HI].quirks);

    memory_region_init_io(&vdev->vga->region[QEMU_PCI_VGA_IO_HI].mem,
                          OBJECT(vdev), &vfio_vga_ops,
                          &vdev->vga->region[QEMU_PCI_VGA_IO_HI],
                          "vfio-vga-io@0x3c0",
                          QEMU_PCI_VGA_IO_HI_SIZE);

    return true;
}

void vfio_pci_config_register_vga(VFIOPCIDevice *vdev)
{
    PCIDevice *pdev = PCI_DEVICE(vdev);
    assert(vdev->vga != NULL);

    pci_register_vga(pdev, &vdev->vga->region[QEMU_PCI_VGA_MEM].mem,
                     &vdev->vga->region[QEMU_PCI_VGA_IO_LO].mem,
                     &vdev->vga->region[QEMU_PCI_VGA_IO_HI].mem);
}
