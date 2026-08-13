ASPEED AST2700 UFS Host Controller
===================================

The AST2700 SoC includes a UFS host controller identified in the device tree
as ``aspeed,ufshc-m31-16nm``, mapped at ``0x12c08200`` (IRQ SPI 118).

QEMU models it as ``aspeed-ufs``, a subclass of the generic sysbus UFS
controller ``sysbus-ufs`` (``hw/ufs/ufs-sysbus.c``), which in turn wraps the
transport-independent UFS core (``hw/ufs/ufs.c`` and ``hw/ufs/lu.c``) that
also backs the PCI UFS device. The sysbus device provides the MMIO region,
the interrupt line, the DMA address space and the controller properties; the
core implements the UFSHCI registers, the UTP transfer and task list
processing, the UPIU and query handling and the SCSI logical-unit logic.

The only AST2700-specific behaviour ``aspeed-ufs`` adds is the controller
version register value ``0x00000200``, the UFSHCI 2.0 interface the hardware
reports.

The clock/reset wrapper at ``0x12c08000`` (``aspeed,ast2700-ufscnr``) is left
as an ``UnimplementedDevice``.

Logical units
-------------

Storage is attached through ``ufs-lu`` devices on the controller's UFS bus
(``ufs-bus.0``), exactly as for the PCI UFS device. The Huygens OpenBMC image
is laid out for 512-byte sectors, so its logical unit is created with
``logical-block-size=512``.

Usage
-----

Attach a UFS image as logical unit 0 of the controller's UFS bus:

.. code-block:: console

  qemu-system-aarch64 -M huygens-bmc \
    -nodefaults \
    -blockdev node-name=fmc0,driver=file,filename=image-bmc \
    -device w25q01jvq,bus=ssi.0,cs=0,drive=fmc0 \
    -blockdev node-name=ufs0,driver=file,filename=ufs.img \
    -device ufs-lu,bus=ufs-bus.0,drive=ufs0,lun=0,logical-block-size=512 \
    -display none -serial mon:stdio

Please check :doc:`../../system/arm/aspeed` for more details on the
``huygens-bmc`` machine.
