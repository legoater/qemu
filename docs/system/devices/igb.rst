.. SPDX-License-Identifier: GPL-2.0-or-later
.. _igb:

igb
---

igb is a family of Intel's gigabit ethernet controllers. In QEMU, 82576
emulation is implemented in particular. Its datasheet is available at [1]_.

This implementation is expected to be useful to test SR-IOV networking without
requiring physical hardware.

Limitations
===========

This igb implementation was tested with Linux Test Project [2]_ and Windows HLK
[3]_ during the initial development. Later it was also tested with DPDK Test
Suite [4]_. The command used when testing with LTP is:

.. code-block:: shell

  network.sh -6mta

Be aware that this implementation lacks many functionalities available with the
actual hardware, and you may experience various failures if you try to use it
with a different operating system other than DPDK, Linux, and Windows or if you
try functionalities not covered by the tests.

Using igb
=========

Using igb should be nothing different from using another network device. See
:ref:`Network_emulation` in general.

However, you may also need to perform additional steps to activate SR-IOV
feature on your guest. For Linux, refer to [5]_.

Developing igb
==============

igb is the successor of e1000e, and e1000e is the successor of e1000 in turn.
As these devices are very similar, if you make a change for igb and the same
change can be applied to e1000e and e1000, please do so.

Please do not forget to run tests before submitting a change. As tests included
in QEMU is very minimal, run some application which is likely to be affected by
the change to confirm it works in an integrated system.

Testing igb
===========

A qtest of the basic functionality is available. Run the below at the build
directory:

.. code-block:: shell

  pyvenv/bin/meson test qtest-x86_64/qos-test

ethtool can test register accesses, interrupts, etc. It is automated as an
functional test and can be run from the build directory with the following
command:

.. code:: shell

  pyvenv/bin/meson test --suite thorough func-x86_64-netdev_ethtool

VF Migration (experimental)
===========================

The igb device supports an experimental VF migration interface that allows
the ``igb-vfio-pci`` variant driver to migrate VF state during live
migration. This is enabled with the ``x-vf-migration`` property::

  -device igb,x-vf-migration=on,...

When enabled, each emulated VF advertises a vendor-specific PCI capability
(cap id 0x09) with a magic signature (``0x4D494742`` / "MIGB") that the
variant driver probes at bind time. The capability contains an interface
version number, the BAR index hosting the migration register region, and
feature flags indicating which migration features are supported.

This feature is experimental and the ``x-`` prefix indicates the interface
may change.

Migration BAR layout
~~~~~~~~~~~~~~~~~~~~

The migration BAR (BAR2, 64 KB) implements a VFIO-like state machine with
the following register layout::

  Offset  Name                Access  Description
  0x000   DEVICE_STATE        RW      Migration state (RUNNING=2, STOP=1,
                                      STOP_COPY=3, RESUMING=4, PRE_COPY=5)
  0x004   STATUS              RO      Flags[2:0]: DATA_AVAIL, ERROR, QUIESCED
                                      Error code[15:8] (when ERROR is set)
  0x008   CAPS                RO      F_STATE, F_DIRTY, max_ranges[11:8],
                                      pgsizes[31:12]
  0x00C   VERSION             RO      Interface version (1)
  0x010   DATA_SIZE           RW      Max state size at reset, actual after save
  0x014   DATA_XFER           WO      Trigger DMA save or DMA load
  0x018   DATA_BUF_ADDR_LO    WO      Low 32 bits of state DMA buffer address
  0x01C   DATA_BUF_ADDR_HI    WO      High 32 bits of state DMA buffer address

State transitions follow the VFIO migration state machine: the driver
writes to ``DEVICE_STATE`` to move between states and reads ``STATUS``
to check for completion.

State data is transferred via a driver-provided DMA buffer. The driver
writes its physical address to ``DATA_BUF_ADDR_LO/HI`` and triggers the
transfer with ``DATA_XFER``. The device DMA-writes the serialized state
on save and DMA-reads it on restore.

The state blob is a versioned sequence of register (offset, value)
pairs.

When ``STATUS`` has the ``ERROR`` bit set, bits [15:8] contain an error
code identifying the failure::

  0  (none)          No error
  1  BAD_MAGIC       State blob magic mismatch
  2  BAD_VERSION     State blob version mismatch
  3  BAD_SIZE        State blob too large or empty
  4  BAD_VFN         VF number mismatch (source != destination)
  5  DMA_FAILED      DMA transfer to/from state buffer failed
  6  NO_BUFFER       DATA_XFER without buffer address set

References
==========

.. [1] https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82576eb-gigabit-ethernet-controller-datasheet.pdf
.. [2] https://github.com/linux-test-project/ltp
.. [3] https://learn.microsoft.com/en-us/windows-hardware/test/hlk/
.. [4] https://doc.dpdk.org/dts/gsg/
.. [5] https://docs.kernel.org/PCI/pci-iov-howto.html
