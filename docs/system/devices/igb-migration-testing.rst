.. SPDX-License-Identifier: GPL-2.0-or-later
.. _igb-migration-testing:

igb VF migration testing
------------------------

This document describes the network setup for testing igb VF live
migration in a nested virtualization environment. See :ref:`igb` for
the device documentation and migration BAR register layout.

NetworkManager configuration
=============================

In a nested setup, the L1 VMs (source and destination) have emulated
igb PFs connected to the L0 bridge. By default, NetworkManager
acquires DHCP leases on those PF interfaces and on any igb VFs created
later. This causes the VF MAC address to be learned on the L0 bridge,
which can misdirect iperf3 traffic after migration.

To prevent this, configure NetworkManager on **both L1 VMs** and on the
**L2 guest disk image**.

L1 VMs (source and destination)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Prevent NetworkManager from managing igbvf interfaces:

.. code-block:: bash

   cat > /etc/NetworkManager/conf.d/99-no-igbvf.conf <<EOF
   [keyfile]
   unmanaged-devices=driver:igbvf
   EOF

2. Disable IP on the igb PF connections (keep the interfaces UP for
   bridging, but with no DHCP lease):

.. code-block:: bash

   # Identify the NM connections for the igb PFs (NOT the virtio management NIC)
   nmcli -t -f NAME,DEVICE connection show

   # For each igb PF connection:
   nmcli connection modify "<igb-pf-connection>" ipv4.method disabled ipv6.method disabled

3. Reload NetworkManager:

.. code-block:: bash

   nmcli general reload

L2 guest disk image
~~~~~~~~~~~~~~~~~~~

Use ``virt-customize`` to add the igbvf unmanaged config to the guest
image (offline, before any test run):

.. code-block:: bash

   virt-customize -a /srv/migration/rhel10.qcow2 \
     --write /etc/NetworkManager/conf.d/99-no-igbvf.conf:'[keyfile]
   unmanaged-devices=driver:igbvf'

Network diagram
===============

The diagram below shows the nested setup where the source and
destination hosts are themselves VMs (L1) running on a physical
host (L0) that emulates the igb NIC::

  ┌────────────────────────────────────────────────────────────────────────────┐
  │  L0: physical host                                                         │
  │                                                                            │
  │  virbr0  192.168.199.1/24                                                  │
  │  ├── NFS server: /srv/migration                                            │
  │  └── iperf3 client: iperf3 -c 192.168.199.200 -t 60 -i 1                   │
  │      │                                                                     │
  │      │  L0 virbr0 bridge (192.168.199.0/24)                                │
  │  ────┼──────────┬──────────────────┬──────────────────────────────         │
  │      │          │                  │                                       │
  │      │     ┌────┴────┐        ┌────┴────┐                                  │
  │      │     │ virtio  │        │ emulated│                                  │
  │      │     │c0:ff:ee:│        │  igb PF │    L0 QEMU (vm6-fedora)          │
  │      │     │ :00:06  │        │ + igbvf │    tracks DMA dirty pages        │
  │      │     └────┬────┘        └────┬────┘                                  │
  │      │          │                  │                                       │
  │  ┌───┼──────────┼──────────────────┼──────────────────────────────────┐    │
  │  │   │  L1: vm6 (source)           │                                  │    │
  │  │   │  enp1s0: 192.168.199.6      │                                  │    │
  │  │   │  (management)               │                                  │    │
  │  │   │                        enp8s0 (igb PF, no IP)                  │    │
  │  │   │                             │                                  │    │
  │  │   │                        igb VF0 ──► igb-vfio-pci (VFIO)         │    │
  │  │   │                             │      dirty_sync → L0 igbvf       │    │
  │  │   │                             │                                  │    │
  │  │   │   virbr0                    │ VFIO passthrough                 │    │
  │  │   │   192.168.200.1/24          │                                  │    │
  │  │   │       │                     │                                  │    │
  │  │   │  ┌────┼─────────────────────┼───────────────────────────┐      │    │
  │  │   │  │    │  L2: rhel10 guest   │                           │      │    │
  │  │   │  │    │                     │                           │      │    │
  │  │   │  │  virtio NIC           igb VF (enp7s0)                │      │    │
  │  │   │  │  192.168.200.130/24   192.168.199.200/24             │      │    │
  │  │   │  │  (SSH login)          (iperf3 data path)             │      │    │
  │  │   │  │                          │                           │      │    │
  │  │   │  │            iperf3 -s -D  │ (listens on 0.0.0.0)      │      │    │
  │  │   │  └──────────────────────────┼───────────────────────────┘      │    │
  │  │   │                             │                                  │    │
  │  │   │  virsh migrate --live ──────┼──────────────────► vm7           │    │
  │  │   │                             │                                  │    │
  │  └───┼─────────────────────────────┼──────────────────────────────────┘    │
  │      │                             │                                       │
  │      │          iperf3 traffic     │                                       │
  │      └─────────────────────────────┘                                       │
  │                                                                            │
  │  ────────────────────────────────────────────────────────────────          │
  │      │                  │                                                  │
  │      │     ┌────────┐   │   ┌─────────┐                                    │
  │      │     │ virtio │   │   │emulated │    L0 QEMU (vm7-fedora)            │
  │      │     │c0:ff:ee│   │   │ igb PF  │                                    │
  │      │     │ :00:07 │   │   │ + igbvf │                                    │
  │      │     └────┬───┘   │   └────┬────┘                                    │
  │  ┌──────────────┼───────┼────────┼────────────────────────────────────┐    │
  │  │   L1: vm7 (destination)       │                                    │    │
  │  │   enp1s0: 192.168.199.7       │                                    │    │
  │  │   (management)           enp8s0 (igb PF, no IP)                    │    │
  │  │                               │                                    │    │
  │  │                          igb VF0 ──► igb-vfio-pci (VFIO)           │    │
  │  │                               │                                    │    │
  │  │   virbr0                      │ VFIO passthrough                   │    │
  │  │   192.168.200.1/24            │                                    │    │
  │  │       │                       │                                    │    │
  │  │  ┌────┼───────────────────────┼────────────────────────────┐       │    │
  │  │  │    │  L2: rhel10 (after migration)                      │       │    │
  │  │  │    │                       │                            │       │    │
  │  │  │  virtio NIC             igb VF (enp7s0)                 │       │    │
  │  │  │  192.168.200.130/24     192.168.199.200/24              │       │    │
  │  │  │                            │                            │       │    │
  │  │  │              iperf3 -s -D  │ (connection survives)      │       │    │
  │  │  └────────────────────────────┼────────────────────────────┘       │    │
  │  └───────────────────────────────┼────────────────────────────────────┘    │
  │                                  │                                         │
  │      iperf3 traffic resumes ─────┘                                         │
  │      (same IP, same MAC, same L2 segment → transparent to client)          │
  └────────────────────────────────────────────────────────────────────────────┘
