# Test that Linux kernel boots on microwatt powerpc machine
# Copyright 2022 IBM Corp.
# SPDX-License-Identifier: GPL-2.0-or-later

from avocado.utils import archive
from avocado_qemu import QemuSystemTest
from avocado_qemu import wait_for_console_pattern

class microwattMachine(QemuSystemTest):
    """
    :avocado: tags=arch:ppc64
    :avocado: tags=machine:microwatt
    """
    timeout = 100

    panic_message = 'Kernel panic - not syncing'
    console_pattern = 'Welcome to Buildroot'

    def test_ppc64le_microwatt_kernel(self):
        kernel_url = ('https://ozlabs.org/~joel/microwatt/dtbImage.microwatt.elf')
        kernel_hash = '6f2be4bcbdb10aaca38679a7edce91b5bcaaf42b'
        kernel_path = self.fetch_asset(kernel_url, asset_hash=kernel_hash)

        self.vm.set_console()
        self.vm.add_args('-kernel', kernel_path)
        self.vm.launch()
        wait_for_console_pattern(self, self.console_pattern, self.panic_message)

    def test_ppc64le_microwatt_flash(self):
        flash_url = ('https://ozlabs.org/~joel/microwatt/flash-microwatt')
        flash_hash = '7dd88cedc7e2beeeab17c5d56573d687a63911fc'
        flash_path = self.fetch_asset(flash_url, asset_hash=flash_hash)

        self.vm.set_console()
        self.vm.add_args('-drive', 'file=' + flash_path + ',if=mtd,format=raw')
        self.vm.launch()
        wait_for_console_pattern(self, self.console_pattern, self.panic_message)
