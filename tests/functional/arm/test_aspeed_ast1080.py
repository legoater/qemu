#!/usr/bin/env python3
#
# Functional test that boots the ASPEED SoCs with firmware
#
# Copyright (C) 2026 ASPEED Technology Inc
#
# SPDX-License-Identifier: GPL-2.0-or-later

from aspeed import AspeedTest
from qemu_test import Asset, exec_command_and_wait_for_pattern


class AST1080Machine(AspeedTest):

    ASSET_ASPEED_AST1080_EVB_01_00 = Asset(
        ('https://github.com/AspeedTech-BMC'
         '/aspeed-mcu-fw-hub/releases/download/v01.00'
         '/ast1080a0-evb-v01.00.tar.gz'),
         '7c8a95bc51945d70591b20f1a22f8fffebc2b164e6fda47f73ff67f37bed8d87')

    def test_arm_ast1080_zephyros(self):
        self.set_machine('ast1080-evb')

        kernel_name = ('ast1080a0-evb-v01.00/ast1080/ast1080a0/evb/'
                       'zephyr-shell-module.bin')
        kernel_file = self.archive_extract(
            self.ASSET_ASPEED_AST1080_EVB_01_00, member=kernel_name)

        self.vm.set_console()
        self.vm.add_args('-kernel', kernel_file, '-nographic')
        self.vm.launch()
        self.wait_for_console_pattern("uart:~$")
        exec_command_and_wait_for_pattern(self, "help",
                                          "Available commands")

if __name__ == '__main__':
    AspeedTest.main()
