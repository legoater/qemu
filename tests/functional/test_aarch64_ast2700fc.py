#!/usr/bin/env python3
#
# Functional test that boots the AST2700 multi-SoCs with firmware
#
# Copyright (C) 2025 ASPEED Technology Inc
#
# SPDX-License-Identifier: GPL-2.0-or-later

import os

from qemu_test import QemuSystemTest, Asset
from qemu_test import wait_for_console_pattern
from qemu_test import exec_command_and_wait_for_pattern


class AST2700fcMachineSDK(QemuSystemTest):
    ASSET_SDK_V905_AST2700 = Asset(
            'https://github.com/AspeedTech-BMC/openbmc/releases/download/v09.05/ast2700-default-obmc.tar.gz',
            'c1f4496aec06743c812a6e9a1a18d032f34d62f3ddb6956e924fef62aa2046a5')
    ASSET_SDK_V905_AST2700_A0 = Asset(
            'https://github.com/AspeedTech-BMC/openbmc/releases/download/v09.05/ast2700-a0-default-obmc.tar.gz',
            'cfbbd1cce72f2a3b73b9080c41eecdadebb7077fba4f7806d72ac99f3e84b74a')
    ASSET_SDK_SSP_TSP_AST2700_A0 = Asset(
            'https://github.com/AspeedTech-BMC/openbmc/releases/download/v09.05/ast2700-a0-ssp-tsp.tar.gz',
            'a3ca348bd1cb39086f9fdc40eab293afead351925a46f9ab0da4dcc3a1157382')

    def do_test_aarch64_ast2700fc_ca35_start(self, image):
        self.require_netdev('user')
        self.vm.set_console()
        self.vm.add_args('-drive', 'file=' + image + ',if=mtd,format=raw',
                         '-net', 'nic', '-net', 'user', '-snapshot')

        self.vm.launch()

        wait_for_console_pattern(self, 'U-Boot 2023.10')
        wait_for_console_pattern(self, '## Loading kernel from FIT Image')
        wait_for_console_pattern(self, 'Starting kernel ...')

    def do_test_aarch64_ast2700fc_ssp_start(self):

        self.vm.shutdown()
        self.vm.set_console(console_index=1)
        self.vm.launch()

    def do_test_aarch64_ast2700fc_tsp_start(self):
        self.vm.shutdown()
        self.vm.set_console(console_index=2)
        self.vm.launch()

    def start_ast2700fc_test(self, name):
        ca35_core = 4
        uboot_size = os.path.getsize(self.scratch_file(name,
                                                       'u-boot-nodtb.bin'))
        uboot_dtb_load_addr = hex(0x400000000 + uboot_size)

        load_images_list = [
            {
                'addr': '0x400000000',
                'file': self.scratch_file(name,
                                          'u-boot-nodtb.bin')
            },
            {
                'addr': str(uboot_dtb_load_addr),
                'file': self.scratch_file(name, 'u-boot.dtb')
            },
            {
                'addr': '0x430000000',
                'file': self.scratch_file(name, 'bl31.bin')
            },
            {
                'addr': '0x430080000',
                'file': self.scratch_file(name, 'optee',
                                          'tee-raw.bin')
            }
        ]

        for load_image in load_images_list:
            addr = load_image['addr']
            file = load_image['file']
            self.vm.add_args('-device',
                             f'loader,force-raw=on,addr={addr},file={file}')

        for i in range(ca35_core):
            self.vm.add_args('-device',
                             f'loader,addr=0x430000000,cpu-num={i}')

        if name == 'ast2700-a0-default':
            load_elf_list = {
                'ssp': self.scratch_file('ast2700-a0-ssp-tsp', 'ast2700-ssp.elf'),
                'tsp': self.scratch_file('ast2700-a0-ssp-tsp', 'ast2700-tsp.elf')
            }
        else:
            load_elf_list = {
                'ssp': self.scratch_file(name, 'ast2700-ssp.elf'),
                'tsp': self.scratch_file(name, 'ast2700-tsp.elf')
            }

        for cpu_num, key in enumerate(load_elf_list, start=4):
            file = load_elf_list[key]
            self.vm.add_args('-device',
                             f'loader,file={file},cpu-num={cpu_num}')

        self.vm.add_args('-device',
                         'tmp105,bus=aspeed.i2c.bus.1,address=0x4d,id=tmp-test')
        self.do_test_aarch64_ast2700fc_ca35_start(
            self.scratch_file(name, 'image-bmc'))

        wait_for_console_pattern(self, f'{name} login:')

        exec_command_and_wait_for_pattern(self, 'root', 'Password:')
        exec_command_and_wait_for_pattern(self, '0penBmc', f'root@{name}:~#')

        exec_command_and_wait_for_pattern(
                self,
                'echo lm75 0x4d > /sys/class/i2c-dev/i2c-1/device/new_device ',
                'i2c i2c-1: new_device: Instantiated device lm75 at 0x4d')
        exec_command_and_wait_for_pattern(
                self,
                'cat /sys/class/hwmon/hwmon*/temp1_input', '0')
        self.vm.cmd('qom-set', path='/machine/peripheral/tmp-test',
                    property='temperature', value=18000)
        exec_command_and_wait_for_pattern(
                self,
                'cat /sys/class/hwmon/hwmon*/temp1_input', '18000')

        self.do_test_aarch64_ast2700fc_ssp_start()

        exec_command_and_wait_for_pattern(self, '\012', 'ssp:~$')
        exec_command_and_wait_for_pattern(self, 'version',
                                          'Zephyr version 3.7.1')
        if name == 'ast2700-a0-default':
            exec_command_and_wait_for_pattern(self, 'md 72c02000 1',
                                              '[72c02000] 06000103')
        else:
            exec_command_and_wait_for_pattern(self, 'md 72c02000 1',
                                              '[72c02000] 06010103')
        self.do_test_aarch64_ast2700fc_tsp_start()
        exec_command_and_wait_for_pattern(self, '\012', 'tsp:~$')
        exec_command_and_wait_for_pattern(self, 'version',
                                          'Zephyr version 3.7.1')
        if name == 'ast2700-a0-default':
            exec_command_and_wait_for_pattern(self, 'md 72c02000 1',
                                              '[72c02000] 06000103')
        else:
            exec_command_and_wait_for_pattern(self, 'md 72c02000 1',
                                              '[72c02000] 06010103')

    def test_aarch64_ast2700fc_sdk_v09_05(self):
        self.set_machine('ast2700fc-a1')
        self.archive_extract(self.ASSET_SDK_V905_AST2700)
        self.start_ast2700fc_test('ast2700-default')

    def test_aarch64_ast2700fc_a0_sdk_v09_05(self):
        self.set_machine('ast2700fc-a0')
        self.archive_extract(self.ASSET_SDK_V905_AST2700_A0)
        self.archive_extract(self.ASSET_SDK_SSP_TSP_AST2700_A0)
        self.start_ast2700fc_test('ast2700-a0-default')


if __name__ == '__main__':
    QemuSystemTest.main()
