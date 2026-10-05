#!/usr/bin/env python3
#
# Functional test that boots the ASPEED SoCs with firmware
#
# Copyright (C) 2022 ASPEED Technology Inc
#
# SPDX-License-Identifier: GPL-2.0-or-later

import os

from aspeed import AspeedTest
from qemu_test import Asset, exec_command_and_wait_for_pattern


class AST1030Machine(AspeedTest):

    ASSET_ZEPHYR_3_08 = Asset(
        ('https://github.com/AspeedTech-BMC'
         '/zephyr/releases/download/v00.03.08/ast1030-evb-demo.zip'),
         '9eac3691bc7bce1b912bbe2ae4e36608a6532ff8d607f4d1e44b88407a48d4e5')

    def test_arm_ast1030_zephyros_3_08(self):
        self.set_machine('ast1030-evb')

        kernel_name = "ast1030-evb-demo/zephyr.elf"
        kernel_file = self.archive_extract(
            self.ASSET_ZEPHYR_3_08, member=kernel_name)

        self.vm.set_console()
        self.vm.add_args('-kernel', kernel_file, '-nographic')
        self.vm.launch()
        self.wait_for_console_pattern("Booting Zephyr OS")
        exec_command_and_wait_for_pattern(self, "help",
                                          "Available commands")

    ASSET_ZEPHYR_1_07 = Asset(
        ('https://github.com/AspeedTech-BMC'
         '/zephyr/releases/download/v00.01.07/ast1030-evb-demo.zip'),
        'ad52e27959746988afaed8429bf4e12ab988c05c4d07c9d90e13ec6f7be4574c')

    def test_arm_ast1030_zephyros_1_07(self):
        self.set_machine('ast1030-evb')

        kernel_name = "ast1030-evb-demo/zephyr.bin"
        kernel_file = self.archive_extract(
            self.ASSET_ZEPHYR_1_07, member=kernel_name)

        self.vm.set_console()
        self.vm.add_args('-kernel', kernel_file, '-nographic')
        self.vm.launch()
        self.wait_for_console_pattern("Booting Zephyr OS")
        for shell_cmd in [
                'kernel stacks',
                'otp info conf',
                'otp info scu',
                'hwinfo devid',
                'crypto aes256_cbc_vault',
                'random get',
                'jtag JTAG1 sw_xfer high TMS',
                'adc ADC0 resolution 12',
                'adc ADC0 read 42',
                'adc ADC1 read 69',
                'i2c scan I2C_0',
                'i3c attach I3C_0',
                'hash test',
                'kernel uptime',
                'kernel reboot warm',
                'kernel uptime',
                'kernel reboot cold',
                'kernel uptime',
        ]: exec_command_and_wait_for_pattern(self, shell_cmd, "uart:~$")

    ASSET_SDK_V1103_AST2600 = Asset(
        'https://github.com/AspeedTech-BMC/openbmc/releases/download/v11.03/ast2600-default-image.tar.gz',
        '47e3656a14bf7a4de28d3dfbf48bc2325443bc42d270f3bc82646f92f6dea165')

    def test_arm_ast1030_usbredir_to_ast2600(self):
        self.require_device('usb-redir-server')
        self.require_device('usb-redir')
        self.set_machine('ast2600-evb')
        self.set_machine('ast1030-evb')

        ast1030_kernel_file = self.archive_extract(
            self.ASSET_ZEPHYR_3_08, member="ast1030-evb-demo/zephyr.elf")

        ast2600_image_file = self.archive_extract(
            self.ASSET_SDK_V1103_AST2600,
            member="ast2600-default-image/image-bmc")
        sock = os.path.join(self.socket_dir().name, 'usbredir.sock')

        udc = self.get_vm(name='udc')
        udc.set_console()
        udc.add_args('-kernel', ast1030_kernel_file, '-nographic',
                     '-chardev',
                     f'socket,id=usbredir0,path={sock},server=on,wait=off',
                     '-device',
                     'usb-redir-server,id=udcredir,chardev=usbredir0',
                     '-device', 'aspeed.udc-gadget,bus=udcredir.0,'
                                'udc=/machine/soc/udc')
        udc.launch()
        self.wait_for_console_pattern('Booting Zephyr OS', vm=udc)
        exec_command_and_wait_for_pattern(self, 'usb enable', 'uart:~$',
                                          vm=udc)

        host = self.get_vm(name='host')
        host.set_machine('ast2600-evb')
        host.set_console()
        host.add_args('-drive',
                      f'file={ast2600_image_file},if=mtd,format=raw',
                      '-snapshot',
                      '-chardev', f'socket,id=usbredir0,path={sock}',
                      '-device', 'usb-redir,chardev=usbredir0,bus=usb-bus.1')
        host.launch()
        self.wait_for_console_pattern('Starting kernel ...', vm=host)
        self.wait_for_console_pattern('login:', vm=host)
        exec_command_and_wait_for_pattern(self, 'root', 'Password:', vm=host)
        exec_command_and_wait_for_pattern(self, '0penBmc',
                                          'root@ast2600-default:~#', vm=host)
        exec_command_and_wait_for_pattern(self, 'lsusb',
                                          'ZEPHYR Zephyr DFU sample',
                                          vm=host)

    def test_arm_ast1030_otp_blockdev_device(self):
        self.vm.set_machine("ast1030-evb")

        kernel_name = "ast1030-evb-demo/zephyr.elf"
        kernel_file = self.archive_extract(self.ASSET_ZEPHYR_3_08,
                                           member=kernel_name)
        otp_img = self.generate_otpmem_image()

        self.vm.set_console()
        self.vm.add_args(
            "-kernel", kernel_file,
            "-blockdev", f"driver=file,filename={otp_img},node-name=otp",
            "-global", "aspeed-otp.drive=otp",
        )
        self.vm.launch()
        self.wait_for_console_pattern("Booting Zephyr OS")

if __name__ == '__main__':
    AspeedTest.main()
