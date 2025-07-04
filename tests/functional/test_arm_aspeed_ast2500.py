#!/usr/bin/env python3
#
# Functional test that boots the ASPEED machines
#
# SPDX-License-Identifier: GPL-2.0-or-later

from qemu_test import Asset, exec_command_and_wait_for_pattern
from aspeed import AspeedTest


class AST2500Machine(AspeedTest):

    ASSET_BR2_202411_AST2500_FLASH = Asset(
        ('https://github.com/legoater/qemu-aspeed-boot/raw/master/'
         'images/ast2500-evb/buildroot-2024.11/flash.img'),
        '641e6906c18c0f19a2aeb48099d66d4771929c361001d554d0d45c667413e13a')

    def test_arm_ast2500_evb_buildroot(self):
        self.set_machine('ast2500-evb')

        image_path = self.ASSET_BR2_202411_AST2500_FLASH.fetch()

        self.vm.add_args('-device',
                         'tmp105,bus=aspeed.i2c.bus.3,address=0x4d,id=tmp-test')
        self.do_test_arm_aspeed_buildroot_start(image_path, '0x0',
                                                'ast2500-evb login:')

        exec_command_and_wait_for_pattern(self,
             'echo lm75 0x4d > /sys/class/i2c-dev/i2c-3/device/new_device',
             'i2c i2c-3: new_device: Instantiated device lm75 at 0x4d')
        exec_command_and_wait_for_pattern(self,
                             'cat /sys/class/hwmon/hwmon1/temp1_input', '0')
        self.vm.cmd('qom-set', path='/machine/peripheral/tmp-test',
                    property='temperature', value=18000)
        exec_command_and_wait_for_pattern(self,
                             'cat /sys/class/hwmon/hwmon1/temp1_input', '18000')

        self.do_test_arm_aspeed_buildroot_poweroff()

    ASSET_SDK_V906_AST2500 = Asset(
        'https://github.com/AspeedTech-BMC/openbmc/releases/download/v09.06/ast2500-default-obmc.tar.gz',
        '542db84645b4efd8aed50385d7f4dd1caff379a987032311cfa7b563a3addb2a')

    def test_arm_ast2500_evb_sdk(self):
        self.set_machine('ast2500-evb')

        self.archive_extract(self.ASSET_SDK_V906_AST2500)

        self.do_test_arm_aspeed_sdk_start(
            self.scratch_file("ast2500-default", "image-bmc"))

        self.wait_for_console_pattern('ast2500-default login:')

    ASSET_SDK_V906_AST2500_515 = Asset(
        'https://github.com/AspeedTech-BMC/openbmc/releases/download/v09.06/ast2500-default-515-obmc.tar.gz',
        '70fe5fb511b2bbc561133eaa5ea82b7098dd4c64c2fcaf71c162d0f30aa85b3b')

    def test_arm_ast2500_evb_sdk_515(self):
        self.set_machine('ast2500-evb')

        self.archive_extract(self.ASSET_SDK_V906_AST2500_515)

        self.do_test_arm_aspeed_sdk_start(
            self.scratch_file("ast2500-default-515", "image-bmc"))

        self.wait_for_console_pattern('ast2500-default-515 login:')


if __name__ == '__main__':
    AspeedTest.main()
