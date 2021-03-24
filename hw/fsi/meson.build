arm_ss.add(when: 'CONFIG_OPB', if_true: files('opb.c'))
arm_ss.add(when: 'CONFIG_FSI', if_true: files('fsi.c', 'fsi-master.c', 'fsi-slave.c', 'cfam.c'))
arm_ss.add(when: 'CONFIG_FSI', if_true: files('engine-scratchpad.c'))
arm_ss.add(when: 'CONFIG_APB2OPB_ASPEED', if_true: files('aspeed-apb2opb.c'))
