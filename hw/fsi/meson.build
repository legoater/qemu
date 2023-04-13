softmmu_ss.add(when: 'CONFIG_OPB', if_true: files('opb.c'))
softmmu_ss.add(when: 'CONFIG_FSI', if_true: files('fsi.c', 'fsi-master.c', 'fsi-slave.c', 'cfam.c'))
softmmu_ss.add(when: 'CONFIG_FSI', if_true: files('engine-scratchpad.c'))
softmmu_ss.add(when: 'CONFIG_APB2OPB_ASPEED', if_true: files('aspeed-apb2opb.c'))
