i3c_ss = ss.source_set()
i3c_ss.add(when: 'CONFIG_I3C', if_true: files('core.c'))
i3c_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_i3c.c'))
i3c_ss.add(when: 'CONFIG_DW_I3C', if_true: files('dw-i3c.c'))
i3c_ss.add(when: 'CONFIG_MOCK_I3C_TARGET', if_true: files('mock-i3c-target.c'))
system_ss.add_all(when: 'CONFIG_I3C', if_true: i3c_ss)
