deps_config := \
	src/fs/Kconfig \
	src/dev/Kconfig \
	Kconfig

.config include/autoconf.h: $(deps_config)

$(deps_config):
