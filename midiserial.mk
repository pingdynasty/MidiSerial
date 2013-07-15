MIDISERIAL_VERSION = 0.1
MIDISERIAL_LICENSE = GPL
MIDISERIAL_SITE_METHOD = file
MIDISERIAL_SITE = $($(PKG)_DIR_PREFIX)/midiserial/

define MIDISERIAL_EXTRACT_CMDS
        $(INFLATE.gz) $(DL_DIR)/$(MIDISERIAL_SOURCE) |  $(TAR) $(TAR_STRIP_COMPONENTS)=0 -C $(@D)/ $(TAR_OPTIONS) -

endef

define MIDISERIAL_BUILD_CMDS
	$(TARGET_CONFIGURE_OPTS) $(MAKE) -C $(@D)
endef

define MIDISERIAL_CLEAN_CMDS
	$(MAKE) -C $(@D) clean
endef

define MIDISERIAL_INSTALL_TARGET_CMDS
	$(INSTALL) -D $(@D)/build/MidiSerial $(TARGET_DIR)/usr/bin/midiserial
endef

$(eval $(generic-package))
