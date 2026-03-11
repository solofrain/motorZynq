TOP = ..
include $(TOP)/configure/CONFIG
TARGETS = $(CONFIG_TARGETS)
CONFIGS += $(subst ../,,$(wildcard $(foreach dir,$(INSTALL_LOCATION_LIB) $(INSTALL_LOCATION_BIN),$(addsuffix /RULES*,$(dir)))))
include $(TOP)/configure/RULES
