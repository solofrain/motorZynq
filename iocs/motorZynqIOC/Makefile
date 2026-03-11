TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += motorZynqIOCApp
motorZynqIOCApp_DEPEND_DIRS = configure

DIRS += iocBoot
iocBoot_DEPEND_DIRS = motorZynqIOCApp

include $(TOP)/configure/RULES_TOP
