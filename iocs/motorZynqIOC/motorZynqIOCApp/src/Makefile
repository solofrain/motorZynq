TOP = ../..
include $(TOP)/configure/CONFIG

# Build the IOC application
PROD_IOC = motorZynqIOC

# Database definition files
DBD += motorZynqIOC.dbd
motorZynqIOC_DBD += base.dbd
motorZynqIOC_DBD += asyn.dbd
motorZynqIOC_DBD += motorSupport.dbd
motorZynqIOC_DBD += motorZynqSupport.dbd

# Libraries
motorZynqIOC_LIBS += motorZynq
motorZynqIOC_LIBS += motor
motorZynqIOC_LIBS += asyn
motorZynqIOC_LIBS += $(EPICS_BASE_IOC_LIBS)

# Source files
motorZynqIOC_SRCS += motorZynqIOC_registerRecordDeviceDriver.cpp
motorZynqIOC_SRCS += motorZynqIOCMain.cpp

include $(TOP)/configure/RULES
