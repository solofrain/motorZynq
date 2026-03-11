TOP = ../..
include $(TOP)/configure/CONFIG

# Build a support library
LIBRARY_IOC += motorZynq

# Source files
motorZynq_SRCS += zynq_reg.cpp
motorZynq_SRCS += zynqMotorDriver.cpp
motorZynq_SRCS += zynqMotorRegister.cpp

# DBD file
DBD += motorZynqSupport.dbd
motorZynqSupport_DBD += motorZynq.dbd

# Libraries
motorZynq_LIBS += motor
motorZynq_LIBS += asyn
motorZynq_LIBS += $(EPICS_BASE_IOC_LIBS)

# Compiler flags
USR_CXXFLAGS += -std=c++14

include $(TOP)/configure/RULES
