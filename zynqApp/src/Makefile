# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

#=============================
# build a support library
#LIBRARY_IOC = zynqMotor
PROD_IOC = zynqMotor

#DBD += zynqMotorSupport.dbd
#zynqMotorSupport_DBD += zynqMotor.dbd

DBD += zynq.dbd
zynq_DBD += base.dbd
zynq_DBD += asyn.dbd
zynq_DBD += motorSupport.dbd
zynq_DBD += zynqMotorSupport.dbd

#SRCS += ZynqMotorRegister.cc

# Advanced Control Systems driver support.
#SRCS += devZynqMotor.c drvMCB4B.c
zynqMotor_SRCS += zynqMotorDriver.cpp axi_reg.cc spinlock.cc

zynqMotor_LIBS += zynq motor asyn
zynqMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

