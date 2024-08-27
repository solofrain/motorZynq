TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = zynq
# zynq.dbd will be created and installed
DBD += zynq.dbd

# zynq.dbd will be made up from these files:
zynq_DBD += base.dbd

# Include dbd files from all support applications:
zynq_DBD += asyn.dbd
zynq_DBD += motorSupport.dbd
zynq_DBD += zynqMotorSupport.dbd

# Add all the support libraries needed by this IOC
zynq_LIBS += zynqMotor
zynq_LIBS += motor
zynq_LIBS += asyn


# zynq_registerRecordDeviceDriver.cpp derives from zynq.dbd
zynq_SRCS += zynq_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
zynq_SRCS_DEFAULT += zynqMain.cpp
zynq_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#zynq_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
zynq_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

