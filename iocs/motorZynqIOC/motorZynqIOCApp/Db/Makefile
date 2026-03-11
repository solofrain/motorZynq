TOP = ../..
include $(TOP)/configure/CONFIG

DB += motor.substitutions
DB += zynqMotor.substitutions

# Install motor database from motor module
DB_INSTALLS += $(MOTOR)/motorApp/Db/basic_asyn_motor.db

# Install custom zynqMotor template from motorZynq module
DB_INSTALLS += $(MOTORZYNQ)/motorZynqApp/Db/zynqMotor.template

include $(TOP)/configure/RULES
