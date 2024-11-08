#!../../bin/linux-arm/zynq

#- You may have to change zynq to something else
#- everywhere it appears in this file

< envPaths

epicsEnvSet("Sys", "Det")
epicsEnvSet("Dev", "Mtr")
epicsEnvSet("Port", "ZMTR")

cd $(TOP)

## Register all support components
dbLoadDatabase("dbd/zynq.dbd",0,0)
zynq_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadTemplate("db/zynqMotor.substitutions", "Sys=$(Sys),Dev=$(Dev),Port=$(Port)")

## Create motor controller
zynqMotorCreateController($(Port), "ZMTR_MM", 2, 250, 1000)
#zynqMotorCreateController($(Port), "ZMTR_MM", 2, 10000, 10000)


## autosave/restore machinery
save_restoreSet_Debug(0)
save_restoreSet_IncompleteSetsOk(1)
save_restoreSet_DatedBackupFiles(1)

set_savefile_path("${TOP}/as","/save")
set_requestfile_path("${TOP}/as","/req")

set_pass0_restoreFile("info_positions.sav")
set_pass0_restoreFile("info_settings.sav")
set_pass1_restoreFile("info_settings.sav")


iocInit()

cd ${TOP}/as/req
makeAutosaveFiles()
create_monitor_set("info_positions.req", 5 , "")
create_monitor_set("info_settings.req", 15 , "")


dbl
