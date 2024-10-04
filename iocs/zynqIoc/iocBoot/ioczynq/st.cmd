#!../../bin/linux-x86_64/zynq

#- You may have to change zynq to something else
#- everywhere it appears in this file

< envPaths

epicsEnvSet("Sys", "Det")
epicsEnvSet("Dev", "Mtr")
epicsEnvSet("Port", "ZMTR")

## Register all support components
dbLoadDatabase("../../dbd/zynq.dbd",0,0)
zynq_registerRecordDeviceDriver(pdbbase) 

## Load record instances
#dbLoadRecords("./zynqMotor.substitutions")
#dbLoadRecords("../../db/zynqMotor.substitutions", "MOTOR=${MOTOR}, P=$(Sys), PORT=$(Port)")

dbLoadRecords("$(MOTOR)/db/asyn_motor.db","P=$(Sys),M={${Dev}-Ax:X},DTYP=asynMotor,PORT=ZMTR,ADDR=0,DESC=X,EGU=mm,DIR=Pos,VELO=1,VBAS=.1,ACCL=.2,BDST=0,BVEL=1,BACC=.2,MRES=.0025,PREC=4,DHLM=100,DLLM=-100,INIT=")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db","P=$(Sys),M={${Dev}-Ax:Y},DTYP=asynMotor,PORT=ZMTR,ADDR=1,DESC=Y,EGU=mm,DIR=Pos,VELO=1,VBAS=.1,ACCL=.2,BDST=0,BVEL=1,BACC=.2,MRES=.0025,PREC=4,DHLM=100,DLLM=-100,INIT=")

zynqMotorCreateController($(Port), "ZMTR_MM", 2, 250, 1000)

iocInit()


dbl
