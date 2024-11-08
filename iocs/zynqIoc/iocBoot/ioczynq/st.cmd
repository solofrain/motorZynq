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

#dbLoadRecords("$(MOTOR)/db/asyn_motor.db","P=$(Sys),M={${Dev}-Ax:X},DTYP=asynMotor,PORT=ZMTR,ADDR=0,DESC=X,EGU=mm,DIR=Pos,VELO=1,VBAS=.1,ACCL=.2,BDST=0,BVEL=1,BACC=.2,MRES=.0025,PREC=4,DHLM=0,DLLM=0,INIT=")
#dbLoadRecords("$(MOTOR)/db/asyn_motor.db","P=$(Sys),M={${Dev}-Ax:Y},DTYP=asynMotor,PORT=ZMTR,ADDR=1,DESC=Y,EGU=mm,DIR=Pos,VELO=1,VBAS=.1,ACCL=.2,BDST=0,BVEL=1,BACC=.2,MRES=.01,PREC=4,DHLM=0,DLLM=0,INIT=")

#zynqMotorCreateController($(Port), "ZMTR_MM", 2, 250, 1000)
zynqMotorCreateController($(Port), "ZMTR_MM", 2, 10000, 10000)

iocInit()


dbl
