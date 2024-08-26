#!../../bin/linux-arm/zynq

#- You may have to change zynq to something else
#- everywhere it appears in this file

< envPaths

epicsEnvSet("P",  "Zynq")
epicsEnvSet("R",  "{Mtr:1}")
epicsEnvSet("PORT", "MMAP")

cd "${TOP}"

## Register all support components
dbLoadDatabase "${MOTOR}/dbd/zynq.dbd"
zynq_registerRecordDeviceDriver(pdbbase)

zynqMotorCreateController("ZynqMotor", "MMAP", 2, 250, 1000)

## Load record instances
#dbLoadRecords("db/xxx.db","user=liji")
dbLoadRecords("$(MOTOR)/db/asyn_motor.db","P=zynq,M=Motor,DTYP=asynMotor,PORT=${PORT},ADDR=0,DESC=X,EGU=mm,DIR=Pos,VELO=1,VBAS=.1,ACCL=.2,BDST=0,BVEL=1,BACC=.2,MRES=.0025,PREC=4,DHLM=100,DLLM=-100,INIT=")

dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(P),R=asyn1,PORT=${PORT},ADDR=0,OMAX=0,IMAX=0")


cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=liji"
