#!../../bin/linux-x86_64/zynq

#- You may have to change zynq to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/zynq.dbd",0,0)
zynq_registerRecordDeviceDriver(pdbbase) 

## Load record instances
#dbLoadRecords("../../db/zynq.db","user=liji")

iocInit()

## Start any sequence programs
#seq snczynq,"user=liji"
