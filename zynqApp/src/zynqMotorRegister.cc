/*
FILENAME...	ZynqMotorRegister.cc
USAGE...	Register Zynq motor device driver shell commands.

*/

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <iocsh.h>
#include "ZynqMotorRegister.h"
#include "epicsExport.h"

extern "C"
{

// ACS Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// ACS Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const ZynqMotorSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const ZynqMotorConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupZynqMotor  = {"ZynqMotorSetup",  2, ZynqMotorSetupArgs};
static const iocshFuncDef configZynqMotor = {"ZynqMotorConfig", 2, ZynqMotorConfigArgs};

static void setupZynqMotorCallFunc(const iocshArgBuf *args)
{
    ZynqMotorSetup(args[0].ival, args[1].ival);
}
static void configZynqMotorCallFunc(const iocshArgBuf *args)
{
    ZynqMotorConfig(args[0].ival, args[1].sval);
}

static void ZynqMotorRegister(void)
{
    iocshRegister(&setupZynqMotor, setupZynqMotorCallFunc);
    iocshRegister(&configZynqMotor, configZynqMotorCallFunc);
}

epicsExportRegistrar(ZynqMotorRegister);

} // extern "C"
