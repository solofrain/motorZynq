/*
 * zynqMotorRegister.cpp
 *
 * EPICS iocsh registration for zynqMotorCreateController command.
 */

#include <cstdlib>
#include <cstdint>

#include <iocsh.h>
#include <epicsExport.h>

#include "zynqMotorDriver.h"

/* ------------------------------------------------------------------ */
/*  zynqMotorCreateController                                          */
/* ------------------------------------------------------------------ */

static const iocshArg arg0 = {"Port name",          iocshArgString};
static const iocshArg arg1 = {"Number of axes",     iocshArgInt};
static const iocshArg arg2 = {"Base address (hex)",  iocshArgInt};
static const iocshArg arg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg arg4 = {"Idle poll period (ms)",   iocshArgInt};

static const iocshArg *const createControllerArgs[] = {
    &arg0, &arg1, &arg2, &arg3, &arg4
};

static const iocshFuncDef createControllerDef = {
    "zynqMotorCreateController", 5, createControllerArgs
};

static void createControllerCallFunc(const iocshArgBuf *args)
{
    new zynqMotorController(
        args[0].sval,                               /* portName */
        args[1].ival,                               /* numAxes */
        static_cast<uint32_t>(args[2].ival),        /* baseAddr */
        args[3].ival / 1000.0,                      /* movingPollPeriod (s) */
        args[4].ival / 1000.0                       /* idlePollPeriod (s) */
    );
}

/* ------------------------------------------------------------------ */
/*  Registration                                                       */
/* ------------------------------------------------------------------ */

static void zynqMotorRegisterFunc(void)
{
    iocshRegister(&createControllerDef, createControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(zynqMotorRegisterFunc);
}
