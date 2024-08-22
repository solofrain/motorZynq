/*
FILENAME... zynqDriver.cpp
USAGE...    Motor driver support for the zynq (Direct Register Control) controller implemented in Zynq.

Ji Li
08/14/2024

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "ZynqMotorDriver.h"
#include "axi_reg.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new zynqController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] zynqPortName       The name of the drvAsynSerialPort that was created previously
                                 to connect to the zynq controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
zynqController::zynqController( const char *portName,
                              const char *zynqPortName,
                              int numAxes, 
                              double movingPollPeriod,
                              double idlePollPeriod )
  :  asynMotorController( portName, numAxes, NUM_zynq_PARAMS, 
                          0, // No additional interfaces beyond those in base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                          1, // autoconnect
                          0, 0)  // Default priority and stack size
{
    std::cout << __func__ << ": creating zynqController object..." << std:: endl;
    try
    {
        reg_p = std::make_unique<axi_reg>(reg_base_addr);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Unable to create axi_reg object in zynqController constructor. Try root.");
    }

    int axis;
    asynStatus status;
    zynqAxis *pAxis;
    static const char *functionName = "zynqController::zynqController";
  
    /* Connect to zynq controller */
    status = pasynOctetSyncIO->connect(zynqPortName, 0, &pasynUserController_, NULL);
    if (status)
    {
        asynPrint( this->pasynUserSelf, ASYN_TRACE_ERROR, 
                   "%s: cannot connect to MCB-4B controller\n",
                   functionName);
    }
    
    for (axis=0; axis<numAxes; axis++)
    {
        pAxis = new zynqAxis(this, axis);
    }
  
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

zynqController::~zynqController()
{
    std::cout << __func__ << ": zynqController object destructed." << std::endl;
}

/** Creates a new zynqController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] zynqPortName       The name of the drvAsynIPPPort that was created previouslyi
                                 to connect to the zynq controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
//extern "C" int zynqCreateController(const char *portName, const char *zynqPortName, int numAxes, 
//                                   int movingPollPeriod, int idlePollPeriod)
//{
//    zynqController *pzynqController
//        = new zynqController(portName, zynqPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
//    
//    pzynqController = NULL;
//    return(asynSuccess);
//}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void zynqController::report(FILE *fp, int level)
{
    fprintf( fp,
             "Zynq motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
             this->portName,
             numAxes_,
             movingPollPeriod_,
             idlePollPeriod_ );
  
    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an zynqAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
zynqAxis* zynqController::getAxis(asynUser *pasynUser)
{
    return static_cast<zynqAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an zynqAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
zynqAxis* zynqController::getAxis(int axisNo)
{
    return static_cast<zynqAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the zynqAxis methods

/** Creates a new zynqAxis object.
  * \param[in] pC Pointer to the zynqController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
zynqAxis::zynqAxis(zynqController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC)
{  
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void zynqAxis::report(FILE *fp, int level)
{
    if (level > 0)
    {
        fprintf( fp,
                 "  axis %d\n",
                 axisNo_ );
    }

    // Call the base class method
    asynMotorAxis::report(fp, level);
}

asynStatus zynqAxis::sendAccelAndVelocity( double acceleration, double velocity ) 
{
    asynStatus status;
    int ival;
    // static const char *functionName = "zynq::sendAccelAndVelocity";
  
    // Send the velocity
    ival = NINT( fabs(115200./velocity) );
    if (ival < 2) ival=2;
    if (ival > 255) ival = 255;
    sprintf(pC_->outString_, "#%02dV=%d", axisNo_, ival);
    status = pC_->writeReadController();
  
    // Send the acceleration
    // acceleration is in steps/sec/sec
    // MCB is programmed with Ramp Index (R) where:
    //   dval (steps/sec/sec) = 720,000/(256-R) */
    //   or R=256-(720,000/dval) */
    ival = NINT(256-(720000./acceleration));
    if (ival < 1) ival=1;
    if (ival > 255) ival=255;
    sprintf(pC_->outString_, "#%02dR=%d", axisNo_, ival);
    status = pC_->writeReadController();
    return status;
}


asynStatus zynqAxis::move( double position,
                          int relative,
                          double minVelocity,
                          double maxVelocity,
                          double acceleration )
{
    asynStatus status;
    // static const char *functionName = "zynqAxis::move";
  
    status = sendAccelAndVelocity(acceleration, maxVelocity);
    
    if (relative)
    {
        sprintf(pC_->outString_, "#%02dI%+d", axisNo_, NINT(position));
    }
    else
    {
        sprintf(pC_->outString_, "#%02dG%+d", axisNo_, NINT(position));
    }
    status = pC_->writeReadController();
    return status;
}

asynStatus zynqAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    asynStatus status;
    // static const char *functionName = "zynqAxis::home";
  
    status = sendAccelAndVelocity(acceleration, maxVelocity);
  
    if (forwards)
    {
        sprintf(pC_->outString_, "#%02dH+", axisNo_);
    }
    else
    {
        sprintf(pC_->outString_, "#%02dH-", axisNo_);
    }
    status = pC_->writeReadController();
    return status;
}

asynStatus zynqAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "zynqAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  status = sendAccelAndVelocity(acceleration, maxVelocity);

  /* MCB-4B does not have jog command. Move 1 million steps */
  if (maxVelocity > 0.) {
    /* This is a positive move in zynq coordinates */
    sprintf(pC_->outString_, "#%02dI+1000000", axisNo_);
  } else {
      /* This is a negative move in zynq coordinates */
      sprintf(pC_->outString_, "#%02dI-1000000", axisNo_);
  }
  status = pC_->writeReadController();
  return status;
}

asynStatus zynqAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "zynqAxis::stop";

  sprintf(pC_->outString_, "#%02dQ", axisNo_);
  status = pC_->writeReadController();
  return status;
}

asynStatus zynqAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "zynqAxis::setPosition";

  sprintf(pC_->outString_, "#%02dP=%+d", axisNo_, NINT(position));
  status = pC_->writeReadController();
  return status;
}

asynStatus zynqAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "zynqAxis::setClosedLoop";

  sprintf(pC_->outString_, "#%02dW=%d", axisNo_, closedLoop ? 1:0);
  status = pC_->writeReadController();
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus zynqAxis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int limit;
  double position;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "#%02dP", axisNo_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "#01P=+1000"
  position = atof(&pC_->inString_[5]);
  setDoubleParam(pC_->motorPosition_, position);

  // Read the moving status of this motor
  sprintf(pC_->outString_, "#%02dX", axisNo_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "#01X=1"
  done = (pC_->inString_[5] == '0') ? 1:0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the limit status
  sprintf(pC_->outString_, "#%02dE", axisNo_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "#01E=1"
  limit = (pC_->inString_[5] == '1') ? 1:0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (pC_->inString_[6] == '1') ? 1:0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
  limit = (pC_->inString_[7] == '1') ? 1:0;
  setIntegerParam(pC_->motorStatusAtHome_, limit);

  // Read the drive power on status
  sprintf(pC_->outString_, "#%02dW", axisNo_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  driveOn = (pC_->inString_[5] == '1') ? 1:0;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg zynqCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg zynqCreateControllerArg1 = {"MCB-4B port name", iocshArgString};
static const iocshArg zynqCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg zynqCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg zynqCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const zynqCreateControllerArgs[] = {&zynqCreateControllerArg0,
                                                             &zynqCreateControllerArg1,
                                                             &zynqCreateControllerArg2,
                                                             &zynqCreateControllerArg3,
                                                             &zynqCreateControllerArg4};
static const iocshFuncDef zynqCreateControllerDef = {"zynqCreateController", 5, zynqCreateControllerArgs};
static void zynqCreateContollerCallFunc(const iocshArgBuf *args)
{
  zynqCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void zynqRegister(void)
{
  iocshRegister(&zynqCreateControllerDef, zynqCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(zynqRegister);
}
