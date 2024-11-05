/*
FILENAME... zynqMotorDriver.cpp
USAGE...    Motor driver support for Zynq motor controller.

Ji Li
08/14/2024

*/
#include <iostream>
#include <memory>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

//#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "zynqMotorDriver.h"
#include "axi_reg.h"

using std::cout;
using std::endl;

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new zynqMotorController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] zynqPortName       The name of the drvAsynSerialPort that was created previously
                                 to connect to the zynq controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
zynqMotorController::zynqMotorController( const char *portName,
                                          const char *zynqPortName,
                                          int numAxes, 
                                          double movingPollPeriod,
                                          double idlePollPeriod )
  :  asynMotorController( portName, numAxes, NUM_ZYNQ_PARAMS, 
                          asynUInt32DigitalMask,
                          asynUInt32DigitalMask,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                          1, // autoconnect
                          0, 0)  // Default priority and stack size
{
    cout << __func__ << ": creating zynqMotorController object..." << endl;

    zynqMotorAxis* pAxis;

#ifndef DBG
    try
    {
        reg_p = std::make_unique<axi_reg>(regBaseAddress);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Unable to create axi_reg object in zynqMotorController constructor. Try root.");
    }
#endif

    //asynStatus status;
    //zynqMotorAxis *pAxis;
    //static const char *functionName = "zynqMotorController::zynqMotorController";
      
    for (int axis=0; axis<numAxes; axis++)
    {
        pAxis = new zynqMotorAxis(this, axis);
        //new zynqMotorAxis(this, axis);
    }
  
    cout << "Start poller..." << endl;
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

zynqMotorController::~zynqMotorController()
{
    cout << __func__ << ": zynqMotorController object destructed." << endl;
}

/** Creates a new zynqMotorController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] zynqPortName       The name of the drvAsynIPPPort that was created previouslyi
                                 to connect to the zynq controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int zynqMotorCreateController( const char *portName, const char *zynqPortName, int numAxes, 
                                          int movingPollPeriod, int idlePollPeriod)
{
    print_func;

    zynqMotorController *pzynqMotorController
        = new zynqMotorController(portName, zynqPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    
    pzynqMotorController = NULL;

    return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */

void zynqMotorController::report(FILE *fp, int level)
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

/** Returns a pointer to an zynqMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
zynqMotorAxis* zynqMotorController::getAxis(asynUser *pasynUser)
{
    //print_func;
    return static_cast<zynqMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an zynqMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
zynqMotorAxis* zynqMotorController::getAxis(int axisNo)
{
    //print_func;
    return static_cast<zynqMotorAxis*>(asynMotorController::getAxis(axisNo));
}


void zynqMotorController::writeReg32( int axisNo, uint32_t offset, uint32_t value )
{
    print_func;
    //volatile uint32_t* regAddr = reinterpret_cast<volatile uint32_t*>(regBaseAddress + getAxisOffset(axisNo) + offset);
    uint32_t regAddr = getAxisOffset( axisNo ) + offset;
    cout << __func__
	 << ": write " << value
	 << " to register " << regAddr 
	 << " for axis " << axisNo
	 << endl;
#ifndef DBG
    //*regAddr = value;
    reg_p->reg_wr( getAxisOffset(axisNo) + offset, value );
#endif
}


//void zynqMotorController::readReg32( int axisNo, uint32_t offset, uint32_t value )
void zynqMotorController::readReg32( int axisNo, epicsUInt32 offset, epicsUInt32* value )
{
    //print_func;
    //volatile uint32_t* regAddr = reinterpret_cast<volatile uint32_t*>(regBaseAddress + getAxisOffset(axisNo) + offset);
    uint32_t regAddr = getAxisOffset( axisNo ) + offset;
#ifndef DBG
    //*value = *regAddr;
    *value = reg_p->reg_rd( getAxisOffset( axisNo ) + offset );

    cout << __func__
	 << ": read " << *value
	 << " from register " << regAddr
	 << " for axis " << axisNo
	 << endl;
#endif
}

uint32_t zynqMotorController::getAxisOffset(uint32_t axisNo)
{
    return axisNo * motorAxRegSize + motorRegOffset;
    //return axisNo * MOTOR_AX_REG_RANGE + MOTOR_BASE_ADDR;
}

//====================================================================

// These are the zynqMotorAxis methods

/** Creates a new zynqMotorAxis object.
  * \param[in] pC Pointer to the zynqMotorController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
zynqMotorAxis::zynqMotorAxis(zynqMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC),
      axisNo_(axisNo)
      //axisBaseAddr( regBaseAddress + motorRegOffset + axisNo * motorAxRegSize )
      //axisBaseAddr( ZYNQ_BASE_ADDR + MOTOR_BASE_ADDR + axisNo * MOTOR_AX_REG_RANGE )
{
    print_func;
    cout << "Creating axis " << axisNo_ << endl;
}
    

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void zynqMotorAxis::report(FILE *fp, int level)
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

asynStatus zynqMotorAxis::sendAccelAndVelocity( uint32_t acceleration, uint32_t velocity ) 
{
    cout << __func__ << ": axis " << axisNo_ << endl;

    //asynStatus status;
    //int ival;
    // static const char *functionName = "zynq::sendAccelAndVelocity";
  
    // Send the velocity
    //ival = NINT( fabs(115200./velocity) );
    //if (ival < 2) ival=2;
    //if (ival > 255) ival = 255;
    cout << "velocity is " << velocity << endl;;
    pC_->writeReg32( axisNo_, motorRegSpeed, velocity );
      
    // Send the acceleration
    // acceleration is in steps/sec/sec
    // MCB is programmed with Ramp Index (R) where:
    //   dval (steps/sec/sec) = 720,000/(256-R) */
    //   or R=256-(720,000/dval) */
    //ival = NINT(256-(720000./acceleration));
    //if (ival < 1) ival=1;
    //if (ival > 255) ival=255;
    //sprintf(pC_->outString_, "#%02dR=%d", axisNo_, ival);
    //status = pC_->writeReadController();
    cout << "acceleration is " << acceleration << endl;

    return asynSuccess;
}


asynStatus zynqMotorAxis::move( double position,
                                int    relative,
                                double minVelocity,
                                double maxVelocity,
                                double acceleration )
{
    cout << __func__
	 << ": axis " << axisNo_
	 << " move to " << position
	 << " (" << relative << ")"
	 << endl;

    int32_t moveDistance; 
    asynStatus status;
    minVelocityRaw  = minVelocity  * resolutionCntPerEGU;
    maxVelocityRaw  = maxVelocity  * resolutionCntPerEGU;
    accelerationRaw = acceleration * resolutionCntPerEGU;

    status = sendAccelAndVelocity( accelerationRaw, maxVelocityRaw );
    
    if ( relative ) // relative move
    {
        positionSP      += position;
        moveDistance     = static_cast<int>(position * resolutionCntPerEGU);
    }
    else
    {
        
        positionSP      = position;
        moveDistance    = static_cast<int>(position - positionSP) * resolutionCntPerEGU;
    }
    position       = (moveDistance<0) ? -1 : 1;
    positionSPRaw += moveDistance;
    
    cout << "Move " << moveDistance << " counts to " << positionSPRaw << endl;

    pC_->writeReg32( axisNo_, motorRegDistanceSP, abs(moveDistance) );
    pC_->writeReg32( axisNo_, motorRegDirection, (moveDistance>0)?0:1 );
    pC_->writeReg32( axisNo_, motorRegEnable,    1);

    cout << __func__
	 << ": motor " << axisNo_
	 << " commanded to move to destination." << endl;
    return status;
}


asynStatus zynqMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{

    cout << __func__ << endl;

    epicsStatus status;

    //asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    //  "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    //  __func__, minVelocity, maxVelocity, acceleration);
    
    uint32_t maxVelocityRaw  = maxVelocity * resolutionCntPerEGU;
    uint32_t accelerationRaw = acceleration * resolutionCntPerEGU;

    status = sendAccelAndVelocity( accelerationRaw, maxVelocityRaw );

    pC_->writeReg32( axisNo_, motorRegDirection,  (maxVelocity>0)?0:1 );
    pC_->writeReg32( axisNo_, motorRegDistanceSP, 0xffffffff ); // move long distance
    pC_->writeReg32( axisNo_, motorRegEnable,     1);

    return asynSuccess;
}

asynStatus zynqMotorAxis::stop(double acceleration )
{
    cout << __func__ << endl;

    pC_->writeReg32( axisNo_, motorRegEnable, MOTOR_CONTROL_MASK_STOP  |
                                              MOTOR_CONTROL_MASK_RESET |
                                              MOTOR_CONTROL_MASK_SLEEP  );
    return asynSuccess;
}

asynStatus zynqMotorAxis::setPosition(double position)
{
    print_func;

    positionSP       = position;
    positionSPRaw    = position * resolutionCntPerEGU;
    positionStartRaw = positionSPRaw;

    return asynSuccess;
}


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus zynqMotorAxis::poll(bool *moving)
{
  cout << __func__ << endl;

  //int done;
  //int driveOn;
  //int limit;
  //double position;
  //asynStatus comStatus;
  epicsUInt32 motorStatus;

  // Read the distance the motor has moved
  uint32_t moved;
  pC_->readReg32( axisNo_, motorRegDistanceRB, &moved );

  positionRBRaw += positionSPRaw + direction * moved; 
  positionRB     = positionRBRaw * resolutionCntPerEGU;

  // Read the moving status of this motor
  pC_->readReg32( axisNo_, motorRegStatus, &motorStatus );
  if ( motorStatus & MOTOR_STATUS_MASK_MOVING )
  {
      *moving = true;
      cout << __func__ << ": axis " << axisNo_ << " moving" << endl;
  }
  else
  {
      *moving = false;
      cout << __func__ << ": axis " << axisNo_ << " not moving" << endl;
      positionSPRaw = positionRBRaw;
  }

  // Read the limit status

  // Read the drive power on status

  return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg zynqMotorCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg zynqMotorCreateControllerArg1 = {"ZynqMotor port name", iocshArgString};
static const iocshArg zynqMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg zynqMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg zynqMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const zynqMotorCreateControllerArgs[] = {&zynqMotorCreateControllerArg0,
                                                                 &zynqMotorCreateControllerArg1,
                                                                 &zynqMotorCreateControllerArg2,
                                                                 &zynqMotorCreateControllerArg3,
                                                                 &zynqMotorCreateControllerArg4};

static const iocshFuncDef zynqMotorCreateControllerDef = { "zynqMotorCreateController",
                                                           5,
                                                           zynqMotorCreateControllerArgs};

static void zynqMotorCreateContollerCallFunc(const iocshArgBuf *args)
{
  zynqMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void zynqMotorRegister(void)
{
  iocshRegister(&zynqMotorCreateControllerDef, zynqMotorCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(zynqMotorRegister);
}

