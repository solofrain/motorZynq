/*
FILENAME... zynqMotorDriver.cpp
USAGE...    Motor driver support for Zynq motor controller.

Ji Li
08/14/2024

*/
#include <iostream>
#include <iomanip>
#include <memory>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include <iocsh.h>
#include <epicsThread.h>

//#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "zynqMotorDriver.h"
#include "axi_reg.h"
#include "log.h"

using std::cout;
using std::endl;

//#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)
#define NINT(f) std::round(f)

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

    cout << "The controller has " << numAxes
         << " axes" << endl;
      
    for (int axis=0; axis<numAxes; axis++)
    {
        cout << "axis = " << axis << endl;
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
    return static_cast<zynqMotorAxis*>(asynMotorController::getAxis(axisNo));
}


void zynqMotorController::writeReg32( int axisNo, uint32_t offset, uint32_t value )
{
    print_func;
    uint32_t regAddr = getAxisOffset( axisNo ) + offset;
    //cout << __func__
    //     << ": write " << value
    //     << " to register " << regAddr 
    //     << " for axis " << axisNo
    //     << endl;
    reg_p->reg_wr( getAxisOffset(axisNo) + offset, value );
}


//void zynqMotorController::readReg32( int axisNo, uint32_t offset, uint32_t value )
void zynqMotorController::readReg32( int axisNo, epicsUInt32 offset, epicsUInt32* value )
{
    uint32_t regAddr = getAxisOffset( axisNo ) + offset;
    *value = reg_p->reg_rd( getAxisOffset( axisNo ) + offset );

    //cout << __func__
    //     << ": read " << *value
    //     << " from register " << regAddr
    //     << " for axis " << axisNo
    //     << endl;
}

uint32_t zynqMotorController::getAxisOffset(uint32_t axisNo)
{
    return axisNo * motorAxRegSize + motorRegOffset;
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
{
    cout << "============================================" << endl;
    cout << __func__
         << "Creating axis " << axisNo_
         << endl;

    setMicrostep( 0 );
    setStepRate( 42950 );
    setResolution( 400 );
    
    // Enable the axis
    pC_->writeReg32( axisNo_, motorRegControl, 0x6 );

    cout << __func__
         << ": axis " << axisNo_
         << " initial position at " << positionSP
         << endl;
}
    
asynStatus zynqMotorAxis::setResolution( uint32_t resolution )
{
    resolutionCntPerEGU = resolution;
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

asynStatus zynqMotorAxis::setMicrostep( uint32_t microstep )
{
    cout << __func__
     << ": set microstep = " << microstep
     << endl;
    pC_->writeReg32( axisNo_, motorRegMicrostep, microstep );

    return asynSuccess;
}

asynStatus zynqMotorAxis::setStepRate( uint32_t stepRate )
{
    cout << __func__
     << ": set step rate = " << stepRate
     << endl;
    pC_->writeReg32( axisNo_, motorRegSpeed, stepRate );

    return asynSuccess;
}

asynStatus zynqMotorAxis::sendAccelAndVelocity( uint32_t acceleration, uint32_t velocity ) 
{
    cout << __func__ << ": axis " << axisNo_ << endl;

    cout << "velocity is " << velocity << endl;;
    pC_->writeReg32( axisNo_, motorRegSpeed, velocity );
      
    cout << "acceleration is " << acceleration << endl;

    return asynSuccess;
}


asynStatus zynqMotorAxis::move( double position,
                                int    relative,
                                double minVelocity,
                                double maxVelocity,
                                double acceleration )
{
    trace_move("============================================");
    trace_move( __func__,
                ": axis ", axisNo_,
                " move to position ", position,
                ( relative ? " (relative) " : " (absolute) " ),
                ", v_min = ", minVelocity,
                ", v_max = ", maxVelocity,
                ", acceleration = ", acceleration );

    int32_t moveDistance;
    asynStatus status;

    double velocity = 42.95 * maxVelocity;

    status = sendAccelAndVelocity( NINT(acceleration), NINT(velocity) );
    
    trace_move( "Current position = ", positionRB );
    if ( relative ) // relative move
    {
        positionSP    = positionRB + NINT(position);
        moveDistance  = NINT(position);
    }
    else
    {
        moveDistance = NINT(position - positionRB);
        positionSP   = NINT(position);
    }
    trace_move( "New position = ", positionSP );
    trace_move( "moveDistance = ", moveDistance );

    direction       = (moveDistance<0) ? -1 : 1;
    //moveDistance = moveDistance * direction;
    
    trace_move( "Move ",
                moveDistance,
                " counts to ",
                positionSP );

    pC_->writeReg32( axisNo_, motorRegDistanceSP, abs(moveDistance) );
    pC_->writeReg32( axisNo_, motorRegDirection, (moveDistance>0)?0:1 );
    pC_->writeReg32( axisNo_, motorRegEnable,    1);
    pC_->writeReg32( axisNo_, motorRegEnable,    0);

    trace_move( __func__,
                ": axis ", axisNo_,
                " commanded to move to destination." );
    return status;
}


asynStatus zynqMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{

    trace_move( __func__ );

    epicsStatus status;

    double velocity = maxVelocity * 42.95;
    status = sendAccelAndVelocity( NINT(acceleration), NINT(maxVelocity) );

    pC_->writeReg32( axisNo_, motorRegDirection,  (maxVelocity>0)?0:1 );
    pC_->writeReg32( axisNo_, motorRegDistanceSP, 0xffffffff ); // move long distance
    pC_->writeReg32( axisNo_, motorRegEnable,     1);

    return asynSuccess;
}

asynStatus zynqMotorAxis::stop(double acceleration )
{
    trace_stop( __func__,
                ": stopping axis ", axisNo_ );

    pC_->writeReg32( axisNo_, motorRegControl, MOTOR_CONTROL_MASK_STOP  |
                                               MOTOR_CONTROL_MASK_RESET |
                                               MOTOR_CONTROL_MASK_SLEEP  );
    return asynSuccess;
}

asynStatus zynqMotorAxis::setPosition(double position)
{
    print_func;

    positionSP = NINT( position );
    positionRB = positionSP;

    trace_set_pos( __func__,
                   ": set axis ", axisNo_,
                   " position to ", positionSP );

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
  //int done;
  //int driveOn;
  //int limit;
  //double position;
  //asynStatus comStatus;
  epicsUInt32 motorStatus;

  // Read the moving status of this motor
  uint32_t left;
  static uint32_t left_prev[2];
  static bool moving_prev[2];
  pC_->readReg32( axisNo_, motorRegStatus, &motorStatus );

  // Read the steps left to go
  pC_->readReg32( axisNo_, motorRegDistanceRB, &left );

  positionRB = positionSP - direction * left;
  setDoubleParam(pC_->motorPosition_, positionRB);

  if ( motorStatus & MOTOR_STATUS_MASK_MOVING )
  {
      *moving = true;
      //trace_poll( "axis ", axisNo_, " moving" );
  }
  else
  {
      *moving = false;
      //trace_poll( "axis", axisNo_, "not moving" );
  }
  setIntegerParam( pC_->motorStatusDone_, *moving ? 0 : 1 );
  setIntegerParam( pC_->motorStatusMoving_, *moving ? 1 : 0 );

  if( *moving || ((!*moving) && moving_prev[axisNo_] ) )
      trace_poll( "axis ", axisNo_, ": ",
                  "positionSP = ", positionSP, ", ",
                  //" dir = ",  std::setw(2), direction,
                  "moved ", left_prev[axisNo_]-left, " steps, ",
                  left, " steps left, ",
                  left_prev[axisNo_], " steps left previously, "
                  "positionRB = ", positionRB
                );

  moving_prev[axisNo_] = *moving;
  left_prev[axisNo_] = left;

  // Read the limit status

  // Read the drive power on status

  callParamCallbacks();

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

