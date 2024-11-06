#ifndef _ZYNQMOTORDRIVER_H_
#define _ZYNQMOTORDRIVER_H_


#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "axi_reg.h"


#define NUM_ZYNQ_PARAMS 0

//#define DBG

#ifdef DBG
#define print_func   cout<<__func__<<endl
#else
#define print_func
#endif

class epicsShareClass zynqMotorAxis : public asynMotorAxis
{
public:
    zynqMotorAxis(class zynqMotorController *pC, int axis);
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    //asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setPosition(double position);
    //asynStatus setClosedLoop(bool closedLoop);
    asynStatus setMicrostep( uint32_t microstep );
    asynStatus setStepRate( uint32_t stepRate );
    asynStatus setResolution( uint32_t resolution );

private:
    zynqMotorController *pC_;   // Pointer to the asynMotorController to which this axis belongs.
    int axisNo_;

    static const uintptr_t motorRegControl    = 0x00; // {28'x0, stop, rst_n, sleep_n, en_n}
    static const uintptr_t motorRegMicrostep  = 0x04; // microstep
    static const uintptr_t motorRegDirection  = 0x08;
    static const uintptr_t motorRegEnable     = 0x0C; // {31'x0, step_en}
    static const uintptr_t motorRegSpeed      = 0x10; // step rate. motor_speed = step_rate * 2^32/100e6
    static const uintptr_t motorRegDistanceSP = 0x14; // steps set point
    static const uintptr_t motorRegDistanceRB = 0x18; // steps readback
    static const uintptr_t motorRegStatus     = 0x1C; // status {30'd0, fault_n, move}

    // Register masks
    //   Motor control {28'x0, stop, rst_n, sleep_n, en_n}
    static const uint32_t MOTOR_CONTROL_MASK_ENABLE = 0x01;
    static const uint32_t MOTOR_CONTROL_MASK_SLEEP  = 0x02;
    static const uint32_t MOTOR_CONTROL_MASK_RESET  = 0x04;
    static const uint32_t MOTOR_CONTROL_MASK_STOP   = 0X08;

    //   Motor status {30'd0, fault_n, move}
    static const uint32_t MOTOR_STATUS_MASK_MOVING  = 0x01;
    static const uint32_t MOTOR_STATUS_MASK_FAULT   = 0x02;

    int      direction;               // 1: positive; -1: negative
    int   positionSP;              // absolute position set point
    int   positionRB;              // absolute position readback value
    //uint32_t positionStartRaw;        // position start point in counts
    //uint32_t positionSPRaw;           // absolute position set point in counts
    //uint32_t positionRBRaw;           // absolute position readback value in counts
    //uint32_t minVelocityRaw;          // minimum velocity
    //uint32_t maxVelocityRaw;          // maximum velocity
    //uint32_t accelerationRaw;         // acceleration in count
    //uint32_t speedAdjStepRaw;         // speed increase/decrease step
    //uint32_t speedAadjIntervalRaw;    // speed increase/decrease interval

    uint32_t resolutionCntPerEGU;     // resolution in cnt/EGU

    asynStatus sendAccelAndVelocity(uint32_t accel, uint32_t velocity);

friend class zynqMotorController;
};


class epicsShareClass zynqMotorController : public asynMotorController {
public:
    zynqMotorController( const char *portName,
                         const char *ZynqPortName,
                         int         numAxes,
                         double      movingPollPeriod,
                         double      idlePollPeriod);

    ~zynqMotorController();

    //asynStatus writeUInt32Digital( asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask );
    //asynStatus writeUInt32( asynUser* pasynUser, epicsUInt32 value );
    //asynStatus readUInt32( asynUser* pasynUser, epicsUInt32* value );
    void writeReg32( int axisNo, epicsUInt32 offset, epicsUInt32 value );
    void readReg32( int axisNo, epicsUInt32 offset, epicsUInt32* value );

    void report(FILE *fp, int level);
    zynqMotorAxis* getAxis(asynUser *pasynUser);
    zynqMotorAxis* getAxis(int axisNo);

private:
    uint32_t getAxisOffset(uint32_t axis);

    const uintptr_t regBaseAddress = 0x43C00000;
    const uint32_t  motorRegOffset = 0x1000;
    const uint32_t  motorAxRegSize = 0x40;
    std::unique_ptr<axi_reg>  reg_p;


friend class zynqMotorAxis;

};


#endif


