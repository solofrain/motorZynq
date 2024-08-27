#ifndef _ZYNQMOTORDRIVER_H_
#define _ZYNQMOTORDRIVER_H_


#include "asynMotorController.h"
#include "asynMotorAxis.h"
//#include "axi_reg.h"

// Register definition
#define ZYNQ_BASE_ADDR     0x043C0000

#define MOTOR_BASE_ADDR     0x50
#define MOTOR_AX_REG_RANGE  0x20

#define MOTOR_CONTROL_MASK_ENABLE  0x01
#define MOTOR_CONTROL_MASK_SLEEP   0x02
#define MOTOR_CONTROL_MASK_RESET   0x04

#define MOTOR_STATUS_MASK_MOVING   0x01

#define NUM_ZYNQ_PARAMS 0

#define DBG

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

private:
    zynqMotorController *pC_;   // Pointer to the asynMotorController to which this axis belongs.
    int axisNo_;

#ifndef DBG
    std::unique_ptr<axi_reg>  reg_p;
#endif

    uintptr_t axisBaseAddr;

    static const uintptr_t motorBaseAddr      = 0x50;
    static const uintptr_t motorAxisRegSize   = 0x20;

    static const uintptr_t motorRegControl    = 0x00;
    static const uintptr_t motorRegMicrostep  = 0x04;
    static const uintptr_t motorRegDirection  = 0x08;
    static const uintptr_t motorRegEnable     = 0x0C; // FPGA clears move to indicate end of moving?
    static const uintptr_t motorRegSpeed      = 0x10;
    static const uintptr_t motorRegDistanceSP = 0x14;
    static const uintptr_t motorRegDistanceRB = 0x18;
    static const uintptr_t motorRegStatus     = 0x1C;

    int      direction;               // 1: positive; -1: negative
    double   positionSP;              // absolute position set point
    double   positionRB;              // absolute position readback value
    uint32_t positionStartRaw;        // position start point in counts
    uint32_t positionSPRaw;           // absolute position set point in counts
    uint32_t positionRBRaw;           // absolute position readback value in counts
    uint32_t minVelocityRaw;          // minimum velocity
    uint32_t maxVelocityRaw;          // maximum velocity
    uint32_t accelerationRaw;         // acceleration in count
    uint32_t speedAdjStepRaw;         // speed increase/decrease step
    uint32_t speedAadjIntervalRaw;    // speed increase/decrease interval

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
    uintptr_t getAxisOffset(uint32_t axis);

    uintptr_t baseAddress = 0x43C00000;

friend class zynqMotorAxis;

};


#endif


