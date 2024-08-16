#ifndef _DRCDRIVER_H_
#define _DRCDRIVER_H_


#include "asynMotorController.h"
#include "asynMotorAxis.h"

// Register definition
#define MOTOR_0_BASE_ADDR  0x50
#define MOTOR_1_BASE_ADDR  0x70
#define MOTOR_CONTROL      0x00
#define MOTOR_MICROSTEP    0x04
#define MOTOR_DIRECTION    0x08
#define MOTOR_ENABLE       0x0C
#define MOTOR_SPEED        0x10
#define MOTOR_DISTANCE     0x14
#define MOTOR_DISTANCE_RB  0x18
#define MOTOR_FAULT        0x1C



#define MAX_DRC_AXES   2

#define NUM_DRC_PARAMS 0

class epicsShareClass DRCAxis : public asynMotorAxis
{
public:
    DRCAxis(class DRCController *pC, int axis);
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);

private:
    DRCController *pC_;   // Pointer to the asynMotorController to which this axis belongs.

    asynStatus sendAccelAndVelocity(double accel, double velocity);

friend class DRCController;
}


class epicsShareClass DRCController : public asynMotorController {
public:
    DRCController( const char *portName,
                   const char *DRCPortName,
                   int numAxes,
                   double movingPollPeriod,
                   double idlePollPeriod);

  void report(FILE *fp, int level);
  DRCAxis* getAxis(asynUser *pasynUser);
  DRCAxis* getAxis(int axisNo);

friend class DRCAxis;

};


#endif

