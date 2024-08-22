#ifndef _ZYNQDRIVER_H_
#define _ZYNQDRIVER_H_


#include "asynMotorController.h"
#include "asynMotorAxis.h"

// Register definition
#define ZYNQ_BASE_ADDR     0x043C0000

#define MOTOR_0_BASE_ADDR  0x50
#define MOTOR_1_BASE_ADDR  0x70

#define MOTOR_CONTROL      0x00 // bit 0: controls enable, bit 1: sleep, bit 2: reset
#define MOTOR_MICROSTEP    0x04 // m
#define MOTOR_DIRECTION    0x08 // d
#define MOTOR_ENABLE       0x0C // e
#define MOTOR_SPEED        0x10 // v
#define MOTOR_DISTANCE     0x14 // s
#define MOTOR_DISTANCE_RB  0x18
#define MOTOR_FAULT        0x1C



#define MAX_Zynq_AXES   2

#define NUM_Zynq_PARAMS 0

class epicsShareClass ZynqAxis : public asynMotorAxis
{
public:
    ZynqAxis(class ZynqController *pC, int axis);
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);

private:
    ZynqController *pC_;   // Pointer to the asynMotorController to which this axis belongs.

    std::unique_ptr<axi_reg>  reg_p;
    uint32_t base_addr_reg;
    uint32_t control_reg;
    uint32_t microstep_reg;
    uint32_t direction_reg;
    uint32_t enable_reg;
    uint32_t speed_reg;
    uint32_t distance_sp_reg;
    uint32_t distance_rb_reg;
    uint32_t status_reg;

    uint8_t  direction;
    uint32_t position;           // absolute direction in steps
    uint32_t max_speed;          // maximum speed
    uint32_t speed_adj_step;     // speed increase/decrease step
    uint32_t speed_adj_interval  // speed increase/decrease interval

    asynStatus sendAccelAndVelocity(double accel, double velocity);

friend class ZynqController;
}


class epicsShareClass ZynqController : public asynMotorController {
public:
    ZynqController( const char *portName,
                    const char *ZynqPortName,
                    int numAxes,
                    double movingPollPeriod,
                    double idlePollPeriod);

    asynStatus writeUint32Digital( asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask );

    void report(FILE *fp, int level);
    ZynqAxis* getAxis(asynUser *pasynUser);
    ZynqAxis* getAxis(int axisNo);

friend class ZynqAxis;

};


#endif

