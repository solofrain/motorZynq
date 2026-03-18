/*
 * zynqMotorDriver.h
 *
 * EPICS asyn motor driver for KR260-based 4-axis stepper motor controller
 * using TI DRV8434A step-direction drivers.
 *
 * Features:
 *   - mmap-based register access via /dev/mem
 *   - Software S-curve motion profiling (raised-cosine velocity)
 *   - Software position tracking (step_rb is remaining steps)
 *   - Custom asyn parameters for DRV8434A configuration
 */

#ifndef ZYNQ_MOTOR_DRIVER_H
#define ZYNQ_MOTOR_DRIVER_H

#include <memory>
#include <cstdint>
#include <cmath>

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsMutex.h>

#include "zynq_reg.h"

/* ------------------------------------------------------------------ */
/* Hardware constants                                                  */
/* ------------------------------------------------------------------ */

static const uint64_t FPGA_CLOCK_HZ       = 100000000ULL;  /* 100 MHz */
static const uint32_t MAX_STEP_FREQ_HZ    = 250000;        /* DRV8434A max */
static const uint32_t COUNTER_BITS        = 32;
static const uint64_t COUNTER_MAX         = (1ULL << COUNTER_BITS);
static const uint32_t MAX_STEP_RATE       =
    static_cast<uint32_t>((static_cast<uint64_t>(MAX_STEP_FREQ_HZ) * COUNTER_MAX) / FPGA_CLOCK_HZ);

static const off_t  MOTOR_REG_OFFSET = 0x100;
static const off_t  MOTOR_REG_STRIDE = 0x30;
static const size_t REG_SIZE         = 0x1000;

/* Per-axis register offsets (relative to axis base) */
static const off_t REG_CFG       = 0x00;
static const off_t REG_STEP_RATE = 0x04;
static const off_t REG_CONTROL   = 0x08;
static const off_t REG_STEP_SP   = 0x0C;
static const off_t REG_STEP_RB   = 0x10;
static const off_t REG_STATUS    = 0x14;

/* Config register bit fields (cfg, offset 0x00)
 *   [0]    limit_en    - limit switch enable
 *   [1]    limit_pol   - limit switch polarity
 *   [15:8] ustep_mode  - microstep mode
 */
static const uint8_t CFG_LIMIT_EN_BIT   = 0;
static const uint8_t CFG_LIMIT_POL_BIT  = 1;
static const uint8_t CFG_USTEP_MODE_BIT = 8;
static const uint8_t CFG_USTEP_MODE_WID = 8;

/* Control register bit fields (control, offset 0x08)
 *   [0] en     - enable (active high; hardware inverts to enbn)
 *   [1] reset  - reset  (active high; hardware inverts to resetn)
 *   [2] sleep  - sleep  (active high; hardware inverts to sleepn)
 *   [3] dir    - direction
 *   [4] mstart - start (one-shot)
 *   [5] mstop  - stop  (one-shot)
 */
static const uint8_t CTRL_EN_BIT     = 0;
static const uint8_t CTRL_RESET_BIT  = 1;
static const uint8_t CTRL_SLEEP_BIT  = 2;
static const uint8_t CTRL_DIR_BIT    = 3;
static const uint8_t CTRL_MSTART_BIT = 4;
static const uint8_t CTRL_MSTOP_BIT  = 5;

/* Status register bit fields (status, offset 0x14)
 *   [0] fault   - inverted from faultn
 *   [1] moving
 *   [2] p_limit - positive limit
 *   [3] n_limit - negative limit
 */
static const uint8_t STAT_FAULT_BIT   = 0;
static const uint8_t STAT_MOVING_BIT  = 1;
static const uint8_t STAT_PLIMIT_BIT  = 2;
static const uint8_t STAT_NLIMIT_BIT  = 3;

/* S-curve profiler update interval */
static const double PROFILE_UPDATE_SEC = 0.0001;  /* 0.1 ms */

/* ------------------------------------------------------------------ */
/* Custom asyn parameter string names                                  */
/* ------------------------------------------------------------------ */

#define ZYNQ_DIR_POL_STRING      "ZYNQ_DIR_POL"
#define ZYNQ_LIMIT_EN_STRING     "ZYNQ_LIMIT_EN"
#define ZYNQ_LIMIT_POL_STRING    "ZYNQ_LIMIT_POL"
#define ZYNQ_USTEP_MODE_STRING   "ZYNQ_USTEP_MODE"
#define ZYNQ_SLEEP_STRING        "ZYNQ_SLEEP"
#define ZYNQ_RESET_STRING        "ZYNQ_RESET"
#define ZYNQ_STEP_RATE_STRING    "ZYNQ_STEP_RATE"

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */

class zynqMotorController;

/* ------------------------------------------------------------------ */
/* zynqMotorAxis                                                       */
/* ------------------------------------------------------------------ */

class zynqMotorAxis : public asynMotorAxis {
public:
    zynqMotorAxis(zynqMotorController *pC, int axisNo);

    /* Motor record interface */
    asynStatus move(double position, int relative,
                    double minVelocity, double maxVelocity,
                    double acceleration) override;
    asynStatus stop(double acceleration) override;
    asynStatus poll(bool *moving) override;
    asynStatus setPosition(double position) override;

    /* Called by the profiler thread to update velocity */
    void updateProfile();

private:
    zynqMotorController *pC_;

    /* Per-axis register base offset (MOTOR_REG_OFFSET + axisNo * MOTOR_REG_STRIDE) */
    off_t axisRegBase_;

    /* Move lifecycle FSM (owned exclusively by poll()) */
    enum MoveState { MOVE_IDLE, MOVE_ACTIVE, MOVE_DONE };
    MoveState moveState_;
    bool     moveRequested_;     /* set by move(), consumed by poll() */

    /* Software position tracking */
    double   softPosition_;      /* current absolute position in steps */
    double   moveStartPos_;      /* position at start of current move */
    uint32_t totalMoveSteps_;    /* total steps for current move */
    int      moveDirection_;     /* +1 or -1 */

    /* S-curve profile state */
    enum ProfilePhase { PHASE_IDLE, PHASE_ACCEL, PHASE_CRUISE, PHASE_DECEL, PHASE_DONE };
    ProfilePhase profilePhase_;
    bool     profileActive_;
    double   vBase_;             /* base velocity in steps/sec */
    double   vMax_;              /* peak velocity in steps/sec (may be reduced for short moves) */
    double   tAccel_;            /* acceleration time in seconds */
    uint32_t dAccelSteps_;       /* distance (steps) covered during accel phase */
    uint32_t decelStartRemaining_; /* step_rb threshold to begin deceleration */
    epicsTimeStamp moveStartTime_;
    epicsTimeStamp decelStartTime_;

    /* Velocity <-> step_rate conversion */
    uint32_t velocityToStepRate(double stepsPerSec);
    double   stepRateToVelocity(uint32_t stepRate);

    friend class zynqMotorController;
};

/* ------------------------------------------------------------------ */
/* zynqMotorController                                                 */
/* ------------------------------------------------------------------ */

class zynqMotorController : public asynMotorController {
public:
    zynqMotorController(const char *portName, int numAxes,
                        uint32_t baseAddr,
                        double movingPollPeriod, double idlePollPeriod);

    ~zynqMotorController() override;

    /* Override writeInt32 for custom parameters */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value) override;

    /* Register access */
    uint32_t readReg32(off_t offset);
    void writeReg32(off_t offset, uint32_t value);
    uint32_t readRegField(off_t offset, uint8_t bitPos, uint8_t width);
    void writeRegField(off_t offset, uint8_t bitPos, uint8_t width, uint32_t value);

    /* Axis accessor */
    zynqMotorAxis *getAxis(int axisNo);
    zynqMotorAxis *getAxis(asynUser *pasynUser);

    /* Profiler thread */
    void profilerThread();

protected:
    /* Custom asyn parameter indices */
    int zynqLimitEn_;
    int zynqLimitPol_;
    int zynqUstepMode_;
    int zynqSleep_;
    int zynqReset_;
    int zynqStepRate_;

#define FIRST_ZYNQ_PARAM zynqLimitEn_
#define LAST_ZYNQ_PARAM  zynqStepRate_
#define NUM_ZYNQ_PARAMS  (LAST_ZYNQ_PARAM - FIRST_ZYNQ_PARAM + 1)

private:
    std::unique_ptr<zynqReg> reg_;
    epicsThreadId profilerThreadId_;
    bool profilerRunning_;

    static void profilerThreadC(void *drvPvt);

    friend class zynqMotorAxis;
};

#endif /* ZYNQ_MOTOR_DRIVER_H */
