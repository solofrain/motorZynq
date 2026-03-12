/*
 * zynqMotorDriver.cpp
 *
 * EPICS asyn motor driver for KR260-based 4-axis stepper motor controller
 * using TI DRV8434A step-direction drivers.
 *
 * Key design:
 *   - step_rb is REMAINING steps (counts down to 0), not absolute position.
 *   - Absolute position tracked in software (softPosition_).
 *   - S-curve motion profiling via raised-cosine velocity updates every 5 ms.
 *   - step_rate can be updated on-the-fly during motion (DDS is combinational).
 */

#include "zynqMotorDriver.h"

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <stdexcept>

#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <asynOctetSyncIO.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *driverName = "zynqMotorDriver";

/* ================================================================== */
/*  zynqMotorAxis                                                      */
/* ================================================================== */

zynqMotorAxis::zynqMotorAxis(zynqMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo)
    , pC_(pC)
    , axisRegBase_(MOTOR_REG_OFFSET + axisNo * MOTOR_REG_STRIDE)
    , softPosition_(0.0)
    , moveStartPos_(0.0)
    , totalMoveSteps_(0)
    , moveDirection_(1)
    , profilePhase_(PHASE_IDLE)
    , profileActive_(false)
    , vBase_(0.0)
    , vMax_(0.0)
    , tAccel_(0.0)
    , dAccelSteps_(0)
    , decelStartRemaining_(0)
{
    memset(&moveStartTime_, 0, sizeof(moveStartTime_));
    memset(&decelStartTime_, 0, sizeof(decelStartTime_));

    /* Enable the motor driver by default */
    pC_->writeRegField(axisRegBase_ + REG_CONTROL, CTRL_EN_BIT, 1, 1);
}

/* ------------------------------------------------------------------ */

uint32_t zynqMotorAxis::velocityToStepRate(double stepsPerSec)
{
    //printf("stepsPerSec = %f\n", stepsPerSec);
    if (stepsPerSec <= 0.0)
        return 0;
    if (stepsPerSec > MAX_STEP_FREQ_HZ)
        stepsPerSec = MAX_STEP_FREQ_HZ;

    uint64_t rate = static_cast<uint64_t>(stepsPerSec * COUNTER_MAX / FPGA_CLOCK_HZ);
    if (rate > MAX_STEP_RATE)
        rate = MAX_STEP_RATE;
    return static_cast<uint32_t>(rate);
}

double zynqMotorAxis::stepRateToVelocity(uint32_t stepRate)
{
    return static_cast<double>(stepRate) * FPGA_CLOCK_HZ / COUNTER_MAX;
}

/* ------------------------------------------------------------------ */

asynStatus zynqMotorAxis::move(double position, int relative,
                               double minVelocity, double maxVelocity,
                               double acceleration)
{
    /* position and velocities are in steps (motor record converts EGU via MRES).
     * acceleration is in seconds (ACCL field = time from VBAS to VELO).
     */

    /* Compute target position and total steps */
    double targetPos;
    if (relative) {
        targetPos = softPosition_ + position;
    } else {
        targetPos = position;
    }

    double delta = targetPos - softPosition_;
    moveDirection_ = (delta >= 0.0) ? 1 : -1;
    totalMoveSteps_ = static_cast<uint32_t>(fabs(delta) + 0.5);

    if (totalMoveSteps_ == 0)
        return asynSuccess;

    // Set EN, clear SLEEP and RESET bits here to guarantee timing requirement
    pC_->writeReg32( axisRegBase_ + REG_CONTROL, 1U << CTRL_EN_BIT );

    /* Velocity parameters */
    vBase_ = fabs(minVelocity);
    vMax_  = fabs(maxVelocity);
    tAccel_ = (vMax_ - vBase_) / fabs(acceleration);

    if (vBase_ < 1.0) vBase_ = 1.0;        /* minimum 1 step/sec */
    if (vMax_ < vBase_) vMax_ = vBase_;
    if (tAccel_ < PROFILE_UPDATE_SEC) tAccel_ = PROFILE_UPDATE_SEC;

    /* Compute acceleration distance using mean velocity over accel time:
     *   d_accel = (v_base + v_max) / 2 * t_accel
     * For a raised-cosine profile, the actual integral is the same as
     * trapezoidal average.
     */
    double dAccel = (vBase_ + vMax_) / 2.0 * tAccel_;

    /* If move is too short for full accel + decel, reduce peak velocity */
    if (totalMoveSteps_ < 2.0 * dAccel) {
        /* Solve: totalSteps = 2 * (v_base + v_peak) / 2 * t_accel
         *        v_peak = totalSteps / t_accel - v_base
         * But also ensure v_peak >= v_base */
        double vPeak = static_cast<double>(totalMoveSteps_) / tAccel_ - vBase_;
        if (vPeak < vBase_) vPeak = vBase_;
        vMax_ = vPeak;
        dAccel = (vBase_ + vMax_) / 2.0 * tAccel_;
    }

    dAccelSteps_ = static_cast<uint32_t>(dAccel + 0.5);
    decelStartRemaining_ = dAccelSteps_;

    /* Record move start state */
    moveStartPos_ = softPosition_;
    epicsTimeGetCurrent(&moveStartTime_);

    /* Configure hardware registers */
    /* Set direction */
    uint32_t hwDir = (moveDirection_ > 0) ? 0 : 1;
    pC_->writeRegField(axisRegBase_ + REG_CONTROL, CTRL_DIR_BIT, 1, hwDir);

    /* Set step count */
    pC_->writeReg32(axisRegBase_ + REG_STEP_SP, totalMoveSteps_);

    /* Set initial velocity (base speed) */
    uint32_t initRate = velocityToStepRate(vBase_);
    pC_->writeReg32(axisRegBase_ + REG_STEP_RATE, initRate);

    /* Start the move: write en=1, mstart=1 to control register.
     * Read-modify-write to preserve other bits, then set mstart (one-shot). */
    uint32_t ctrl = pC_->readReg32(axisRegBase_ + REG_CONTROL);
    ctrl |= (1U << CTRL_EN_BIT);       /* ensure enabled */
    ctrl |= (1U << CTRL_MSTART_BIT);   /* one-shot start */
    ctrl &= ~(1U << CTRL_MSTOP_BIT);   /* clear stop bit */
    /* Direction already set above via setField, re-apply here */
    ctrl = (ctrl & ~(1U << CTRL_DIR_BIT)) | (hwDir << CTRL_DIR_BIT);
    pC_->writeReg32(axisRegBase_ + REG_CONTROL, ctrl);

    /* Activate profiler */
    profilePhase_ = PHASE_ACCEL;
    profileActive_ = true;

    return asynSuccess;
}

/* ------------------------------------------------------------------ */

asynStatus zynqMotorAxis::stop(double acceleration)
{
    /* Deactivate profiler */
    profileActive_ = false;
    profilePhase_ = PHASE_IDLE;

    /* Issue hardware stop (one-shot) */
    pC_->writeRegField(axisRegBase_ + REG_CONTROL, CTRL_MSTOP_BIT, 1, 1);

    /* Disable the motor */
    pC_->writeRegField(axisRegBase_ + REG_CONTROL, CTRL_EN_BIT, 1, 0);

    /* Update position based on steps completed so far */
    uint32_t remaining = pC_->readReg32(axisRegBase_ + REG_STEP_RB);
    uint32_t completed = (totalMoveSteps_ > remaining) ? (totalMoveSteps_ - remaining) : 0;
    softPosition_ = moveStartPos_ + moveDirection_ * static_cast<double>(completed);

    return asynSuccess;
}

/* ------------------------------------------------------------------ */

asynStatus zynqMotorAxis::poll(bool *moving)
{
    /* Read status register */
    uint32_t status = pC_->readReg32(axisRegBase_ + REG_STATUS);

    int fault   = (status >> STAT_FAULT_BIT)  & 1;
    int isMoving = (status >> STAT_MOVING_BIT) & 1;
    int pLimit  = (status >> STAT_PLIMIT_BIT) & 1;
    int nLimit  = (status >> STAT_NLIMIT_BIT) & 1;

    *moving = (isMoving != 0);

    /* Update position from step_rb (remaining steps) */
    if (*moving || profileActive_) {
        uint32_t remaining = pC_->readReg32(axisRegBase_ + REG_STEP_RB);
        uint32_t completed = (totalMoveSteps_ > remaining) ? (totalMoveSteps_ - remaining) : 0;
        softPosition_ = moveStartPos_ + moveDirection_ * static_cast<double>(completed);
    }

    /* If hardware says done but profiler was still active, finalize */
    if (!(*moving) && profileActive_) {
        profileActive_ = false;
        profilePhase_ = PHASE_IDLE;
	pC_->writeRegField( axisRegBase_ + REG_CONTROL, CTRL_EN_BIT, 1, 0 );
        softPosition_ = moveStartPos_ + moveDirection_ * static_cast<double>(totalMoveSteps_);
    }

    /* Read power-on (enable) state */
    int powerOn = pC_->readRegField(axisRegBase_ + REG_CONTROL, CTRL_EN_BIT, 1);

    /* Update motor record status */
    setIntegerParam(pC_->motorStatusMoving_, isMoving);
    setIntegerParam(pC_->motorStatusDone_, !isMoving);
    setIntegerParam(pC_->motorStatusProblem_, fault);
    setIntegerParam(pC_->motorStatusHighLimit_, pLimit);
    setIntegerParam(pC_->motorStatusLowLimit_, nLimit);
    setIntegerParam(pC_->motorStatusPowerOn_, powerOn);
    setIntegerParam(pC_->motorStatusDirection_, (moveDirection_ > 0) ? 1 : 0);

    setDoubleParam(pC_->motorPosition_, softPosition_);
    setDoubleParam(pC_->motorEncoderPosition_, softPosition_);

    /* Update custom readback parameters */
    uint32_t stepRate = pC_->readReg32(axisRegBase_ + REG_STEP_RATE);
    setIntegerParam(pC_->zynqStepRate_, static_cast<epicsInt32>(stepRate));

    uint32_t limitEn = pC_->readRegField(axisRegBase_ + REG_CFG, CFG_LIMIT_EN_BIT, 1);
    setIntegerParam(pC_->zynqLimitEn_, limitEn);

    uint32_t limitPol = pC_->readRegField(axisRegBase_ + REG_CFG, CFG_LIMIT_POL_BIT, 1);
    setIntegerParam(pC_->zynqLimitPol_, limitPol);

    uint32_t ustepMode = pC_->readRegField(axisRegBase_ + REG_CFG,
                                           CFG_USTEP_MODE_BIT, CFG_USTEP_MODE_WID);
    setIntegerParam(pC_->zynqUstepMode_, ustepMode);

    uint32_t sleepBit = pC_->readRegField(axisRegBase_ + REG_CONTROL, CTRL_SLEEP_BIT, 1);
    setIntegerParam(pC_->zynqSleep_, sleepBit);

    callParamCallbacks();
    return asynSuccess;
}

/* ------------------------------------------------------------------ */

asynStatus zynqMotorAxis::setPosition(double position)
{
    /* Hardware has no writable position register; track in software only */
    softPosition_ = position;
    return asynSuccess;
}

/* ------------------------------------------------------------------ */

void zynqMotorAxis::updateProfile()
{
    if (!profileActive_)
        return;

    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);

    uint32_t remaining = pC_->readReg32(axisRegBase_ + REG_STEP_RB);
    double v = vBase_;

    switch (profilePhase_) {

    case PHASE_ACCEL: {
        double elapsed = epicsTimeDiffInSeconds(&now, &moveStartTime_);

        if (elapsed >= tAccel_) {
            /* Acceleration complete */
            v = vMax_;
            profilePhase_ = PHASE_CRUISE;
        } else {
            /* Raised-cosine S-curve: v(t) = vBase + (vMax-vBase) * (1 - cos(pi*t/T)) / 2 */
            double frac = (1.0 - cos(M_PI * elapsed / tAccel_)) / 2.0;
            v = vBase_ + (vMax_ - vBase_) * frac;
        }

        /* Also check position-based decel trigger */
        if (remaining <= decelStartRemaining_ && remaining > 0) {
            profilePhase_ = PHASE_DECEL;
            epicsTimeGetCurrent(&decelStartTime_);
            /* Recalculate v for decel below */
            v = vMax_;
        }
        break;
    }

    case PHASE_CRUISE:
        v = vMax_;
	
        /* Start deceleration when remaining steps <= decel distance */
        if (remaining <= decelStartRemaining_ && remaining > 0) {
            profilePhase_ = PHASE_DECEL;
            epicsTimeGetCurrent(&decelStartTime_);
        }
        break;

    case PHASE_DECEL: {
        double elapsed = epicsTimeDiffInSeconds(&now, &decelStartTime_);

        if (elapsed >= tAccel_) {
            v = vBase_;
        } else {
            /* Mirror of accel: v(t) = vMax - (vMax-vBase) * (1 - cos(pi*t/T)) / 2 */
            double frac = (1.0 - cos(M_PI * elapsed / tAccel_)) / 2.0;
            v = vMax_ - (vMax_ - vBase_) * frac;
        }

        if (remaining == 0) {
            profilePhase_ = PHASE_DONE;
            profileActive_ = false;
        }
        break;
    }

    case PHASE_DONE:
    case PHASE_IDLE:
        profileActive_ = false;
        return;
    }

    /* Clamp velocity */
    if (v < vBase_) v = vBase_;
    if (v > static_cast<double>(MAX_STEP_FREQ_HZ)) v = MAX_STEP_FREQ_HZ;

    /* Convert to step_rate and write to hardware */
    uint32_t stepRate = velocityToStepRate(v);
    if (stepRate == 0) stepRate = 1;
    pC_->writeReg32(axisRegBase_ + REG_STEP_RATE, stepRate);
}

/* ================================================================== */
/*  zynqMotorController                                                */
/* ================================================================== */

zynqMotorController::zynqMotorController(const char *portName, int numAxes,
                                         uint32_t baseAddr,
                                         double movingPollPeriod,
                                         double idlePollPeriod)
    : asynMotorController(portName, numAxes,
                          NUM_ZYNQ_PARAMS,
                          0, /* No additional interfaces */
                          0, /* No additional callback interfaces */
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1, /* autoConnect */
                          0, 0) /* default priority and stack size */
    , profilerRunning_(true)
{
    /* Create custom parameters */
    createParam(ZYNQ_LIMIT_EN_STRING,   asynParamInt32, &zynqLimitEn_);
    createParam(ZYNQ_LIMIT_POL_STRING,  asynParamInt32, &zynqLimitPol_);
    createParam(ZYNQ_USTEP_MODE_STRING, asynParamInt32, &zynqUstepMode_);
    createParam(ZYNQ_SLEEP_STRING,      asynParamInt32, &zynqSleep_);
    createParam(ZYNQ_RESET_STRING,      asynParamInt32, &zynqReset_);
    createParam(ZYNQ_STEP_RATE_STRING,  asynParamInt32, &zynqStepRate_);

    /* Open mmap register access */
    reg_ = std::make_unique<zynqReg>(static_cast<off_t>(baseAddr), REG_SIZE);

    /* Create axes */
    for (int axis = 0; axis < numAxes; axis++) {
        new zynqMotorAxis(this, axis);
    }

    /* Start the S-curve profiler thread */
    profilerThreadId_ = epicsThreadCreate(
        "zynqProfiler",
        epicsThreadPriorityHigh,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        profilerThreadC,
        this);

    /* Start the poller */
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

zynqMotorController::~zynqMotorController()
{
    profilerRunning_ = false;
    /* Give profiler thread time to exit */
    epicsThreadSleep(2 * PROFILE_UPDATE_SEC);
}

/* ------------------------------------------------------------------ */

void zynqMotorController::profilerThreadC(void *drvPvt)
{
    static_cast<zynqMotorController *>(drvPvt)->profilerThread();
}

void zynqMotorController::profilerThread()
{
    while (profilerRunning_) {
        epicsThreadSleep(PROFILE_UPDATE_SEC);

        lock();
        for (int i = 0; i < numAxes_; i++) {
            zynqMotorAxis *pAxis = getAxis(i);
            if (pAxis) {
                pAxis->updateProfile();
            }
        }
        unlock();
    }
}

/* ------------------------------------------------------------------ */

uint32_t zynqMotorController::readReg32(off_t offset)
{
    return reg_->read(offset);
}

void zynqMotorController::writeReg32(off_t offset, uint32_t value)
{
    reg_->write(offset, value);
}

uint32_t zynqMotorController::readRegField(off_t offset, uint8_t bitPos, uint8_t width)
{
    return reg_->getField(offset, bitPos, width);
}

void zynqMotorController::writeRegField(off_t offset, uint8_t bitPos, uint8_t width, uint32_t value)
{
    reg_->setField(offset, bitPos, width, value);
}

/* ------------------------------------------------------------------ */

zynqMotorAxis *zynqMotorController::getAxis(int axisNo)
{
    return static_cast<zynqMotorAxis *>(asynMotorController::getAxis(axisNo));
}

zynqMotorAxis *zynqMotorController::getAxis(asynUser *pasynUser)
{
    return static_cast<zynqMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

/* ------------------------------------------------------------------ */

asynStatus zynqMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int axisNo = 0;
    getAddress(pasynUser, &axisNo);

    zynqMotorAxis *pAxis = getAxis(axisNo);
    if (!pAxis)
        return asynMotorController::writeInt32(pasynUser, value);

    off_t axBase = pAxis->axisRegBase_;

    if (function == zynqLimitEn_) {
        writeRegField(axBase + REG_CFG, CFG_LIMIT_EN_BIT, 1, value & 1);
    } else if (function == zynqLimitPol_) {
        writeRegField(axBase + REG_CFG, CFG_LIMIT_POL_BIT, 1, value & 1);
    } else if (function == zynqUstepMode_) {
        writeRegField(axBase + REG_CFG, CFG_USTEP_MODE_BIT, CFG_USTEP_MODE_WID,
                       static_cast<uint32_t>(value));
    } else if (function == zynqSleep_) {
        writeRegField(axBase + REG_CONTROL, CTRL_SLEEP_BIT, 1, value & 1);
    } else if (function == zynqReset_) {
        if (value) {
            writeRegField(axBase + REG_CONTROL, CTRL_RESET_BIT, 1, 1);
            /* Auto-clear reset after a brief hold */
            epicsThreadSleep(0.001);
            writeRegField(axBase + REG_CONTROL, CTRL_RESET_BIT, 1, 0);
        }
    } else {
        /* Delegate to base class for standard motor parameters */
        return asynMotorController::writeInt32(pasynUser, value);
    }

    /* Store the parameter value */
    setIntegerParam(axisNo, function, value);
    callParamCallbacks(axisNo);

    return asynSuccess;
}
