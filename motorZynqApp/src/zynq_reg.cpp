/*
 * zynq_reg.cpp
 *
 * Memory-mapped register access for Zynq/Kria FPGA via /dev/mem.
 *
 * Based on Kria-Motor-Controller/src/sw/Reg.cpp
 */

#include "zynq_reg.h"

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>

#include <epicsStdio.h>

zynqReg::zynqReg(off_t baseAddr, size_t size)
    : baseAddr_(baseAddr)
    , size_(size)
    , mappedBase_(nullptr)
    , fd_(-1)
{
    fd_ = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_ < 0) {
        throw std::runtime_error("zynqReg: Failed to open /dev/mem. Run as root?");
    }

    void *mapped = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_, baseAddr_);
    if (mapped == MAP_FAILED) {
        close(fd_);
        throw std::runtime_error("zynqReg: mmap failed");
    }

    mappedBase_ = static_cast<volatile uint32_t *>(mapped);
}

zynqReg::~zynqReg()
{
    if (mappedBase_ != nullptr) {
        munmap((void *)mappedBase_, size_);
    }
    if (fd_ >= 0) {
        close(fd_);
    }
}

uint32_t zynqReg::read(off_t offset)
{
    mutex_.lock();
    uint32_t val = mappedBase_[offset / sizeof(uint32_t)];
    mutex_.unlock();
    return val;
}

void zynqReg::write(off_t offset, uint32_t value)
{
    mutex_.lock();
    mappedBase_[offset / sizeof(uint32_t)] = value;
    mutex_.unlock();
}

uint32_t zynqReg::getField(off_t offset, uint8_t bitPos, uint8_t width)
{
    uint32_t mask = (width >= 32) ? 0xFFFFFFFFU : ((1U << width) - 1);
    uint32_t val = read(offset);
    return (val >> bitPos) & mask;
}

void zynqReg::setField(off_t offset, uint8_t bitPos, uint8_t width, uint32_t value)
{
    uint32_t fieldMask = (width >= 32) ? 0xFFFFFFFFU : ((1U << width) - 1);
    uint32_t mask = fieldMask << bitPos;

    mutex_.lock();
    uint32_t regVal = mappedBase_[offset / sizeof(uint32_t)];
    regVal &= ~mask;
    regVal |= (value & fieldMask) << bitPos;
    mappedBase_[offset / sizeof(uint32_t)] = regVal;
    mutex_.unlock();
}
