/*
 * zynq_reg.h
 *
 * Memory-mapped register access for Zynq/Kria FPGA via /dev/mem.
 * Thread-safe using epicsMutex.
 *
 * Based on Kria-Motor-Controller/src/sw/Reg.cpp
 */

#ifndef ZYNQ_REG_H
#define ZYNQ_REG_H

#include <cstdint>
#include <cstddef>
#include <sys/types.h>
#include <epicsMutex.h>

class zynqReg {
public:
    zynqReg(off_t baseAddr, size_t size);
    ~zynqReg();

    zynqReg(const zynqReg&) = delete;
    zynqReg& operator=(const zynqReg&) = delete;

    /* Full 32-bit register read/write */
    uint32_t read(off_t offset);
    void write(off_t offset, uint32_t value);

    /* Bit-field read/write (read-modify-write for setField) */
    uint32_t getField(off_t offset, uint8_t bitPos, uint8_t width);
    void setField(off_t offset, uint8_t bitPos, uint8_t width, uint32_t value);

private:
    const off_t baseAddr_;
    const size_t size_;
    volatile uint32_t *mappedBase_;
    int fd_;
    epicsMutex mutex_;
};

#endif /* ZYNQ_REG_H */
