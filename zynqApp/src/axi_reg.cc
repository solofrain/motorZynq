/************************************************************
 *
 * Project: 3FI detector
 *
 * Module: FPGA driver
 *         Access FPGA registers via memmory map.
 *
 * Revision:
 *   . v0.1  6/7/2024
 *
 * Author: Ji Li <liji@bnl.gov>
 *
 ************************************************************/
#include <fcntl.h>
#include <cstdint>
#include <stdexcept>
#include <sys/mman.h>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "axi_reg.h"
#include "log.h"


using std::cin;
using std::cout;
using std::endl;

//=========================================
//=========================================
axi_reg::axi_reg(uint32_t axi_base_addr): axi_base_addr(axi_base_addr),
                                          reg(nullptr),
                                          reg_size(0x10000)
{
    int fd = open("/dev/mem",O_RDWR | O_SYNC);
    if (fd == -1)
    {
        throw std::runtime_error("Failed to open /dev/mem. Try root.");
    }

    try
    {
        //reg_size = getpagesize();
        reg = static_cast<uint32_t *>( mmap( nullptr, reg_size,
                                             PROT_READ | PROT_WRITE,
                                             MAP_SHARED,
                                             fd,
                                             axi_base_addr));
    }
    catch (const std::exception& e)
    {
        close(fd);
        throw std::runtime_error("Memory mapping failed: " + std::string(e.what()));
    }


    close(fd);
    
    if(reg == MAP_FAILED)
    {
        throw std::runtime_error("Memory mapping failed");
    }

    trace_reg( __func__,
               ": axi_reg object created at 0x",
               std::hex, static_cast<void*>(reg), std::dec
             );
}


//=========================================
//=========================================
axi_reg::~axi_reg()
{
    if (reg != nullptr)
    {
        munmap(reg, reg_size);
    }
    trace_reg( __func__, ": axi_reg object destructed." );
}


//=========================================
//=========================================
uint32_t axi_reg::reg_rd(size_t offset)
{
    uint32_t val;
    if (offset % sizeof(uint32_t) != 0)
    {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    reg_lock.lock();
    val = *registerPtr;
    reg_lock.unlock();


    trace_reg( __func__, ": read 0x",
               std::hex, val,
               " @ 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    //io_wait();
    return val;
}


//=========================================
//=========================================
void axi_reg::reg_wr(size_t offset, uint32_t value)
{
    if (offset % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    reg_lock.lock();
    *registerPtr = value;
    reg_lock.unlock();


    trace_reg( __func__,
               ": write 0x", std::hex, value,
               " to 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    reg_rd(offset);
}

/*
void axi_reg::io_wait()
{
    using namespace std::chrono_literals;

    std::this_thread::sleep_for(1ms);
}
*/
