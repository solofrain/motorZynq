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


using std::cin;
using std::cout;
using std::endl;

//=========================================
//=========================================
axi_reg::axi_reg(uint32_t axi_base_addr): axi_base_addr(axi_base_addr),
                                          reg(nullptr),
                                          reg_size(0x10000)
{
    cout << __func__ << ": constructing axi_reg object..." << endl;

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

    cout << __func__ << ": axi_reg object created at 0x" << static_cast<void*>(reg) << endl;
}


//=========================================
//=========================================
axi_reg::~axi_reg()
{
    if (reg != nullptr)
	{
	    munmap(reg, reg_size);
    }
    cout << __func__ << ": axi_reg object destructed." << endl;
}


//=========================================
//=========================================
uint32_t axi_reg::reg_rd(size_t offset)
{
    cout << __func__ << std::hex
         << ": input offset = 0x" << offset
         << endl;

    uint32_t val;
    if (offset % sizeof(uint32_t) != 0)
    {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    reg_lock.lock();
    val = *registerPtr;
    reg_lock.unlock();


    cout << __func__ << std::hex
         << ": read 0x" << val
	 << " @ 0x" << offset
	 << " (0x" << static_cast<void*>(const_cast<uint32_t*>(registerPtr)) << ")"
	 << endl;

    //io_wait();
    return val;
}


//=========================================
//=========================================
void axi_reg::reg_wr(size_t offset, uint32_t value)
{
    cout << __func__ << std::hex
         << ": input offset = 0x" << offset
         << ", wr_data = 0x" << value
         << endl;

    if (offset % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    reg_lock.lock();
    *registerPtr = value;
    reg_lock.unlock();


    cout << __func__ << std::hex
         << ": write 0x" << value
	 << " to 0x" << offset
	 << " (0x" << static_cast<void*>(const_cast<uint32_t*>(registerPtr)) << ")"
         << endl;

    reg_rd(offset);
}

/*
void axi_reg::io_wait()
{
    using namespace std::chrono_literals;

    std::this_thread::sleep_for(1ms);
}
*/
