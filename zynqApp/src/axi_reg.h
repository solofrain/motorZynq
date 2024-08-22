#ifndef _AXI_REG_H_
#define _AXI_REG_H_

#include <cstdint>
#include <stdexcept>

#include "spinlock.h"

class axi_reg
{
private:
    uint32_t axi_base_addr;
    uint8_t  *reg;
    size_t   reg_size;
    spinlock reg_lock;

public:

    axi_reg(uint32_t axi_base_addr);

    ~axi_reg();

    uint32_t reg_rd(size_t offset);
	void reg_wr(size_t offset, uint32_t val);
    void io_wait();
};


#endif
