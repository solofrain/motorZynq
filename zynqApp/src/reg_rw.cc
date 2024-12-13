#include <iostream>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include "axi_reg.h"

using namespace std;

class Reg
{

private:
    std::unique_ptr<axi_reg>  reg_p;
        const uintptr_t regBaseAddress = 0x43C00000;
        uint32_t getAxisOffset(uint32_t axis);
};


int main( int argc, char *argv[] )
{
    if (argc < 3)
    {
       cout << "Usage: " << argv[0] << " [R W] addr value" << endl;
       exit(1);
    }

    std::unique_ptr<axi_reg>  reg_p;
    const uintptr_t regBaseAddress = 0x43C00000;

    try
    {
        reg_p = std::make_unique<axi_reg>(regBaseAddress);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Unable to create axi_reg object. Try root.");
    }

    uint32_t addr = strtol(argv[2],NULL,0); 
    uint32_t val;
    
    if (strcmp(argv[1],"W") == 0)
    {
        cout << "Write to reigster 0x" << addr << endl;
        val = strtoul(argv[3],NULL,0);
        reg_p->reg_wr( addr, val );
    }
    else if (strcmp(argv[1], "R") == 0)
    {
        cout << "Read from register 0x" << addr << endl;
        val = reg_p->reg_rd( addr );
    }
}
