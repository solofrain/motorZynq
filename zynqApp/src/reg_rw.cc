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
       printf("Usage: %s [R W] addr value\n",argv[0]);
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
    cout << "Register @" << addr << " (original) / ";

    cout << addr << "(converted)";

	uint32_t val;
    
	if (strcmp(argv[1],"W") == 0)
	{
		cout << "written." << endl;
        val = strtoul(argv[3],NULL,0);
		reg_p->reg_wr( addr, val );

        cout << "New value = " << val << endl;
    }
    else if (strcmp(argv[1], "R") == 0)
	{
		cout << "read." << endl;
		val = reg_p->reg_rd( addr );

        cout << " has value " << val << endl;
    }
}
