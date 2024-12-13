#./reg_rw W 0x50 6       # sleepn=1, rstn=1, enbn=0
#./reg_rw W 0x54 0       # ustep mode, m2=m1=m0 = 0
#./reg_rw W 0x58 1       #direction
#./reg_rw W 0x60 42950   #step rate (1KHz)
#./reg_rw W 0x64 500000  # number of steps
#./reg_rw W 0x5C 1       # enable step (enables the step pulse)

./reg_rw R 0x1000        # sleepn=1, rstn=1, enbn=0
./reg_rw W 0x1000 6       # sleepn=1, rstn=1, enbn=0

./reg_rw R 0x1004        # ustep mode, m2=m1=m0 = 0
./reg_rw W 0x1004 0       # ustep mode, m2=m1=m0 = 0

./reg_rw R 0x1008        #direction
./reg_rw W 0x1008 1       #direction

#./reg_rw W 0x1010 0xA7C6  #step rate (1KHz)

./reg_rw R 0x1014     # number of steps
./reg_rw W 0x1014 0xC8    # number of steps

./reg_rw R 0x100C        # enable step (enables the step pulse)
./reg_rw W 0x100C 1       # enable step (enables the step pulse)
