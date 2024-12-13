#./reg_rw W 0x70 6       # sleepn=1, rstn=1, enbn=0
#./reg_rw W 0x74 0       # ustep mode, m2=m1=m0 = 0
#./reg_rw W 0x78 1       #direction
#./reg_rw W 0x80 42950   #step rate (1KHz)
#./reg_rw W 0x84 500000  # number of steps
#./reg_rw W 0x7C 1       # enable step (enables the step pulse)

echo "----------------------------------"
./reg_rw R 0x1040        # sleepn=1, rstn=1, enbn=0
./reg_rw W 0x1040 6       # sleepn=1, rstn=1, enbn=0

echo "----------------------------------"
./reg_rw R 0x1044        # ustep mode, m2=m1=m0 = 0
./reg_rw W 0x1044 0       # ustep mode, m2=m1=m0 = 0

echo "----------------------------------"
./reg_rw R 0x1048        #direction
./reg_rw W 0x1048 1       #direction

echo "----------------------------------"
#./reg_rw W 0x1050   #step rate (1KHz)
#./reg_rw W 0x1050 0xA7C6  #step rate (1KHz)

./reg_rw R 0x1054     # number of steps 200
./reg_rw W 0x1054 0xC8    # number of steps 200

echo "----------------------------------"
./reg_rw R 0x104C        # enable step (enables the step pulse)
./reg_rw W 0x104C 1       # enable step (enables the step pulse)
echo "----------------------------------"
