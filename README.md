# 6 deg of freedom IMU I2C class

## Methods

### calibrate(int n)

 n = number of iterations (int)   100 or more is recommended

void



### read()

read all data

void



### getAngle(int axis)

returns the angle of any axis as a double

axis (int) 
           
           0 x

           1 y
           
           2 z 



### getAcc(  )

returns the total axis vector in mG's (gravity included)
