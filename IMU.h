#ifndef _IMU_h_
#define _IMU_h_

#include <inttypes.h>
#include <Wire.h>

class MPU{
    
    public:
    
    MPU();
    
    void begin();
    void read();
    void calibrate(int n);
    double getAngle(int a);
    double getAcc();
    
    private:
    
    struct Acc{

        long value[3];
        long vector;
        long cal;
        double angle[3];

    }acc;

    struct Gyro{

        long cal[3];
        double spin[3];
        double angle[3];
    
    }gyro;

    struct Output{	

        double angle[3];
        double radial[3];

    }output;

long temp;
long loopTimer;
boolean set;

};

#endif