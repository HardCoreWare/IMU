#ifndef _MPU_h_
#define _MPU_h_

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

//gyroscope variables
struct Gyro{
long cal[3];
double spin[3];
double angle[3];
}gyro;

//output data for getters
struct Output{	
double angle[3];
double radial[3];       // not used yet
}output;

long temp;
long loopTimer;
boolean set;

};

#endif