#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "IMU.h"
#include <Wire.h>


IMU::IMU(){
  
}


void IMU::begin(){

 //Activate the IMU-6050
 Wire.beginTransmission(0x68);                                        
 Wire.write(0x6B);                                                    
 Wire.write(0x00);                                                    
 Wire.endTransmission();                                              
 //Configure the accelerometer (+/-8g)
 Wire.beginTransmission(0x68);                                        
 Wire.write(0x1C);                                                    
 Wire.write(0x10);                                                    
 Wire.endTransmission();                                              
 //Configure the gyro (500dps full scale)
 Wire.beginTransmission(0x68);                                        
 Wire.write(0x1B);                                                    
 Wire.write(0x08);                                                    
 Wire.endTransmission();                                              

}
 
void IMU::read(){    

const int x=0,y=1,z=2;
const int R=0,P=1,Y=2;
   
Wire.beginTransmission(0x68);                                         
Wire.write(0x3B);                                                     
Wire.endTransmission();                                                
Wire.requestFrom(0x68,14);                                             
while(Wire.available() < 14);                                          
acc.value[x] = Wire.read()<<8|Wire.read();                             
acc.value[y] = Wire.read()<<8|Wire.read();
acc.value[z] = Wire.read()<<8|Wire.read();
temp = Wire.read()<<8|Wire.read();
gyro.spin[x] = Wire.read()<<8|Wire.read();
gyro.spin[y] = Wire.read()<<8|Wire.read();
gyro.spin[z] = Wire.read()<<8|Wire.read();
Wire.endTransmission();

gyro.spin[x] -= gyro.cal[x];                                              
gyro.spin[y] -= gyro.cal[y];                                           
gyro.spin[z] -= gyro.cal[z];
  
gyro.angle[P] += gyro.spin[x] * 0.0000611;                                         //Calculate the traveled pitch angle and add this to the angle_pitch variable
gyro.angle[R] += gyro.spin[y] * 0.0000611;                                         //Calculate the traveled roll angle and add this to the angle_roll variable
  
 //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
gyro.angle[P] += gyro.angle[R] * sin(gyro.spin[z] * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel
gyro.angle[R] -= gyro.angle[P] * sin(gyro.spin[z] * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel  
 //Accelerometer angle calculations
acc.vector = sqrt((acc.value[x]*acc.value[x])+(acc.value[y]*acc.value[y])+(acc.value[z]*acc.value[z]));   //Calculate the total accelerometer vector
 //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
acc.angle[P] = asin((double)acc.value[P]/acc.vector)*  57.296;                     //Calculate the pitch angle
acc.angle[R] = asin((double)acc.value[R]/acc.vector)* -57.296;                     //Calculate the roll angle
  
 //Place the IMU-6050 spirit level and note the values in the following two lines for calibration
acc.angle[P] -= 0.0;                                                               //Accelerometer calibration value for pitch
acc.angle[R] -= 0.0;                                                               //Accelerometer calibration value for roll

if(set){                                                                           //If the IMU is already started
gyro.angle[P] = gyro.angle[P] * 0.9 + acc.angle[P] * 0.1;                          //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
gyro.angle[R] = gyro.angle[R] * 0.9 + acc.angle[R] * 0.1;                          //Correct the drift of the gyro roll angle with the accelerometer roll angle
}
 
else{                                                                              //At first start
gyro.angle[P] = acc.angle[P];                                                      //Set the gyro pitch angle equal to the accelerometer pitch angle 
gyro.angle[R] = acc.angle[R];                                                      //Set the gyro roll angle equal to the accelerometer roll angle 
set = true;                                                                        //Set the IMU started flag
}
  
 //To dampen the pitch and roll angles a complementary filter is used
output.angle[P] = output.angle[P] * 0.9 + gyro.angle[P] * 0.1;                 //Take 90% of the output pitch value and add 10% of the raw pitch value
output.angle[R] = output.angle[R] * 0.9 + gyro.angle[R] * 0.1;                 //Take 90% of the output roll value and add 10% of the raw roll value
  
                                                                      
while(millis() - loopTimer < 4);                                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
loopTimer = millis();   

}

void IMU::calibrate(int n){

int x=0,y=1,z=2;

for (int i = 0; i < n ; i ++){                  

Wire.beginTransmission(0x68);                                    //Start communicating with the IMU-6050
Wire.write(0x3B);                                                //Send the requested starting register
Wire.endTransmission();                                          //End the transmission
Wire.requestFrom(0x68,14);                                       //Request 14 bytes from the IMU-6050
while(Wire.available() < 14);                                    //Wait until all the bytes are received
acc.value[x] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the acc_x variable
acc.value[y] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the acc_y variable
acc.value[z] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the acc_z variable
temp = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable
gyro.spin[x] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the gyro_x variable
gyro.spin[y] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the gyro_y variable
gyro.spin[z] = Wire.read()<<8|Wire.read();                       //Add the low and high byte to the gyro_z variable
Wire.endTransmission();
                                            
gyro.cal[x] += gyro.spin[x];                                             
gyro.cal[y] += gyro.spin[y];                                             
gyro.cal[z] += gyro.spin[z];

acc.vector = sqrt((acc.value[x]*acc.value[x])+(acc.value[y]*acc.value[y])+(acc.value[z]*acc.value[z]));           //Calculate the total accelerometer vector
acc.cal += acc.vector;
                                          
delay(5);   
                                                        
}
 
gyro.cal[x] /= n;                                                  
gyro.cal[y] /= n;                                                  
gyro.cal[z] /= n;   

acc.cal /= n;                                              

loopTimer = millis(); 
}

// get the output angle  0: roll 1: pitch
double IMU::getAngle(int a){

return output.angle[a];
  
}

// get the total aceleration in mG's  (gravity included)
double IMU::getAcc(){
	
return (double(acc.vector)/double(acc.cal))*1000;
	
}
