#include "ML_EC.h"
ML_EC::ML_EC(/* args */)
{
    ec = new DFRobot_EC();
}
ML_EC::ML_EC(int pin)
{
    ec = new DFRobot_EC();
    sensor_pin=pin;
}



ML_EC::~ML_EC()
{
}

void ML_EC::begin(){
    ec->begin();
}
float ML_EC::readSensorMs(){
return voltage;
}
float ML_EC::readSennsorUs(){
return us;
}
void ML_EC::setPinSensor(int pin){
    sensor_pin=pin;
}
void ML_EC::loop(){
       static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(sensor_pin)/1024.0*510;   // read the voltage
      us= voltage/1000;

    }
    ec->calibration(voltage,temperature); 
}