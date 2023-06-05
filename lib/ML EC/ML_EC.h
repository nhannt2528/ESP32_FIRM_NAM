#ifndef _ML_EC_H_
#define _ML_EC_H_
#include "Arduino.h"
#include "DFRobot_EC.h"
#include "EEPROM.h"
class ML_EC
{
private:
DFRobot_EC *ec;
    
    float voltage;
    float ecValue;
    float us;
    float temperature=25;
    int sensor_pin;
public:
float readSensorMs();
float readSennsorUs();
void setPinSensor(int pin);
    ML_EC();
     ML_EC(int pin);
    void begin();
    void loop();
    ~ML_EC();
};




#endif