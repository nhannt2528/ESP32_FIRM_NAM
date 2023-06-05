/*
 * file DFRobot_EC.ino
 * @ https://github.com/DFRobot/DFRobot_EC
 *
 * This is the sample code for Gravity: Analog Electrical Conductivity Sensor / Meter Kit V2 (K=1.0), SKU: DFR0300.
 * In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * You can send commands in the serial monitor to execute the calibration.
 * Serial Commands:
 *   enterec -> enter the calibration mode
 *   calec -> calibrate with the standard buffer solution, two buffer solutions(1413us/cm and 12.88ms/cm) will be automaticlly recognized
 *   exitec -> save the calibrated parameters and exit from calibration mode
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-03-21
 */

#include "DFRobot_EC.h"
#include <EEPROM.h>

#define EC_PIN 39

float voltage,ecValue,us,temperature = 25;
DFRobot_EC ec;

void appInit();
void appRun();

void appInit()
{
  Serial.begin(115200);  
  ec.begin();
}

void appRun()
{
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*510;   // read the voltage
      us= voltage/1000;
      Serial.print("temperature:");
      Serial.print(temperature,1);
      Serial.println("^C");
      Serial.print(analogRead(EC_PIN));
      Serial.println("......");
      Serial.print("EC:");
      Serial.print(voltage);
      Serial.println("us/cm");
      Serial.print("EC:");
      Serial.print(us,2);
      Serial.println("ms/cm");

    }
    ec.calibration(voltage,temperature);          // calibration process by Serail CMD
}

float readTemperature()
{
  //add your code here to get the temperature from your temperature sensor
}