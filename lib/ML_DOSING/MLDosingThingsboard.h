#ifndef _ML_DOSING_THINGSBOARD_
#define _ML_DOSING_THINGSBOARD_
#include "Thingsboard.h"
#include "ArduinoJson.h"
#include <WiFiClient.h>
#include <PubSubClient.h>



#define DEBUG 1
#ifdef DEBUG
#define DB_PRINTLN(x) Serial.println(x)
#define DB_PRINT(x) Serial.print(x)
#else
#define DB_PRINTLN(x)
#define DB_PRINT(x)
#endif

class MLDosingThingsboard :public Thingsboard
{
private:
  
    String _tokent;
    String _deviceID;
public:
    friend class Thingsboard;
  static void on_message(const char *topic, byte *payload, unsigned int length);
    MLDosingThingsboard(String ID,String TOKENT);
    ~MLDosingThingsboard();
};

#endif