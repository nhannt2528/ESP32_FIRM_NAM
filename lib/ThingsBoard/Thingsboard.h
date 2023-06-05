#ifndef THINGSBOARD_H_
#define THINGSBOARD_H_
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <time.h>
#include "MLFs.h"
#define PATH_SCHEDULE "/schedule.json"
#if defined(ESP8266) || defined(ESP32)
#include <functional>
#define MQTT_CALLBACK_SIGNATURE std::function<void(char *, uint8_t *, unsigned int)> callback
#else
#define MQTT_CALLBACK_SIGNATURE void (*callback)(char *, uint8_t *, unsigned int)
#endif
#define DEBUG 1
#ifdef DEBUG
#define DB_PRINTLN(x) Serial.println(x)
#define DB_PRINT(x) Serial.print(x)
#else
#define DB_PRINTLN(x)
#define DB_PRINT(x)
#endif

#define MAX_ACTION 10
#define MAX_TIMER 10
struct Action
{
    String key;
    String value;
};
struct TSchedule
{
    bool enable;
    String time;
    String interval;
    int action_count;
    Action action[MAX_ACTION];
};
class Thingsboard
{
private:
    String _token;
    String _ID;
    String _mqttHost;
    String _deviceID;
    int _mqttPort;
    PubSubClient *_client;
    int _numOfSchedule = 0;
    TSchedule _mSchedule[MAX_TIMER];
    MLFs *FILE_FS ;
private:
    void reconnect();
    void jsonParseToStruct(String jsonString, TSchedule *m_schedule);
    void printlnStruct();
public:
Thingsboard();
~Thingsboard();
    void begin(PubSubClient *client, String mqttHost, int mqttPort, String ID, String mqtt_tokent, MQTT_CALLBACK_SIGNATURE);
    void loop();
    void sendTelemertry(String key, bool value);
    void sendTelemertry(String key, float value);
    void sendTelemertry(String key, int value);
    void sendTelemertry(String key, double value);
    void sendTelemertry(String key, String value);
    void sendTelemertry(String key, IPAddress value);
    String httpGETRequest();
    int getNumberOfSchedule();
    TSchedule getSchedule(int offset);
    bool connected();
};
#endif