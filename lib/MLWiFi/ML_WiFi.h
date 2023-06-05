#ifndef _ML_WIFI_H_
#define _ML_WIFI_H_
#include <WiFi.h>
#include <Arduino.h>

class ML_Wifi : public WiFi
{
private:
    void connectWiFi();
    void disconnectWiFi();

public:
    ML_Wifi(/* args */);
    ~ML_Wifi();
    void init();
    void reconnect();
    void loop();
};
#endif