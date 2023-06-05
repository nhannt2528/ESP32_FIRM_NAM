#ifndef _ML_TIME_H_
#define _ML_TIME_H_
#include <WiFi.h>
#include "time.h"
class MLtime
{
private:
struct tm timeinfo;
public:
    MLtime(/* args */);
    ~MLtime();
    void begin();
    void updateTime();
    String getStringTime();
    String getDayOfWeek();
};
extern MLtime ML_time;


#endif