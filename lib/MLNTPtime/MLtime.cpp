#include "MLtime.h"
MLtime ML_time;
MLtime::MLtime(/* args */)
{
}

MLtime::~MLtime()
{
}

void MLtime::begin()
{
    configTime(0, 60 * 60 * 7, "pool.ntp.org");
}

void MLtime::updateTime()
{
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
}

String MLtime::getStringTime()
{
    String hour;
    timeinfo.tm_hour > 9 ? hour = (String)timeinfo.tm_hour : hour = "0" + (String)timeinfo.tm_hour;
    String _time;
    timeinfo.tm_min > 9 ? _time = hour + ":" + (String)timeinfo.tm_min : _time = hour + ":0" + (String)timeinfo.tm_min;
    return _time;
}

String MLtime::getDayOfWeek()
{
    return (String)timeinfo.tm_wday;
}
