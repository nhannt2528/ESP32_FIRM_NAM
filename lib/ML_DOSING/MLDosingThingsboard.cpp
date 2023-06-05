#include "MLDosingThingsboard.h"
void MLDosingThingsboard::on_message(const char *topic, byte *payload, unsigned int length){
    StaticJsonDocument<200> doc;
    DB_PRINTLN("On message");
    char json[length + 1];
    strncpy(json, (char *)payload, length);
    json[length] = '\0';
    DB_PRINTLN("TOPIC: " + (String)topic);
    DB_PRINTLN("Message: " + (String)json);
    DeserializationError error = deserializeJson(doc, json);
    if (error)
    {
        DB_PRINTLN("deserializeJson failed");
        DB_PRINTLN(error.f_str());
        return;
    }
    if (strstr((char *)payload, "set_state") != NULL)
    {
        if (strstr((char *)payload, "set_state") != NULL)
        {
        }
    }
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    DB_PRINTLN(responseTopic.c_str());
}


MLDosingThingsboard:: MLDosingThingsboard(String ID,String TOKENT)
{
_deviceID=ID;
_tokent=TOKENT;
}
MLDosingThingsboard::~MLDosingThingsboard()
{
}
