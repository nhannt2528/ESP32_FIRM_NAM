
#include "Thingsboard.h"
void Thingsboard::begin(PubSubClient *client, String mqttHost, int mqttPort, String ID, String mqtt_tokent, MQTT_CALLBACK_SIGNATURE)
{
    _mqttHost = mqttHost;
    _mqttPort = mqttPort;
    _deviceID = ID;
    _token = mqtt_tokent;
    _client = client;
    _client->setServer(_mqttHost.c_str(), _mqttPort);
    _client->setCallback(callback);
    FILE_FS->init();
    this->jsonParseToStruct(FILE_FS->readFile(SPIFFS, PATH_SCHEDULE), _mSchedule);
}
void Thingsboard::loop()
{

    static unsigned long timePrint = millis();
    if (millis() - timePrint > 1000)
    {
        timePrint = millis();
    }
    if (!_client->connected())
    {
        reconnect();
    }
    _client->loop();
}
void Thingsboard::reconnect()
{
    if ((!_client->connected()))
    {
        DB_PRINTLN("Connecting to thingsboard...");
        if (_client->connect(_deviceID.c_str(), _token.c_str(), NULL))
        {
            DB_PRINTLN("Connected");
            _client->subscribe("v1/devices/me/rpc/request/+");
            _client->subscribe("v1/devices/me/attributes");
        }
        else
        {
            DB_PRINTLN("Connect fail");
            DB_PRINTLN(_client->state());
        }
    }
}
void Thingsboard::jsonParseToStruct(String jsonString, TSchedule *m_schedule)
{
    StaticJsonDocument<1024> doc;
    deserializeJson(doc, jsonString);
    JsonArray array = doc["data"].as<JsonArray>();

    _numOfSchedule = array.size();

    for (int i = 0; i < _numOfSchedule; i++)
    {
        JsonObject actionObject = doc["data"][i]["action"].as<JsonObject>();
        int num_of_object = actionObject.size();
        m_schedule[i].action_count = num_of_object;
        m_schedule[i].enable = doc["data"][i]["enable"].as<bool>();
        m_schedule[i].interval = doc["data"][i]["interval"].as<String>();
        m_schedule[i].time = doc["data"][i]["time"].as<String>();
        int j = 0;
        for (JsonPair kv : actionObject)
        {
            m_schedule[i].action[j].key = kv.key().c_str();
            m_schedule[i].action[j].value = kv.value().as<String>();
            j++;
        }
    }
}
void Thingsboard::printlnStruct()
{
    Serial.println("enable: " + (String)_mSchedule[0].enable);
    Serial.println("interval: " + (String)_mSchedule[0].interval);
    Serial.println("key: " + (String)_mSchedule[0].action[0].key);
    Serial.println("vale: " + (String)_mSchedule[0].action[0].value);
}
Thingsboard::Thingsboard()
{
    FILE_FS = new MLFs();
}
Thingsboard::~Thingsboard()
{
}
void Thingsboard::sendTelemertry(String key, bool value)
{
    DynamicJsonDocument data(200);
    value == true ? data[key] = true : data[key] = false;
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
void Thingsboard::sendTelemertry(String key, float value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
void Thingsboard::sendTelemertry(String key, String value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
void Thingsboard::sendTelemertry(String key, int value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
void Thingsboard::sendTelemertry(String key, IPAddress value)
{
    DynamicJsonDocument data(200);
    data[key] = value.toString();
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
void Thingsboard::sendTelemertry(String key, double value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    _client->publish("v1/devices/me/telemetry", objectString.c_str());
    DB_PRINTLN(objectString);
}
String Thingsboard::httpGETRequest()
{
    String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
    String payload;
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    String serverPath = serverName + (String)_token;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
        DB_PRINTLN("HTTP Response code: ");
        DB_PRINTLN(httpResponseCode);
        payload = http.getString();
        DB_PRINTLN(payload);
        FILE_FS->writeFile(SPIFFS, PATH_SCHEDULE, payload.c_str());
        // writeFile(SPIFFS, PATH_SCHEDULE, payload.c_str());
    }
    else
    {
        Serial.print("Error code: ");
        DB_PRINTLN(httpResponseCode);
    }
    http.end();
    jsonParseToStruct(payload, _mSchedule);
    return payload;
}
int Thingsboard::getNumberOfSchedule()
{
    return _numOfSchedule;
}
TSchedule Thingsboard::getSchedule(int offset)
{
    return _mSchedule[offset];
}

bool Thingsboard::connected()
{
 return _client->connected();
}
