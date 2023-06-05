#include "Arduino.h"
#include "ETH.h"
#include "ModbusRTU.h"
#include <ModbusIP_ESP8266.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "time.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "FS.h"
#include <LITTLEFS.h>
#include "NTPClient.h"
#include "WiFiUdp.h"
int RL[4] = {19, 18, 5, 4};
#define RL1 19
#define RL2 18
#define RL3 5
#define RL4 4
bool rl[4];
////////////////////////////////////////////////////
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "WmQ7HEAMmQnajiMpxDcU"
#define ID "23f575f0-e402-11ed-83d0-a9059b6f3a6c"
#define TIME_CHECK_WIFI 2000
#define TIME_SEND_SS 1000
////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient client(espClient);
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
const char *ssid = "DalatTech";
const char *pass = "mltech@2019";
//////////////////////////////
// bool rl1 = false;
// bool rl2 = false;
// bool rl3 = false;
// bool rl4 = false;
//////////////////////////
void updateState();
void wifiConnect();
void sendTelemertry(String key, bool value);
void mqttInit();
void on_message(const char *topic, byte *payload, unsigned int length);
void mqtt_loop();
void mqtt_reconnect();
void saveLocalStorage();
void localStorageExport();
//////////////////////
void timeInit();
void printLocalTime();
void NTPTimeUpdate();
void jsonObjectTimer(String jsonString);
void timerInit();
String httpGETRequest(const char *serverName);
/*Local Storage API*/
void fileInit();
void writeFile(fs::FS &fs, const char *path, const char *message);
String readFile(fs::FS &fs, const char *path);
void createDir(fs::FS &fs, const char *path);
//////////////////////////////
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const long gmtOffset_sec = +6 * 60 * 60;
const int daylightOffset_sec = 3600;
uint8_t number_of_timers = 0;

//////////////////////////////
// IPAddress local_IP(192, 168, 1, 48); // tu sam 2
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   // optional
// IPAddress secondaryDNS(8, 8, 4, 4); // optional

struct timer
{
    int id;
    bool enable;
    String time;
    String interval;
    String keyName[11];
    bool state[11];
};
timer TIMER[100];

void appInit()
{
    Serial.begin(9600);
    pinMode(RL1, OUTPUT);
    pinMode(RL2, OUTPUT);
    pinMode(RL3, OUTPUT);
    pinMode(RL4, OUTPUT);
    fileInit();
    timerInit();
    // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    // {
    //     Serial.println("STA Failed to configure");
    // }
    WiFi.begin(ssid, pass);
    mqttInit();
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wificonnect...");
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    localStorageExport();
    createDir(LITTLEFS, "/data");
    writeFile(LITTLEFS, "/data/config.json", "hello");
    Serial.println(readFile(LITTLEFS, "/data/config.json"));
}
void appRun()
{
    if (WL_CONNECTED)
    {
        mqtt_loop();
    }
    NTPTimeUpdate();
}

void wifiConnect()
{
    static unsigned long preTime = millis();
    if ((millis() - preTime > TIME_CHECK_WIFI) && (WiFi.status() != WL_CONNECTED))
    {
        Serial.println("WiFi connecting...");
        preTime = millis();
    }
}
void mqttInit()
{
    client.setServer(THINGS_BOARD_SERVER, 1883);
    client.setCallback(on_message);
}
void on_message(const char *topic, byte *payload, unsigned int length)
{
    StaticJsonDocument<200> doc;
    Serial.println("On message");
    char json[length + 1];
    strncpy(json, (char *)payload, length);
    json[length] = '\0';
    Serial.println("TOPIC: " + (String)topic);
    Serial.println("Message: " + (String)json);
    DeserializationError error = deserializeJson(doc, json);
    if (error)
    {
        Serial.println("deserializeJson failed");
        Serial.println(error.f_str());
        return;
    }
    if (strstr((char *)payload, "set_state") != NULL)
    {
        if (doc["params"].containsKey("power_1"))
        {
            rl[1] = doc["params"]["power_1"].as<bool>();
            sendTelemertry("power_1", rl[1]);
            digitalWrite(RL1, rl[1]);
        }
        if (doc["params"].containsKey("power_2"))
        {
            rl[2] = doc["params"]["power_2"].as<bool>();
            sendTelemertry("power_2", rl[2]);
            digitalWrite(RL2, rl[2]);
        }
        if (doc["params"].containsKey("power_3"))
        {
            rl[3] = doc["params"]["power_3"].as<bool>();
            sendTelemertry("power_3", rl[3]);
            digitalWrite(RL3, rl[3]);
        }
        if (doc["params"].containsKey("power_4"))
        {
            rl[4] = doc["params"]["power_4"].as<bool>();
            sendTelemertry("power_4", rl[4]);
            digitalWrite(RL4, rl[4]);
        }
        saveLocalStorage();
    }
    else if (strstr((char *)payload, "update_schedule") != NULL)
    {
        jsonObjectTimer(httpGETRequest(serverName.c_str()));
    }
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    Serial.println(responseTopic.c_str());
}
void mqtt_loop()
{
    if (!client.connected())
    {
        mqtt_reconnect();
    }

    client.loop();
}
void mqtt_reconnect()
{
    if ((!client.connected()))
    {
        Serial.println("Connecting to thingsboard...");
        if (client.connect(ID, TOKEN, NULL))
        {
            Serial.println("Connected");
            client.subscribe("v1/devices/me/rpc/request/+");
            // client.subscribe("v1/devices/me/attributes/request/+");
            client.subscribe("v1/devices/me/attributes");
            // mqtt_sendTelemertry()
            jsonObjectTimer(httpGETRequest(serverName.c_str()));
            sendTelemertry("power_1", rl[1]);
            sendTelemertry("power_2", rl[2]);
            sendTelemertry("power_3", rl[3]);
            sendTelemertry("power_4", rl[4]);
            //   mqtt_sendTelemertry();
        }
        else
        {
            Serial.println("Connect fail");
            Serial.println(client.state());

            //  delay(2000);
        }
    }
}
void sendTelemertry(String key, bool value)
{
    DynamicJsonDocument data(1024);
    value == true ? data[key] = true : data[key] = false;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
////////////////////////////////////////////////////////////////////
void saveLocalStorage()
{
    DynamicJsonDocument data(512);
    rl[1] == true ? data["power_1"] = true : data["power_1"] = false;
    digitalWrite(RL1, rl[1]);
    rl[2] == true ? data["power_2"] = true : data["power_2"] = false;
    digitalWrite(RL2, rl[2]);
    rl[3] == true ? data["power_3"] = true : data["power_3"] = false;
    digitalWrite(RL3, rl[3]);
    rl[4] == true ? data["power_4"] = true : data["power_4"] = false;
    digitalWrite(RL4, rl[4]);

    String objectString;
    serializeJson(data, objectString);
    writeFile(LITTLEFS, "/data/localStorage.json", objectString.c_str());
    Serial.println(objectString);
}
void localStorageExport()
{
    String data = readFile(LITTLEFS, "/data/localStorage.json");
    Serial.println("READ FILE: " + (String)data);
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, data.c_str());
    if (error)
    {
        Serial.println("deserializeJson failed");
        Serial.println(error.f_str());
        return;
    }
    else
    {
        rl[1] = doc["power_1"];
        rl[2] = doc["power_2"];
        rl[3] = doc["power_3"];
        rl[4] = doc["power_4"];
        digitalWrite(RL1, rl[1]);
        digitalWrite(RL2, rl[2]);
        digitalWrite(RL3, rl[3]);
        digitalWrite(RL4, rl[4]);

        Serial.println("Export data success!!");
    }
}
void fileInit()
{
    if (!LITTLEFS.begin(true))
    {
        Serial.println("LITTLEFS Mount Failed");
        return;
    }
}
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- file written");
    }
    else
    {
        Serial.println("- write failed");
    }
    file.close();
}
String readFile(fs::FS &fs, const char *path)
{
    String data = "";
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("- failed to open file for reading");
    }

    Serial.println("- read from file:");
    while (file.available())
    {
        data += (char)file.read();
    }
    file.close();
    return data;
}
void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}
//////////////////////////////////////////////////////////
String httpGETRequest(const char *serverName)
{
    String payload;
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    String serverPath = serverName + (String)TOKEN;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        payload = http.getString();
        Serial.println(payload);
        writeFile(LITTLEFS, "/timer.josn", payload.c_str());
    }
    else
    {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();

    return payload;
}
void timeInit()
{
    timeClient.begin();
    //  timeClient.setTimeOffset(+7*60*60);
}
void jsonObjectTimer(String jsonString)
{

    Serial.println(jsonString);
    StaticJsonDocument<1024> doc;
    deserializeJson(doc, jsonString);
    JsonArray array = doc["data"].as<JsonArray>();
    number_of_timers = array.size();
    for (int i = 0; i < number_of_timers; i++)
    {
        TIMER[i].id = doc["data"][i]["id"].as<int>();
        TIMER[i].enable = doc["data"][i]["enable"].as<bool>();
        TIMER[i].interval = doc["data"][i]["interval"].as<String>();
        TIMER[i].time = doc["data"][i]["time"].as<String>();
        TIMER[i].keyName[0] = "";
        TIMER[i].keyName[1] = "";
        for (int k = 0; k < 7; k++)
        {
            String tempKeyName = "power_" + (String)(k + 1);
            if (doc["data"][i]["action"].containsKey(tempKeyName))
            {
                String a = doc["data"][i]["action"][tempKeyName];
                TIMER[i].keyName[k] = tempKeyName;
                a == "true" ? TIMER[i].state[k] = true : TIMER[i].state[k] = false;
            }
        }
    }
}
void timerInit()
{
    Serial.println("Read data timer");
    String data_timer = readFile(LITTLEFS, "/timer.json");
    jsonObjectTimer(data_timer.c_str());
    for (int i = 0; i < number_of_timers; i++)
    {
        Serial.println(TIMER[i].id);
    }
}
void printLocalTime()
{
    timeClient.update();
    time_t epochTime = timeClient.getEpochTime();
    struct tm *timeInfo = gmtime((time_t *)&epochTime);
    int monthDay = timeInfo->tm_mday;
    Serial.print("Month day: ");
    Serial.println(monthDay);

    int currentMonth = timeInfo->tm_mon + 1;
    Serial.print("Month: ");
    Serial.println(currentMonth);

    String currentMonthName = months[currentMonth - 1];
    Serial.print("Month name: ");
    Serial.println(currentMonthName);

    int currentYear = timeInfo->tm_year + 1900;
    Serial.print("Year: ");
    Serial.println(currentYear);

    // Print complete date:
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
    Serial.print("Current date: ");
    Serial.println(currentDate);

    Serial.println("");
}
void NTPTimeUpdate()
{
    static int nowMM = 100;
    timeClient.update();
    time_t epochTime = timeClient.getEpochTime();
    struct tm *timeinfo = gmtime((time_t *)&epochTime);
    String hour;
    timeinfo->tm_hour > 9 ? hour = (String)timeinfo->tm_hour : hour = "0" + (String)timeinfo->tm_hour;
    String _time;
    timeinfo->tm_min > 9 ? _time = hour + ":" + (String)timeinfo->tm_min : _time = hour + ":0" + (String)timeinfo->tm_min;
    String _dayOfWeek = (String)timeinfo->tm_wday;
    //  Serial.print("time: ");
    //     Serial.println(array);

    for (int i = 0; i < number_of_timers; i++)
    {

        if ((TIMER[i].time == _time) && (strstr(TIMER[i].interval.c_str(), _dayOfWeek.c_str())))
        {
            if (nowMM != timeinfo->tm_min)
            {
                nowMM = timeinfo->tm_min;

                for (int k = 0; k < 7; k++)
                {
                    String keyName = "power_" + (String)(k + 1);
                    if (TIMER[i].keyName[k] == keyName)
                    {
                        // digitalWrite(coils_[k], TIMER[i].state[k]);
                        // rl[k] = TIMER[i].state[k];
                        // digitalWrite(RL[k], rl[k]);
                        rl[k] = TIMER[i].state[k];
                        Serial.println("STATE RL: " + (String)k + "state: " + (String)TIMER[i].state[0]);
                        sendTelemertry("power_1", rl[1]);
                        sendTelemertry("power_2", rl[2]);
                        sendTelemertry("power_3", rl[3]);
                        sendTelemertry("power_4", rl[4]);
                        digitalWrite(RL1, rl[1]);
                        digitalWrite(RL2, rl[2]);
                        digitalWrite(RL3, rl[3]);
                        digitalWrite(RL4, rl[4]);

                    }
                }
            }
        }
        else
        {
        }
    }
}
