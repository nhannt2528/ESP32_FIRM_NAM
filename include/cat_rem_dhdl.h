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
#include "MLtime.h"
#include "MLFs.h"
#include "SPIFFS.h"

bool state_rem_1 = false;
typedef enum
{
    IDLE_STATE,
    CLOSE_REM_1,
    OPEN_REM_1,
} state;
state LOGIC_STATE;

// int RL[6] = {2, 4, 16, 5, 32, 13};
#define RL1 2
#define RL2 4
#define RL3 16
#define RL4 5

bool mode_rem = false;
bool mode_tuoi = false;
bool rl[6];
////////////////////////////////////////////////////
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "QFhCS8a76zYjIJVkoXuU"
#define ID "fe5a8e10-cc79-11ed-86eb-ad5a639611e9"
#define TIME_CHECK_WIFI 2000
#define TIME_SEND_SS 1000
////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient client(espClient);
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
const char *ssid = "nha_kinh_so_04";
const char *pass = "H0983344442";

//////////////////////////
void updateState();
void wifiConnect();
void sendTelemertry(String key, bool value);
void sendTelemertry(String key, String value);
void sendTelemertry(String key, float value);
void sendTelemertry(String key, int value);
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
void open_rem_1();
void close_rem_1();
void logic_sensor_lux();
void logic_sensor_temp();
//////////////////////////////
void updateSchedule();
#define MAX_ACTION 50
#define MAX_TIMER 10
int _numOfSchedule = 0;
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
TSchedule _mSchedule[MAX_TIMER];
void jsonParseToStruct(String jsonString, TSchedule *m_schedule);
/////////////////////////////////////////////////////
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const long gmtOffset_sec = +6 * 60 * 60;
const int daylightOffset_sec = 3600;
uint8_t number_of_timers = 0;

////////////////////////////
IPAddress local_IP(192, 168, 1, 48); // tu sam 2
IPAddress gateway(192, 168, 1, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

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

struct set
{
    int light_Low = 0;
    int light_High = 0;
    int temp_Low = 0;
    int temp_High = 0;
    String device;
    bool state = false;
} set;

ModbusIP mbIP;
float lux;
float temp;
uint32_t floatMidLitteEndianCDAB(uint16_t AB, uint16_t CD);
float uint32ToFloat(uint32_t x);
void ModbusIpInit();
void ModbusIpLoop();

void appInit()
{
    Serial.begin(115200);
    pinMode(RL1, OUTPUT);
    pinMode(RL2, OUTPUT);
    pinMode(RL3, OUTPUT);
    pinMode(RL4, OUTPUT);
    fileInit();
    timerInit();
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }
    WiFi.begin(ssid, pass);
    mqttInit();
    ModbusIpInit();
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
void ModbusIpInit()
{
    mbIP.server();
    mbIP.addHreg(0, 0, 50);
}
void ModbusIpLoop()
{
    mbIP.task();
}
uint32_t floatMidLitteEndianCDAB(uint16_t AB, uint16_t CD) // cái này là ghép 2 thanh ghi 16 thành 32
{
    uint32_t CDAB = AB | (CD << 16);
    return CDAB;
}

float uint32ToFloat(uint32_t x) // hàm này là chuyển uint32 ssang float (unti32 là 8byte)
{
    float y;
    memcpy((uint8_t *)&y, (uint8_t *)&x, 4);
    return y;
}

void appRun()
{
    temp = (float)mbIP.Hreg(0) / 100;
    uint32_t lux_32t = floatMidLitteEndianCDAB((uint16_t)mbIP.Hreg(2), (uint16_t)mbIP.Hreg(3));
    lux = uint32ToFloat(lux_32t);
    // Serial.println("LUX: " + (String)lux);
    // Serial.println(sensor.temp);
    // Serial.println(sensor.humi);
    Serial.println(mbIP.Hreg(2), HEX);
    Serial.println(mbIP.Hreg(3), HEX);
    Serial.println("TEMP: " + (String)temp);
    Serial.println("LUX: " + (String)lux);
    delay(1000);
    if ((mode_rem == true))
    {
        logic_sensor_lux();
        sendTelemertry("power_2", mode_rem);
    }
    if ((mode_tuoi == true))
    {
        logic_sensor_temp();
        sendTelemertry("power_3", mode_tuoi);
    }
    if (WL_CONNECTED)
    {
        mqtt_loop();
    }
    NTPTimeUpdate();
    ModbusIpLoop();
    digitalWrite(RL3, rl[3]);
    digitalWrite(RL4, rl[4]);
}
void open_rem_1()
{
    state_rem_1 = true;
    digitalWrite(RL1, false);
    digitalWrite(RL2, true);
    Serial.println("rèm 1 mở");
}
void close_rem_1()
{
    state_rem_1 = false;
    digitalWrite(RL2, false);
    digitalWrite(RL1, true);
    Serial.println("rèm 1 đóng");
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
    StaticJsonDocument<1024> doc;
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
            bool state_relay = doc["params"]["power_1"];
            if (state_relay)
            {
                open_rem_1();
            }
            else
            {
                close_rem_1();
            }
            sendTelemertry("power_1", state_rem_1);
        }
        if (doc["params"].containsKey("power_2"))
        {
            mode_rem = doc["params"]["power_2"].as<bool>();
            sendTelemertry("power_2", mode_rem);
        }
        if (doc["params"].containsKey("power_3"))
        {
            mode_tuoi = doc["params"]["power_3"].as<bool>();
            sendTelemertry("power_3", mode_tuoi);
        }
        if (doc["params"].containsKey("power_4"))
        {
            rl[3] = doc["params"]["power_4"].as<bool>();
            sendTelemertry("power_4", rl[3]);
            digitalWrite(RL3, rl[3]);
        }
        if (doc["params"].containsKey("power_5"))
        {
            rl[4] = doc["params"]["power_5"].as<bool>();
            sendTelemertry("power_5", rl[4]);
            digitalWrite(RL4, rl[4]);
        }
        if (doc["params"].containsKey("set_LUX_LOW"))
        {
            set.light_Low = doc["params"]["set_LUX_LOW"];
            sendTelemertry("set_LUX_LOW", set.light_Low);
        }
        if (doc["params"].containsKey("set_LUX_HIGH"))
        {
            set.light_High = doc["params"]["set_LUX_HIGH"];
            sendTelemertry("set_LUX_HIGH", set.light_High);
        }
        if (doc["params"].containsKey("set_TEMP_LOW"))
        {
            set.temp_Low = doc["params"]["set_TEMP_LOW"];
            sendTelemertry("set_TEMP_LOW", set.temp_Low);
        }
        if (doc["params"].containsKey("set_TEMP_HIGH"))
        {
            set.temp_High = doc["params"]["set_TEMP_HIGH"];
            sendTelemertry("set_TEMP_HIGH", set.temp_High);
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
    static unsigned long lastTime;
    if (!client.connected())
    {
        mqtt_reconnect();
    }
    if (millis() - lastTime > 60000)
    {
        sendTelemertry("lux_value", lux);
        sendTelemertry("temp_value", temp);
        lastTime = millis();
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
            sendTelemertry("power_1", state_rem_1);
            sendTelemertry("power_2", mode_rem);
            sendTelemertry("power_3", mode_tuoi);
            sendTelemertry("power_4", rl[3]);
            sendTelemertry("power_5", rl[4]);
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
    DynamicJsonDocument data(200);
    value == true ? data[key] = true : data[key] = false;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, String value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, float value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, int value)
{
    DynamicJsonDocument data(200);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
////////////////////////////////////////////////////////////////////
void saveLocalStorage()
{
    DynamicJsonDocument data(512);

    data["power_1"] = state_rem_1;
    sendTelemertry("power_1", state_rem_1);

    mode_rem == true ? data["power_2"] = true : data["power_2"] = false;
    mode_tuoi == true ? data["power_3"] = true : data["power_3"] = false;
    rl[3] == true ? data["power_4"] = true : data["power_4"] = false;
    digitalWrite(RL3, rl[3]);
    rl[4] == true ? data["power_5"] = true : data["power_5"] = false;
    digitalWrite(RL4, rl[4]);
    data["set_LUX_LOW"] = set.light_Low;
    data["set_LUX_HIGH"] = set.light_High;
    data["set_TEMP_LOW"] = set.temp_Low;
    data["set_TEMP_HIGH"] = set.temp_High;
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
        if (doc.containsKey("power_1"))
        {
            bool state_relay = doc["power_1"];
            if (state_relay)
            {
                open_rem_1();
            }
            else
            {
                close_rem_1();
            }
        }
        mode_rem = doc["power_2"];
        mode_tuoi = doc["power_3"];
        rl[3] = doc["power_4"];
        rl[4] = doc["power_5"];
        digitalWrite(RL3, rl[3]);
        digitalWrite(RL4, rl[4]);
        set.light_High = doc["set_LUX_HIGH"];
        set.light_Low = doc["set_LUX_LOW"];
        set.temp_High = doc["set_TEMP_HIGH"];
        set.temp_Low = doc["set_TEMP_LOW"];

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
void logic_sensor_lux()
{

    if (lux < set.light_Low)
    {
        if (state_rem_1 == false && mode_rem == true)
        {
            open_rem_1();
            sendTelemertry("power_0", state_rem_1);
            sendTelemertry("power_2", mode_rem);
        }
    }
    else if (lux > set.light_High)
    {
        if (state_rem_1 == true && mode_rem == true)
        {
            close_rem_1();
            sendTelemertry("power_0", state_rem_1);
            sendTelemertry("power_2", mode_rem);
        }
    }
    sendTelemertry("power_0", state_rem_1);
}

void logic_sensor_temp()
{

    if ((set.temp_High < temp))
    {
        digitalWrite(RL4, rl[4] = true);
        sendTelemertry("power_5", rl[4]);
    }
    if ((temp < set.temp_Low))
    {
        digitalWrite(RL3, rl[3] = false);
        sendTelemertry("power_5", rl[4]);
    };
}

//////////////////////////////////////////////////////////////////
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
    StaticJsonDocument<2048> doc;
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
                        if (keyName == "power_1")
                        {
                            if (TIMER[i].state[k] == true)
                            {
                                open_rem_1();
                            }
                            else
                            {
                                close_rem_1();
                            }
                            sendTelemertry("power_1", state_rem_1);
                        }
                        rl[k] = TIMER[i].state[k];
                        sendTelemertry("power_3", mode_tuoi);
                        sendTelemertry("power_4", rl[3]);
                        sendTelemertry("power_5", rl[4]);
                    }
                }
            }
        }
        else
        {
        }
    }
}

void updateSchedule()
{

    timeClient.update();
    TSchedule m_schedule;
    time_t epochTime = timeClient.getEpochTime();
    struct tm *timeinfo = gmtime((time_t *)&epochTime);
    static int nowMM = 100;

    String hour;
    timeinfo->tm_hour > 9 ? hour = (String)timeinfo->tm_hour : hour = "0" + (String)timeinfo->tm_hour;
    String _time;
    timeinfo->tm_min > 9 ? _time = hour + ":" + (String)timeinfo->tm_min : _time = hour + ":0" + (String)timeinfo->tm_min;
    String _dayOfWeek = (String)timeinfo->tm_wday;
    int _mm = timeinfo->tm_min;

    for (int i = 0; i < _numOfSchedule; i++) // Quét từng object hẹn giờ
    {
        m_schedule = _mSchedule[i];
        if ((m_schedule.time == _time && m_schedule.enable == true) && (nowMM != _mm && strstr(m_schedule.interval.c_str(), _dayOfWeek.c_str())))
        {
            Serial.println("enter hen gio");
            nowMM = timeinfo->tm_min;
            for (int j = 0; j < m_schedule.action_count; j++)
            {
                //========Cài này là quét match với key power==========//
                for (int k = 0; k < 6; k++)
                {

                    String key = "power_" + (String)(k + 1);
                    if (m_schedule.action[j].key == key)
                    {
                        bool state = false;
                        m_schedule.action[j].value == "true" ? state = true : state = false;
                        digitalWrite(rl[k], state);
                        Serial.println("key: " + (String)m_schedule.action[j].key);
                        Serial.println("val: " + (String)m_schedule.action[j].value);
                    }
                }
                //===============Cái này là auto rem
                if (m_schedule.action[j].key == "power_2")
                {
                    bool state = false;
                    m_schedule.action[j].value == "true" ? state = true : state = false;
                    mode_rem = state;
                    sendTelemertry("power_2", mode_rem);
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value);
                }
                //===============Cái này là auto tuoi tren
                if (m_schedule.action[j].key == "power_3")
                {
                    bool state = false;
                    m_schedule.action[j].value == "true" ? state = true : state = false;
                    mode_tuoi = state;
                    sendTelemertry("power_2", mode_tuoi);
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value);
                }
            }
        }
    }
}
