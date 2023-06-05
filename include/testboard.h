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
#include <ML_ModbusRtuMaster.h>
#include "ModbusIP_ESP8266.h"
#include <FlowMeter.h>
#include "HardwareSerial.h"

// int RL[6] = {2, 4, 16, 5};
#define RL1 2
#define RL2 4
#define RL3 16
#define RL4 5

#define SLAVE_ID 2
#define KEY_POWER_DOSING "set_power_dosing"
#define KEY_TIME_TUOI_PHAN "time_tuoi_phan"
#define KEY_SET_TIME_A "set_time_a"
#define KEY_SET_TIME_B "set_time_b"
#define KEY_SET_TIME_C "set_time_c"
#define KEY_SET_TIME_D "set_time_d"
#define POWER_0 "power_0"
#define POWER_1 "power_1"
#define POWER_2 "power_2"
#define POWER_3 "power_3"
#define POWER_4 "power_4"
#define POWER_5 "power_5"
#define POWER_6 "power_6"
#define POWER_7 "power_7"
#define POWER_8 "power_8"
#define POWER_9 "power_9"
#define POWER_10 "power_10"
#define POWER_11 "power_11"
#define POWER_12 "power_12"
#define POWER_13 "power_13"
#define POWER_14 "power_14"
#define POWER_15 "power_15"
#define POWER_16 "power_16"
#define POWER_17 "power_17"
#define POWER_18 "power_18"
#define POWER_19 "power_19"
#define EC "ec"
#define Flow1 "flow1"
#define Flow2 "flow2"
#define Flow3 "flow3"
#define Flow4 "flow4"
#define RS485_TX 14
#define RS485_RX 12
bool power[20];

////////////////////////////////////////////////////
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "3tBqy4HdqLLZMbWV7ki4"
#define ID "6f0cbf10-f9d5-11ed-b16a-6fb926720a05"
#define TIME_CHECK_WIFI 2000
#define TIME_SEND_SS 1000
#define TIME_OFFSET 60000
////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient client(espClient);
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
AsyncWebServer server(80);
ModbusRTU mb;
ModbusIP mbIP;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
// const char *ssid = "nha_kinh_so_04";
// const char *pass = "H0983344442";
const char *ssid = "DalatTech";
const char *pass = "mltech@2019";
//////////////////////////////
// bool rl1 = false;
// bool rl2 = false;
// bool rl3 = false;
// bool rl4 = false;
bool rl[4];
//////////////////////////
FlowSensorProperties MySensor = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor2 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor3 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor4 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// connect a flow meter to an interrupt pin (see notes on your Arduino model for pin numbers)
FlowMeter *Meter1;
FlowMeter *Meter2;
FlowMeter *Meter3;
FlowMeter *Meter4;
//////////////////////////
const unsigned long period = 1000;
long lastTime = 0;
void Meter1ISR()
{
    Meter1->count();
}
void Meter2ISR()
{
    Meter2->count();
}

void Meter3ISR()
{
    Meter3->count();
}

void Meter4ISR()
{
    Meter4->count();
}
/////////////////////////
void readSensor();
void readEc();
void setTime();
void updateState();
void updateTelemertry();
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
void updateSchedule();
//////////////////////////////
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const long gmtOffset_sec = +6 * 60 * 60;
const int daylightOffset_sec = 3600;
uint8_t number_of_timers = 0;

//////////////////////////////
// IPAddress local_IP(192, 168, 1, 140);
// IPAddress gateway(192, 168, 1, 254);
// // IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   // optional
// IPAddress secondaryDNS(8, 8, 4, 4); // optional
//////////////////////////////////////////
// bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
// {
//     Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
// }

float flow1;
float flow2;
float flow3;
float flow4;
float ec;
///////////////////////////////////
bool state_system = false;
bool mode_tuoi = false;
#define TUOI_PHAN 0
#define TUOI_NUOC 1
int time_tuoi_phan = 100;
typedef enum
{
    IDLE_STATE,
    START_STATE,
    STOP_STATE,
    PHAN_STATE,
    NUOC_STATE
} state;
state dosing_state;
struct relay
{
    unsigned long lastTime;
    bool pre_state;
    bool now_state;
    int time_set;
};
relay rle[5];
struct timer
{
    int id;
    bool enable;
    String time;
    String interval;
    String keyName[30];
    bool state[30];
    int value[30];
};
timer TIMER[100];
unsigned long pre_time;
/////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////
void appInit()
{
    Serial.begin(9600);
    pinMode(RL1, OUTPUT);
    pinMode(RL2, OUTPUT);
    pinMode(RL3, OUTPUT);
    pinMode(RL4, OUTPUT);
    Meter1 = new FlowMeter(digitalPinToInterrupt(25), MySensor, Meter1ISR, FALLING);
    Meter2 = new FlowMeter(digitalPinToInterrupt(26), MySensor2, Meter2ISR, FALLING);
    Meter3 = new FlowMeter(digitalPinToInterrupt(27), MySensor3, Meter3ISR, RISING);
    Meter4 = new FlowMeter(digitalPinToInterrupt(32), MySensor4, Meter4ISR, RISING);
    Serial1.begin(9600, SERIAL_8N1, RS485_TX, RS485_RX);
    mb.begin(&Serial1);
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
    mbIP.server();
    mbIP.addCoil(0, false, 20);
    mb.addCoil(0, false, 20);
    mb.master();
    pre_time = millis();
}
uint16_t data[7];

void appRun()
{
    if (WL_CONNECTED)
    {
        mqtt_loop();
    }
    if ((millis() - pre_time) >= 100)
    {
        // Serial.println("polling");
        pre_time = millis();
        updateState();
    }
    readEc();
    readSensor();
    setTime();
    NTPTimeUpdate();
    mb.task();
    yield();
    mbIP.task();
}
void readSensor()
{

    long currentTime = millis();
    long duration = currentTime - lastTime;
    if (duration >= period)
    {
        Meter1->tick(period);
        Meter2->tick(period);
        Meter3->tick(period);
        Meter4->tick(period);
        //////////////////////////////
        Serial.println("Meter 1 currently " + String(Meter1->getCurrentFlowrate()) + " l/min, " + String(Meter1->getTotalVolume()) + " l total.");
        Serial.println("Meter 2 currently " + String(Meter2->getCurrentFlowrate()) + " l/min, " + String(Meter2->getTotalVolume()) + " l total.");
        Serial.println("Meter 3 currently " + String(Meter3->getCurrentFlowrate()) + " l/min, " + String(Meter3->getTotalVolume()) + " l total.");
        Serial.println("Meter 4 currently " + String(Meter4->getCurrentFlowrate()) + " l/min, " + String(Meter4->getTotalVolume()) + " l total.");
        flow1 = (float)Meter1->getCurrentFlowrate();
        flow2 = (float)Meter2->getCurrentFlowrate();
        flow3 = (float)Meter3->getCurrentFlowrate();
        flow4 = (float)Meter4->getCurrentFlowrate();
        lastTime = currentTime;
    }
}
void readEc()
{
    static unsigned long preTime = millis();
    if (millis() - preTime > 100)
    {
        if (!mb.slave())
        {
            //     for (int i = 0; i < 20; i++)
            // {
            //     mb.writeCoil(2, i, power[i]);
            // }
            mb.readHreg(2, 0, data, 7);
            ec = (float)data[0];
            Serial.print("ec ");
            Serial.print(ec);
            Serial.println(" us ");
        }
        preTime = millis();
    }
}
// void updateState()
// {
//     for (int i = 0; i < 20; i++)
//     {
//         mbIP.Coil(i, power[i]);
//     }
//     digitalWrite(RL1, rle[1].pre_state);
//     digitalWrite(RL2, rle[2].pre_state);
//     digitalWrite(RL3, rle[3].pre_state);
//     digitalWrite(RL4, rle[4].pre_state);
// }
void updateState()
{
    // if (!mb.slave())
    // {
    //     for (int i = 0; i < 20; i++)
    //     {
    //         mb.writeCoil(2, i, power[i]);
    //     }
    //     mb.task();
    //     yield();
    // }
    digitalWrite(RL1, rle[1].pre_state);
    digitalWrite(RL2, rle[2].pre_state);
    digitalWrite(RL3, rle[3].pre_state);
    digitalWrite(RL4, rle[4].pre_state);
}
void setTime()
{
    static unsigned long pre_time_tuoi_phan = millis();
    static unsigned long timeStart = millis();
    switch (dosing_state)
    {
    case IDLE_STATE:
    {
        if (state_system == true)
        {
            if (state_system == true)
            {
                timeStart = millis();
                dosing_state = START_STATE;
            }
        }
        break;
    }
    case START_STATE:
    {
        // Serial.println("STATRT STATE");

        power[0] = true;
        sendTelemertry(POWER_0, power[0]);

        power[1] = true;
        sendTelemertry(POWER_1, power[1]);

        mode_tuoi = TUOI_PHAN;
        if (mode_tuoi == TUOI_PHAN)
        {

            pre_time_tuoi_phan = millis();

            if (millis() - timeStart > 10000)
            {
                for (int i = 0; i < 5; i++)
                {
                    String key = "power_" + (String)(i + 15);

                    rle[i].time_set == 0 ? rle[i].pre_state = 0 : false;
                    rle[i].pre_state = true;
                    rle[i].lastTime = millis();
                    sendTelemertry(key, rle[i].pre_state);
                }
                dosing_state = PHAN_STATE;
            }
        }
        break;
    }
    case STOP_STATE:
    {
        break;
    }
    case PHAN_STATE:
    {
        // Serial.println("PHAN STATE");

        static bool pre_value = false;

        for (int i = 0; i < 5; i++)
        {
            String key = "power_" + (String)(i + 15);
            if (millis() - rle[i].lastTime >= rle[i].time_set * TIME_OFFSET)
            {
                rle[i].time_set == 0 ? rle[i].pre_state = 0 : false;
                rle[i].pre_state = false;
                sendTelemertry(key, rle[i].pre_state);
            }
        }

        if (millis() - pre_time_tuoi_phan >= time_tuoi_phan * TIME_OFFSET || mode_tuoi == TUOI_NUOC || state_system == false)
        {
            //    Serial.println("BREAK OUT PHANC_STATE");
            state_system = false;
            sendTelemertry(KEY_POWER_DOSING, state_system);
            power[0] = false;
            sendTelemertry(POWER_0, power[0]);
            power[1] = false;
            sendTelemertry(POWER_1, power[1]);
            for (int i = 0; i < 5; i++)
            {
                String key = "power_" + (String)(i + 15);
                rle[i].pre_state = false;
                sendTelemertry(key, rle[i].pre_state);
            }
            pre_value = false;
            dosing_state = IDLE_STATE;
            sendTelemertry(KEY_POWER_DOSING, state_system);
        }
        break;
    }
    }
}
void updateTelemertry()
{
    static bool prePump = false;
    static unsigned long timeSendFlow = 0;
    if (prePump != power[1])
    {
        prePump = power[1];
        sendTelemertry(POWER_1, power[1]);
    }
    for (int i = 0; i < 5; i++)
    {
        String key = "power_" + (String)(i + 15);

        if (rle[i].pre_state != rle[i].now_state)
        {
            rle[i].now_state = rle[i].pre_state;
            sendTelemertry(key, rle[i].pre_state);
        }
    }
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
        if (doc["params"].containsKey(KEY_POWER_DOSING))
        {
            state_system = doc["params"][KEY_POWER_DOSING].as<bool>();
            state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
        }
        if (doc["params"].containsKey(KEY_TIME_TUOI_PHAN))
        {
            time_tuoi_phan = doc["params"][KEY_TIME_TUOI_PHAN].as<int>();
            sendTelemertry(KEY_TIME_TUOI_PHAN, time_tuoi_phan);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_A))
        {
            rle[1].time_set = doc["params"][KEY_SET_TIME_A].as<int>();
            sendTelemertry(KEY_SET_TIME_A, rle[1].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_B))
        {
            rle[2].time_set = doc["params"][KEY_SET_TIME_B].as<int>();
            sendTelemertry(KEY_SET_TIME_B, rle[2].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_C))
        {
            rle[3].time_set = doc["params"][KEY_SET_TIME_C].as<int>();
            sendTelemertry(KEY_SET_TIME_C, rle[3].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_D))
        {
            rle[4].time_set = doc["params"][KEY_SET_TIME_D].as<int>();
            sendTelemertry(KEY_SET_TIME_D, rle[4].time_set);
        }
        if (doc["params"].containsKey(POWER_0))
        {
            power[0] = doc["params"][POWER_0].as<bool>();
            mbIP.Coil(0, power[0]);
            sendTelemertry(POWER_0, power[0]);
        }
        if (doc["params"].containsKey(POWER_1))
        {
            power[1] = doc["params"][POWER_1].as<bool>();
            mbIP.Coil(1, power[1]);
            sendTelemertry(POWER_1, power[1]);
        }
        if (doc["params"].containsKey(POWER_2))
        {
            power[2] = doc["params"][POWER_2].as<bool>();
            mbIP.Coil(2, power[2]);
            sendTelemertry(POWER_2, power[2]);
        }
        if (doc["params"].containsKey(POWER_3))
        {
            power[3] = doc["params"][POWER_3].as<bool>();
            mbIP.Coil(3, power[3]);
            sendTelemertry(POWER_3, power[3]);
        }
        if (doc["params"].containsKey(POWER_4))
        {
            power[4] = doc["params"][POWER_4].as<bool>();
            mbIP.Coil(4, power[4]);
            sendTelemertry(POWER_4, power[4]);
        }
        if (doc["params"].containsKey(POWER_5))
        {
            power[5] = doc["params"][POWER_5].as<bool>();
            mbIP.Coil(5, power[5]);
            sendTelemertry(POWER_5, power[5]);
        }
        if (doc["params"].containsKey(POWER_6))
        {
            power[6] = doc["params"][POWER_6].as<bool>();
            mbIP.Coil(6, power[6]);
            sendTelemertry(POWER_6, power[6]);
        }
        if (doc["params"].containsKey(POWER_7))
        {
            power[7] = doc["params"][POWER_7].as<bool>();
            mbIP.Coil(7, power[7]);
            sendTelemertry(POWER_7, power[7]);
        }
        if (doc["params"].containsKey(POWER_8))
        {
            power[8] = doc["params"][POWER_8].as<bool>();
            mbIP.Coil(8, power[8]);
            sendTelemertry(POWER_8, power[8]);
        }
        if (doc["params"].containsKey(POWER_9))
        {
            power[9] = doc["params"][POWER_9].as<bool>();
            mbIP.Coil(9, power[9]);
            sendTelemertry(POWER_9, power[9]);
        }
        if (doc["params"].containsKey(POWER_10))
        {
            power[10] = doc["params"][POWER_10].as<bool>();
            mbIP.Coil(10, power[10]);
            sendTelemertry(POWER_10, power[10]);
        }
        if (doc["params"].containsKey(POWER_11))
        {
            power[11] = doc["params"][POWER_11].as<bool>();
            mbIP.Coil(11, power[11]);
            sendTelemertry(POWER_11, power[11]);
        }
        if (doc["params"].containsKey(POWER_12))
        {
            power[12] = doc["params"][POWER_12].as<bool>();
            mbIP.Coil(12, power[12]);
            sendTelemertry(POWER_12, power[12]);
        }
        if (doc["params"].containsKey(POWER_13))
        {
            power[13] = doc["params"][POWER_13].as<bool>();
            mbIP.Coil(13, power[13]);
            sendTelemertry(POWER_13, power[13]);
        }
        if (doc["params"].containsKey(POWER_14))
        {
            power[14] = doc["params"][POWER_14].as<bool>();
            mbIP.Coil(14, power[14]);
            sendTelemertry(POWER_14, power[14]);
        }
        if (doc["params"].containsKey(POWER_15))
        {
            power[15] = doc["params"][POWER_15].as<bool>();
            mbIP.Coil(15, power[15]);
            sendTelemertry(POWER_15, power[15]);
        }
        if (doc["params"].containsKey("power_16"))
        {
            rle[1].pre_state = doc["params"]["power_16"].as<bool>();
            sendTelemertry("power_16", rle[1].pre_state);
            digitalWrite(RL1, rle[1].pre_state);
        }
        if (doc["params"].containsKey("power_17"))
        {
            rle[2].pre_state = doc["params"]["power_17"].as<bool>();
            sendTelemertry("power_17", rle[2].pre_state);
            digitalWrite(RL2, rle[2].pre_state);
        }
        if (doc["params"].containsKey("power_18"))
        {
            rle[3].pre_state = doc["params"]["power_18"].as<bool>();
            sendTelemertry("power_18", rle[3].pre_state);
            digitalWrite(RL3, rle[3].pre_state);
        }
        if (doc["params"].containsKey("power_19"))
        {
            rle[4].pre_state = doc["params"]["power_19"].as<bool>();
            sendTelemertry("power_19", rle[4].pre_state);
            digitalWrite(RL4, rle[4].pre_state);
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
    if (millis() - lastTime > 10000)
    {
        sendTelemertry(EC, ec);
        sendTelemertry(Flow1, flow1);
        sendTelemertry(Flow2, flow2);
        sendTelemertry(Flow3, flow3);
        sendTelemertry(Flow4, flow4);
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
            sendTelemertry(KEY_SET_TIME_A, rle[1].time_set);
            sendTelemertry(KEY_SET_TIME_B, rle[2].time_set);
            sendTelemertry(KEY_SET_TIME_C, rle[3].time_set);
            sendTelemertry(KEY_SET_TIME_D, rle[4].time_set);

            sendTelemertry(KEY_POWER_DOSING, state_system);
            sendTelemertry(POWER_0, power[0]);
            sendTelemertry(POWER_1, power[1]);
            sendTelemertry(POWER_2, power[2]);
            sendTelemertry(POWER_3, power[3]);
            sendTelemertry(POWER_4, power[4]);
            sendTelemertry(POWER_5, power[5]);
            sendTelemertry(POWER_6, power[6]);
            sendTelemertry(POWER_7, power[7]);
            sendTelemertry(POWER_8, power[8]);
            sendTelemertry(POWER_9, power[9]);
            sendTelemertry(POWER_10, power[10]);
            sendTelemertry(POWER_11, power[11]);
            sendTelemertry(POWER_12, power[12]);
            sendTelemertry(POWER_13, power[13]);
            sendTelemertry(POWER_14, power[14]);
            sendTelemertry(POWER_15, power[15]);
            sendTelemertry(POWER_16, rle[1].pre_state);
            sendTelemertry(POWER_17, rle[2].pre_state);
            sendTelemertry(POWER_18, rle[3].pre_state);
            sendTelemertry(POWER_19, rle[4].pre_state);
            // sendTelemertry(_POWER_17, power[16]);
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
    DynamicJsonDocument data(1024);
    // for (int i = 0; i < 16; i++)
    // {
    //     power[i] == true ? data["power_" + (String)(i)] = true : data["power_" + (String)(i)] = false;
    // }
    data[KEY_SET_TIME_A] = rle[1].time_set;
    data[KEY_SET_TIME_B] = rle[2].time_set;
    data[KEY_SET_TIME_C] = rle[3].time_set;
    data[KEY_SET_TIME_D] = rle[4].time_set;
    data[KEY_TIME_TUOI_PHAN] = time_tuoi_phan;

    power[0] == true ? data["power_0"] = true : data["power_0"] = false;
    power[1] == true ? data["power_1"] = true : data["power_1"] = false;
    power[2] == true ? data["power_2"] = true : data["power_2"] = false;
    power[3] == true ? data["power_3"] = true : data["power_3"] = false;
    power[4] == true ? data["power_4"] = true : data["power_4"] = false;
    power[5] == true ? data["power_5"] = true : data["power_5"] = false;
    power[6] == true ? data["power_6"] = true : data["power_6"] = false;
    power[7] == true ? data["power_7"] = true : data["power_7"] = false;
    power[8] == true ? data["power_8"] = true : data["power_8"] = false;
    power[9] == true ? data["power_9"] = true : data["power_9"] = false;
    power[10] == true ? data["power_10"] = true : data["power_10"] = false;
    power[11] == true ? data["power_11"] = true : data["power_11"] = false;
    power[12] == true ? data["power_12"] = true : data["power_12"] = false;
    power[13] == true ? data["power_13"] = true : data["power_13"] = false;
    power[14] == true ? data["power_14"] = true : data["power_14"] = false;
    power[15] == true ? data["power_15"] = true : data["power_15"] = false;
    rle[1].pre_state == true ? data["power_1"] = true : data["power_1"] = false;
    rle[2].pre_state == true ? data["power_2"] = true : data["power_2"] = false;
    rle[3].pre_state == true ? data["power_3"] = true : data["power_3"] = false;
    rle[4].pre_state == true ? data["power_4"] = true : data["power_4"] = false;

    String objectString;
    serializeJson(data, objectString);
    writeFile(LITTLEFS, "/data/localStorage.json", objectString.c_str());
    Serial.println(objectString);
}
void localStorageExport()
{
    String data = readFile(LITTLEFS, "/data/localStorage.json");
    Serial.println("READ FILE: " + (String)data);
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, data.c_str());
    if (error)
    {
        Serial.println("deserializeJson failed");
        Serial.println(error.f_str());
        return;
    }
    else
    {
        rle[1].time_set = doc[KEY_SET_TIME_A];
        rle[2].time_set = doc[KEY_SET_TIME_B];
        rle[3].time_set = doc[KEY_SET_TIME_C];
        rle[4].time_set = doc[KEY_SET_TIME_D];

        time_tuoi_phan = doc[KEY_TIME_TUOI_PHAN].as<int>();

        power[0] = doc["power_0"];
        power[1] = doc["power_1"];
        power[2] = doc["power_2"];
        power[3] = doc["power_3"];
        power[4] = doc["power_4"];
        power[5] = doc["power_5"];
        power[6] = doc["power_6"];
        power[7] = doc["power_7"];
        power[8] = doc["power_8"];
        power[9] = doc["power_9"];
        power[10] = doc["power_10"];
        power[11] = doc["power_11"];
        power[12] = doc["power_12"];
        power[13] = doc["power_13"];
        power[14] = doc["power_14"];
        power[15] = doc["power_15"];
        rle[1].pre_state = doc["power_16"];
        rle[2].pre_state = doc["power_17"];
        rle[3].pre_state = doc["power_18"];
        rle[4].pre_state = doc["power_19"];

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
        TIMER[i].keyName[2] = "";
        TIMER[i].keyName[3] = "";
        TIMER[i].keyName[4] = "";
        TIMER[i].keyName[5] = "";
        for (int k = 0; k < 30; k++)
        {
            TIMER[i].keyName[k + 6] = "";
        }

        if (doc["data"][i]["action"].containsKey("set_power_dosing"))
        {
            String a = doc["data"][i]["action"]["set_power_dosing"];
            TIMER[i].keyName[0] = "set_power_dosing";
            a == "true" ? TIMER[i].state[0] = true : TIMER[i].state[0] = false;
        }
        if (doc["data"][i]["action"].containsKey("time_tuoi_phan"))
        {
            String a = doc["data"][i]["action"]["time_tuoi_phan"];
            TIMER[i].keyName[1] = "time_tuoi_phan";
            TIMER[i].value[0] = a.toInt();
        }
        if (doc["data"][i]["action"].containsKey("set_time_a"))
        {
            String a = doc["data"][i]["action"]["set_time_a"];
            TIMER[i].keyName[2] = "set_time_a";
            TIMER[i].value[1] = a.toInt();
        }
        if (doc["data"][i]["action"].containsKey("set_time_b"))
        {
            String a = doc["data"][i]["action"]["set_time_b"];
            TIMER[i].keyName[3] = "set_time_b";
            TIMER[i].value[2] = a.toInt();
        }
        if (doc["data"][i]["action"].containsKey("set_time_c"))
        {
            String a = doc["data"][i]["action"]["set_time_c"];
            TIMER[i].keyName[4] = "set_time_c";
            TIMER[i].value[3] = a.toInt();
        }
        if (doc["data"][i]["action"].containsKey("set_time_d"))
        {
            String a = doc["data"][i]["action"]["set_time_d"];
            TIMER[i].keyName[5] = "set_time_d";
            TIMER[i].value[4] = a.toInt();
        }

        for (int k = 0; k < 30; k++)
        {
            String tempKeyName = "power_" + (String)(k);
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
                if (TIMER[i].keyName[0] == "set_power_dosing")
                {
                    // auto_by = TIMER[i].state[0];
                    state_system = TIMER[i].state[0];
                    state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
                }

                if (TIMER[i].keyName[1] == "time_tuoi_phan")
                {
                    time_tuoi_phan = TIMER[i].value[0];
                }

                if (TIMER[i].keyName[2] == "set_time_a")
                {
                    rle[1].time_set = TIMER[i].value[1];
                }
                if (TIMER[i].keyName[3] == "set_time_b")
                {
                    rle[2].time_set = TIMER[i].value[2];
                }
                if (TIMER[i].keyName[4] == "set_time_c")
                {
                    rle[3].time_set = TIMER[i].value[3];
                }
                if (TIMER[i].keyName[5] == "set_time_d")
                {
                    rle[4].time_set = TIMER[i].value[4];
                }
                ///////////////////////////////////////////////////////////////

                for (int k = 0; k < 30; k++)
                {
                    String keyName = "power_" + (String)(k);
                    if (TIMER[i].keyName[k] == keyName)
                    {
                        // digitalWrite(coils_[k], TIMER[i].state[k]);
                        // rl[k] = TIMER[i].state[k];
                        // digitalWrite(RL[k], rl[k]);
                        power[k] = TIMER[i].state[k];
                        Serial.println("STATE RL: " + (String)k + "state: " + (String)TIMER[i].state[0]);
                        sendTelemertry("power_0", power[0]);
                        sendTelemertry("power_1", power[1]);
                        sendTelemertry("power_2", power[2]);
                        sendTelemertry("power_3", power[3]);
                        sendTelemertry("power_4", power[4]);
                        sendTelemertry("power_5", power[5]);
                        sendTelemertry("power_6", power[6]);
                        sendTelemertry("power_7", power[7]);
                        sendTelemertry("power_8", power[8]);
                        sendTelemertry("power_9", power[9]);
                        sendTelemertry("power_10", power[10]);
                        sendTelemertry("power_11", power[11]);
                        sendTelemertry("power_12", power[12]);
                        sendTelemertry("power_13", power[13]);
                        sendTelemertry("power_14", power[14]);
                        sendTelemertry("power_15", power[15]);
                        sendTelemertry("power_16", rle[1].pre_state);
                        sendTelemertry("power_17", rle[2].pre_state);
                        sendTelemertry("power_18", rle[3].pre_state);
                        sendTelemertry("power_19", rle[4].pre_state);
                    }
                }
            }
        }
        else
        {
        }
    }
}
void jsonParseToStruct(String jsonString, TSchedule *m_schedule)
{
    Serial.println("============================");
    Serial.println(jsonString);
    StaticJsonDocument<4096> doc;
    deserializeJson(doc, jsonString);
    JsonArray array = doc["data"].as<JsonArray>();
    _numOfSchedule = array.size();
    Serial.println("size schedule: " + (String)_numOfSchedule);
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
                // van tuoi
                for (int k = 0; k < 5; k++)
                {
                    String key = "power_" + (String)(k);
                    if (m_schedule.action[j].key == key)
                    {
                        bool state = false;
                        m_schedule.action[j].value == "true" ? state = true : state = false;
                        rle[k].pre_state = m_schedule.action[j].value;
                        Serial.println("key: " + (String)m_schedule.action[j].key);
                        Serial.println("val: " + (String)m_schedule.action[j].value);
                    }
                }
                // cai nay la van tuoi + bơm
                for (int k = 0; k < 16; k++)
                {
                    String key = "power_" + (String)(k);
                    if (m_schedule.action[j].key == key)
                    {
                        bool state = false;
                        m_schedule.action[j].value == "true" ? state = true : state = false;
                        power[k] = m_schedule.action[j].value;
                        Serial.println("key: " + (String)m_schedule.action[j].key);
                        Serial.println("val: " + (String)m_schedule.action[j].value);
                    }
                }
                // cai nay la he thong
                if (m_schedule.action[j].key == "set_power_dosing")
                {
                    bool state = false;
                    // auto_by = TIMER[i].state[0];
                    m_schedule.action[j].value == "true" ? state = true : state = false;
                    state_system = m_schedule.action[j].value;
                    state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
                }
                // cai nay la thoi gian tuoi phan

                if (m_schedule.action[j].key == "time_tuoi_phan")
                {
                    time_tuoi_phan = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van A

                if (m_schedule.action[j].key == "set_time_a")
                {
                    rle[0].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van B

                if (m_schedule.action[j].key == "set_time_b")
                {
                    rle[1].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van c

                if (m_schedule.action[j].key == "set_time_c")
                {
                    rle[2].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van d

                if (m_schedule.action[j].key == "set_time_d")
                {
                    rle[3].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
            }
        }
    }
}
///////////////////////////
// #include "Arduino.h"
// #include "ETH.h"
// #include "ModbusRTU.h"
// #include <ModbusIP_ESP8266.h>
// #include <Arduino.h>
// #include <HTTPClient.h>
// #include "WiFi.h"
// #include "PubSubClient.h"
// #include "ArduinoJson.h"
// #include "time.h"
// #include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncElegantOTA.h>
// #include "FS.h"
// #include <LITTLEFS.h>
// #include "NTPClient.h"
// #include "WiFiUdp.h"
// #include <ML_ModbusRtuMaster.h>
// #include "ModbusIP_ESP8266.h"
// #include <FlowMeter.h>

// // int RL[6] = {2, 4, 16, 5};
// #define RL1 2
// #define RL2 4
// #define RL3 16
// #define RL4 5

// #define SLAVE_ID 1
// #define KEY_POWER_DOSING "set_power_dosing"
// #define KEY_TIME_TUOI_PHAN "time_tuoi_phan"
// #define KEY_SET_TIME_A "set_time_a"
// #define KEY_SET_TIME_B "set_time_b"
// #define KEY_SET_TIME_C "set_time_c"
// #define KEY_SET_TIME_D "set_time_d"
// #define POWER_0 "power_0"
// #define POWER_1 "power_1"
// #define POWER_2 "power_2"
// #define POWER_3 "power_3"
// #define POWER_4 "power_4"
// #define POWER_5 "power_5"
// #define POWER_6 "power_6"
// #define POWER_7 "power_7"
// #define POWER_8 "power_8"
// #define POWER_9 "power_9"
// #define POWER_10 "power_10"
// #define POWER_11 "power_11"
// #define POWER_12 "power_12"
// #define POWER_13 "power_13"
// #define POWER_14 "power_14"
// #define POWER_15 "power_15"
// #define POWER_16 "power_16"
// #define POWER_17 "power_17"
// #define POWER_18 "power_18"
// #define POWER_19 "power_19"
// #define EC "ec"
// #define Flow1 "flow1"
// #define Flow2 "flow2"
// #define Flow3 "flow3"
// #define Flow4 "flow4"

// #define RS485_TX 14
// #define RS485_RX 12

// bool power[20];

// ////////////////////////////////////////////////////
// #define THINGS_BOARD_SERVER "mqtt.viis.tech"
// #define TOKEN "3tBqy4HdqLLZMbWV7ki4"
// #define ID "6f0cbf10-f9d5-11ed-b16a-6fb926720a05"
// #define TIME_CHECK_WIFI 2000
// #define TIME_SEND_SS 1000
// #define TIME_OFFSET 60000
// ////////////////////////////////////////////////////
// WiFiClient espClient;
// PubSubClient client(espClient);
// String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
// AsyncWebServer server(80);
// ModbusRTU mb;
// bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
// {
//     Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
//     return true;
// }
// ModbusIP mbIP;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
// // const char *ssid = "nha_kinh_so_04";
// // const char *pass = "H0983344442";
// const char *ssid = "DalatTech";
// const char *pass = "mltech@2019";
// //////////////////////////////
// // bool rl1 = false;
// // bool rl2 = false;
// // bool rl3 = false;
// // bool rl4 = false;
// bool rl[4];
// //////////////////////////
// FlowSensorProperties MySensor = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// FlowSensorProperties MySensor2 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// FlowSensorProperties MySensor3 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// FlowSensorProperties MySensor4 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// // connect a flow meter to an interrupt pin (see notes on your Arduino model for pin numbers)
// FlowMeter *Meter1;
// FlowMeter *Meter2;
// FlowMeter *Meter3;
// FlowMeter *Meter4;
// //////////////////////////
// const unsigned long period = 1000;
// long lastTime = 0;
// void Meter1ISR()
// {
//     Meter1->count();
// }
// void Meter2ISR()
// {
//     Meter2->count();
// }

// void Meter3ISR()
// {
//     Meter3->count();
// }

// void Meter4ISR()
// {
//     Meter4->count();
// }
// /////////////////////////
// void readSensor();
// void readEc();
// void setTime();
// void updateState();
// void updateTelemertry();
// void wifiConnect();
// void sendTelemertry(String key, bool value);
// void sendTelemertry(String key, String value);
// void sendTelemertry(String key, float value);
// void sendTelemertry(String key, int value);
// void mqttInit();
// void on_message(const char *topic, byte *payload, unsigned int length);
// void mqtt_loop();
// void mqtt_reconnect();
// void saveLocalStorage();
// void localStorageExport();
// //////////////////////
// void timeInit();
// void printLocalTime();
// void NTPTimeUpdate();
// void jsonObjectTimer(String jsonString);
// void timerInit();
// String httpGETRequest(const char *serverName);
// /*Local Storage API*/
// void fileInit();
// void writeFile(fs::FS &fs, const char *path, const char *message);
// String readFile(fs::FS &fs, const char *path);
// void createDir(fs::FS &fs, const char *path);
// void updateSchedule();
// //////////////////////////////
// String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// // Month names
// String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
// const long gmtOffset_sec = +6 * 60 * 60;
// const int daylightOffset_sec = 3600;
// uint8_t number_of_timers = 0;

// //////////////////////////////
// // IPAddress local_IP(192, 168, 1, 140);
// // IPAddress gateway(192, 168, 1, 254);
// // // IPAddress gateway(192, 168, 1, 1);
// // IPAddress subnet(255, 255, 255, 0);
// // IPAddress primaryDNS(8, 8, 8, 8);   // optional
// // IPAddress secondaryDNS(8, 8, 4, 4); // optional
// //////////////////////////////////////////
// // bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
// // {
// //     Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
// // }

// float flow1;
// float flow2;
// float flow3;
// float flow4;
// float ec;
// ///////////////////////////////////
// bool state_system = false;
// bool mode_tuoi = false;
// #define TUOI_PHAN 0
// #define TUOI_NUOC 1
// int time_tuoi_phan = 100;
// typedef enum
// {
//     IDLE_STATE,
//     START_STATE,
//     STOP_STATE,
//     PHAN_STATE,
//     NUOC_STATE
// } state;
// state dosing_state;
// struct relay
// {
//     unsigned long lastTime;
//     bool pre_state;
//     bool now_state;
//     int time_set;
// };
// relay rle[5];
// struct timer
// {
//     int id;
//     bool enable;
//     String time;
//     String interval;
//     String keyName[30];
//     bool state[30];
//     int value[30];
// };
// timer TIMER[100];
// unsigned long pre_time;
// /////////////////////////////////////////////////////////////////////////////
// #define MAX_ACTION 50
// #define MAX_TIMER 10
// int _numOfSchedule = 0;
// struct Action
// {
//     String key;
//     String value;
// };
// struct TSchedule
// {
//     bool enable;
//     String time;
//     String interval;
//     int action_count;
//     Action action[MAX_ACTION];
// };
// TSchedule _mSchedule[MAX_TIMER];
// void jsonParseToStruct(String jsonString, TSchedule *m_schedule);
// ////////////////////////////////////////////////////////////////////
// void appInit()
// {
//     Serial.begin(9600);
//     Serial1.begin(9600, SERIAL_8N1, RS485_TX, RS485_RX);
//     mb.begin(&Serial1);
//     pinMode(RL1, OUTPUT);
//     pinMode(RL2, OUTPUT);
//     pinMode(RL3, OUTPUT);
//     pinMode(RL4, OUTPUT);
//     Meter1 = new FlowMeter(digitalPinToInterrupt(25), MySensor, Meter1ISR, FALLING);
//     Meter2 = new FlowMeter(digitalPinToInterrupt(26), MySensor2, Meter2ISR, FALLING);
//     Meter3 = new FlowMeter(digitalPinToInterrupt(27), MySensor3, Meter3ISR, RISING);
//     Meter4 = new FlowMeter(digitalPinToInterrupt(32), MySensor4, Meter4ISR, RISING);
//     fileInit();
//     timerInit();
//     // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
//     // {
//     //     Serial.println("STA Failed to configure");
//     // }
//     WiFi.begin(ssid, pass);
//     mqttInit();
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         Serial.println("wificonnect...");
//         Serial.print(".");
//     }

//     Serial.println("");
//     Serial.println("WiFi connected");
//     Serial.println("IP address: ");
//     Serial.println(WiFi.localIP());
//     localStorageExport();
//     createDir(LITTLEFS, "/data");
//     writeFile(LITTLEFS, "/data/config.json", "hello");
//     Serial.println(readFile(LITTLEFS, "/data/config.json"));
//     mbIP.server();
//     mb.addCoil(0, false, 20);
//     mb.master();
//     pre_time = millis();
// }
// uint16_t data[7];

// void appRun()
// {
//     if (WL_CONNECTED)
//     {
//         mqtt_loop();
//     }
//     if ((millis() - pre_time) >= 100)
//     {
//         // Serial.println("polling");
//         pre_time = millis();
//         updateState();
//     }
//     readEc();
//     readSensor();
//     setTime();
//     NTPTimeUpdate();
//     mbIP.task();
// }
// void readSensor()
// {

//     long currentTime = millis();
//     long duration = currentTime - lastTime;
//     if (duration >= period)
//     {
//         Meter1->tick(period);
//         Meter2->tick(period);
//         Meter3->tick(period);
//         Meter4->tick(period);
//         //////////////////////////////
//         Serial.println("Meter 1 currently " + String(Meter1->getCurrentFlowrate()) + " l/min, " + String(Meter1->getTotalVolume()) + " l total.");
//         Serial.println("Meter 2 currently " + String(Meter2->getCurrentFlowrate()) + " l/min, " + String(Meter2->getTotalVolume()) + " l total.");
//         Serial.println("Meter 3 currently " + String(Meter3->getCurrentFlowrate()) + " l/min, " + String(Meter3->getTotalVolume()) + " l total.");
//         Serial.println("Meter 4 currently " + String(Meter4->getCurrentFlowrate()) + " l/min, " + String(Meter4->getTotalVolume()) + " l total.");
//         flow1 = (float)Meter1->getCurrentFlowrate();
//         flow2 = (float)Meter2->getCurrentFlowrate();
//         flow3 = (float)Meter3->getCurrentFlowrate();
//         flow4 = (float)Meter4->getCurrentFlowrate();
//         lastTime = currentTime;
//     }
// }
// void readEc()
// {
//     static unsigned long preTime = millis();
//     if (millis() - preTime > 100)
//     {
//         if (!mb.slave())
//         {
//             mb.readHreg(2, 0, data, 7);
//             ec = (float)data[0];
//             Serial.print("ec ");
//             Serial.print(ec);
//             Serial.println(" us ");
//         }
//         mb.task();
//         yield();
//         preTime = millis();
//     }
// }
// void updateState()
// {
//     if (!mb.slave())
//     {
//         for (int i = 0; i < 20; i++)
//         {
//             mb.writeCoil(2, i, power[i]);
//         }
//         mb.task();
//         yield();
//     }
//     digitalWrite(RL1, rle[1].pre_state);
//     digitalWrite(RL2, rle[2].pre_state);
//     digitalWrite(RL3, rle[3].pre_state);
//     digitalWrite(RL4, rle[4].pre_state);
// }
// void setTime()
// {
//     static unsigned long pre_time_tuoi_phan = millis();
//     static unsigned long timeStart = millis();
//     switch (dosing_state)
//     {
//     case IDLE_STATE:
//     {
//         if (state_system == true)
//         {
//             if (state_system == true)
//             {
//                 timeStart = millis();
//                 dosing_state = START_STATE;
//             }
//         }
//         break;
//     }
//     case START_STATE:
//     {
//         // Serial.println("STATRT STATE");

//         power[0] = true;
//         sendTelemertry(POWER_0, power[0]);

//         power[1] = true;
//         sendTelemertry(POWER_1, power[1]);

//         mode_tuoi = TUOI_PHAN;
//         if (mode_tuoi == TUOI_PHAN)
//         {

//             pre_time_tuoi_phan = millis();

//             if (millis() - timeStart > 10000)
//             {
//                 for (int i = 0; i < 5; i++)
//                 {
//                     String key = "power_" + (String)(i + 15);

//                     rle[i].time_set == 0 ? rle[i].pre_state = 0 : false;
//                     rle[i].pre_state = true;
//                     rle[i].lastTime = millis();
//                     sendTelemertry(key, rle[i].pre_state);
//                 }
//                 dosing_state = PHAN_STATE;
//             }
//         }
//         break;
//     }
//     case STOP_STATE:
//     {
//         break;
//     }
//     case PHAN_STATE:
//     {
//         // Serial.println("PHAN STATE");

//         static bool pre_value = false;

//         for (int i = 0; i < 5; i++)
//         {
//             String key = "power_" + (String)(i + 15);
//             if (millis() - rle[i].lastTime >= rle[i].time_set * TIME_OFFSET)
//             {
//                 rle[i].time_set == 0 ? rle[i].pre_state = 0 : false;
//                 rle[i].pre_state = false;
//                 sendTelemertry(key, rle[i].pre_state);
//             }
//         }

//         if (millis() - pre_time_tuoi_phan >= time_tuoi_phan * TIME_OFFSET || mode_tuoi == TUOI_NUOC || state_system == false)
//         {
//             //    Serial.println("BREAK OUT PHANC_STATE");
//             state_system = false;
//             sendTelemertry(KEY_POWER_DOSING, state_system);
//             power[0] = false;
//             sendTelemertry(POWER_0, power[0]);
//             power[1] = false;
//             sendTelemertry(POWER_1, power[1]);
//             for (int i = 0; i < 5; i++)
//             {
//                 String key = "power_" + (String)(i + 15);
//                 rle[i].pre_state = false;
//                 sendTelemertry(key, rle[i].pre_state);
//             }
//             pre_value = false;
//             dosing_state = IDLE_STATE;
//             sendTelemertry(KEY_POWER_DOSING, state_system);
//         }
//         break;
//     }
//     }
// }
// void updateTelemertry()
// {
//     static bool prePump = false;
//     static unsigned long timeSendFlow = 0;
//     if (prePump != power[1])
//     {
//         prePump = power[1];
//         sendTelemertry(POWER_1, power[1]);
//     }
//     for (int i = 0; i < 5; i++)
//     {
//         String key = "power_" + (String)(i + 15);

//         if (rle[i].pre_state != rle[i].now_state)
//         {
//             rle[i].now_state = rle[i].pre_state;
//             sendTelemertry(key, rle[i].pre_state);
//         }
//     }
// }
// void wifiConnect()
// {
//     static unsigned long preTime = millis();
//     if ((millis() - preTime > TIME_CHECK_WIFI) && (WiFi.status() != WL_CONNECTED))
//     {
//         Serial.println("WiFi connecting...");
//         preTime = millis();
//     }
// }
// void mqttInit()
// {
//     client.setServer(THINGS_BOARD_SERVER, 1883);
//     client.setCallback(on_message);
// }
// void on_message(const char *topic, byte *payload, unsigned int length)
// {
//     StaticJsonDocument<1024> doc;
//     Serial.println("On message");
//     char json[length + 1];
//     strncpy(json, (char *)payload, length);
//     json[length] = '\0';
//     Serial.println("TOPIC: " + (String)topic);
//     Serial.println("Message: " + (String)json);
//     DeserializationError error = deserializeJson(doc, json);
//     if (error)
//     {
//         Serial.println("deserializeJson failed");
//         Serial.println(error.f_str());
//         return;
//     }
//     if (strstr((char *)payload, "set_state") != NULL)
//     {
//         if (doc["params"].containsKey(KEY_POWER_DOSING))
//         {
//             state_system = doc["params"][KEY_POWER_DOSING].as<bool>();
//             state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
//         }
//         if (doc["params"].containsKey(KEY_TIME_TUOI_PHAN))
//         {
//             time_tuoi_phan = doc["params"][KEY_TIME_TUOI_PHAN].as<int>();
//             sendTelemertry(KEY_TIME_TUOI_PHAN, time_tuoi_phan);
//         }
//         if (doc["params"].containsKey(KEY_SET_TIME_A))
//         {
//             rle[1].time_set = doc["params"][KEY_SET_TIME_A].as<int>();
//             sendTelemertry(KEY_SET_TIME_A, rle[1].time_set);
//         }
//         if (doc["params"].containsKey(KEY_SET_TIME_B))
//         {
//             rle[2].time_set = doc["params"][KEY_SET_TIME_B].as<int>();
//             sendTelemertry(KEY_SET_TIME_B, rle[2].time_set);
//         }
//         if (doc["params"].containsKey(KEY_SET_TIME_C))
//         {
//             rle[3].time_set = doc["params"][KEY_SET_TIME_C].as<int>();
//             sendTelemertry(KEY_SET_TIME_C, rle[3].time_set);
//         }
//         if (doc["params"].containsKey(KEY_SET_TIME_D))
//         {
//             rle[4].time_set = doc["params"][KEY_SET_TIME_D].as<int>();
//             sendTelemertry(KEY_SET_TIME_D, rle[4].time_set);
//         }
//         if (doc["params"].containsKey(POWER_0))
//         {
//             power[0] = doc["params"][POWER_0].as<bool>();
//             mbIP.Coil(0, power[0]);
//             sendTelemertry(POWER_0, power[0]);
//         }
//         if (doc["params"].containsKey(POWER_1))
//         {
//             power[1] = doc["params"][POWER_1].as<bool>();
//             mbIP.Coil(1, power[1]);
//             sendTelemertry(POWER_1, power[1]);
//         }
//         if (doc["params"].containsKey(POWER_2))
//         {
//             power[2] = doc["params"][POWER_2].as<bool>();
//             mbIP.Coil(2, power[2]);
//             sendTelemertry(POWER_2, power[2]);
//         }
//         if (doc["params"].containsKey(POWER_3))
//         {
//             power[3] = doc["params"][POWER_3].as<bool>();
//             mbIP.Coil(3, power[3]);
//             sendTelemertry(POWER_3, power[3]);
//         }
//         if (doc["params"].containsKey(POWER_4))
//         {
//             power[4] = doc["params"][POWER_4].as<bool>();
//             mbIP.Coil(4, power[4]);
//             sendTelemertry(POWER_4, power[4]);
//         }
//         if (doc["params"].containsKey(POWER_5))
//         {
//             power[5] = doc["params"][POWER_5].as<bool>();
//             mbIP.Coil(5, power[5]);
//             sendTelemertry(POWER_5, power[5]);
//         }
//         if (doc["params"].containsKey(POWER_6))
//         {
//             power[6] = doc["params"][POWER_6].as<bool>();
//             mbIP.Coil(6, power[6]);
//             sendTelemertry(POWER_6, power[6]);
//         }
//         if (doc["params"].containsKey(POWER_7))
//         {
//             power[7] = doc["params"][POWER_7].as<bool>();
//             mbIP.Coil(7, power[7]);
//             sendTelemertry(POWER_7, power[7]);
//         }
//         if (doc["params"].containsKey(POWER_8))
//         {
//             power[8] = doc["params"][POWER_8].as<bool>();
//             mbIP.Coil(8, power[8]);
//             sendTelemertry(POWER_8, power[8]);
//         }
//         if (doc["params"].containsKey(POWER_9))
//         {
//             power[9] = doc["params"][POWER_9].as<bool>();
//             mbIP.Coil(9, power[9]);
//             sendTelemertry(POWER_9, power[9]);
//         }
//         if (doc["params"].containsKey(POWER_10))
//         {
//             power[10] = doc["params"][POWER_10].as<bool>();
//             mbIP.Coil(10, power[10]);
//             sendTelemertry(POWER_10, power[10]);
//         }
//         if (doc["params"].containsKey(POWER_11))
//         {
//             power[11] = doc["params"][POWER_11].as<bool>();
//             mbIP.Coil(11, power[11]);
//             sendTelemertry(POWER_11, power[11]);
//         }
//         if (doc["params"].containsKey(POWER_12))
//         {
//             power[12] = doc["params"][POWER_12].as<bool>();
//             mbIP.Coil(12, power[12]);
//             sendTelemertry(POWER_12, power[12]);
//         }
//         if (doc["params"].containsKey(POWER_13))
//         {
//             power[13] = doc["params"][POWER_13].as<bool>();
//             mbIP.Coil(13, power[13]);
//             sendTelemertry(POWER_13, power[13]);
//         }
//         if (doc["params"].containsKey(POWER_14))
//         {
//             power[14] = doc["params"][POWER_14].as<bool>();
//             mbIP.Coil(14, power[14]);
//             sendTelemertry(POWER_14, power[14]);
//         }
//         if (doc["params"].containsKey(POWER_15))
//         {
//             power[15] = doc["params"][POWER_15].as<bool>();
//             mbIP.Coil(15, power[15]);
//             sendTelemertry(POWER_15, power[15]);
//         }
//         if (doc["params"].containsKey("power_16"))
//         {
//             rle[1].pre_state = doc["params"]["power_16"].as<bool>();
//             sendTelemertry("power_16", rle[1].pre_state);
//             digitalWrite(RL1, rle[1].pre_state);
//         }
//         if (doc["params"].containsKey("power_17"))
//         {
//             rle[2].pre_state = doc["params"]["power_17"].as<bool>();
//             sendTelemertry("power_17", rle[2].pre_state);
//             digitalWrite(RL2, rle[2].pre_state);
//         }
//         if (doc["params"].containsKey("power_18"))
//         {
//             rle[3].pre_state = doc["params"]["power_18"].as<bool>();
//             sendTelemertry("power_18", rle[3].pre_state);
//             digitalWrite(RL3, rle[3].pre_state);
//         }
//         if (doc["params"].containsKey("power_19"))
//         {
//             rle[4].pre_state = doc["params"]["power_19"].as<bool>();
//             sendTelemertry("power_19", rle[4].pre_state);
//             digitalWrite(RL4, rle[4].pre_state);
//         }

//         saveLocalStorage();
//     }
//     else if (strstr((char *)payload, "update_schedule") != NULL)
//     {
//         jsonObjectTimer(httpGETRequest(serverName.c_str()));
//     }
//     String responseTopic = String(topic);
//     responseTopic.replace("request", "response");
//     Serial.println(responseTopic.c_str());
// }
// void mqtt_loop()
// {
//     static unsigned long lastTime;
//     if (!client.connected())
//     {
//         mqtt_reconnect();
//     }
//     if (millis() - lastTime > 10000)
//     {
//         sendTelemertry(EC, ec);
//         sendTelemertry(Flow1, flow1);
//         sendTelemertry(Flow2, flow2);
//         sendTelemertry(Flow3, flow3);
//         sendTelemertry(Flow4, flow4);
//         lastTime = millis();
//     }
//     client.loop();
// }
// void mqtt_reconnect()
// {
//     if ((!client.connected()))
//     {
//         Serial.println("Connecting to thingsboard...");
//         if (client.connect(ID, TOKEN, NULL))
//         {
//             Serial.println("Connected");
//             client.subscribe("v1/devices/me/rpc/request/+");
//             // client.subscribe("v1/devices/me/attributes/request/+");
//             client.subscribe("v1/devices/me/attributes");
//             // mqtt_sendTelemertry()
//             jsonObjectTimer(httpGETRequest(serverName.c_str()));
//             sendTelemertry(KEY_SET_TIME_A, rle[1].time_set);
//             sendTelemertry(KEY_SET_TIME_B, rle[2].time_set);
//             sendTelemertry(KEY_SET_TIME_C, rle[3].time_set);
//             sendTelemertry(KEY_SET_TIME_D, rle[4].time_set);

//             sendTelemertry(KEY_POWER_DOSING, state_system);
//             sendTelemertry(POWER_0, power[0]);
//             sendTelemertry(POWER_1, power[1]);
//             sendTelemertry(POWER_2, power[2]);
//             sendTelemertry(POWER_3, power[3]);
//             sendTelemertry(POWER_4, power[4]);
//             sendTelemertry(POWER_5, power[5]);
//             sendTelemertry(POWER_6, power[6]);
//             sendTelemertry(POWER_7, power[7]);
//             sendTelemertry(POWER_8, power[8]);
//             sendTelemertry(POWER_9, power[9]);
//             sendTelemertry(POWER_10, power[10]);
//             sendTelemertry(POWER_11, power[11]);
//             sendTelemertry(POWER_12, power[12]);
//             sendTelemertry(POWER_13, power[13]);
//             sendTelemertry(POWER_14, power[14]);
//             sendTelemertry(POWER_15, power[15]);
//             sendTelemertry(POWER_16, rle[1].pre_state);
//             sendTelemertry(POWER_17, rle[2].pre_state);
//             sendTelemertry(POWER_18, rle[3].pre_state);
//             sendTelemertry(POWER_19, rle[4].pre_state);
//             // sendTelemertry(_POWER_17, power[16]);
//             //   mqtt_sendTelemertry();
//         }
//         else
//         {
//             Serial.println("Connect fail");
//             Serial.println(client.state());

//             //  delay(2000);
//         }
//     }
// }
// void sendTelemertry(String key, bool value)
// {
//     DynamicJsonDocument data(200);
//     value == true ? data[key] = true : data[key] = false;
//     String objectString;
//     serializeJson(data, objectString);
//     client.publish("v1/devices/me/telemetry", objectString.c_str());
//     Serial.println(objectString);
// }
// void sendTelemertry(String key, String value)
// {
//     DynamicJsonDocument data(200);
//     data[key] = value;
//     String objectString;
//     serializeJson(data, objectString);
//     client.publish("v1/devices/me/telemetry", objectString.c_str());
//     Serial.println(objectString);
// }
// void sendTelemertry(String key, float value)
// {
//     DynamicJsonDocument data(200);
//     data[key] = value;
//     String objectString;
//     serializeJson(data, objectString);
//     client.publish("v1/devices/me/telemetry", objectString.c_str());
//     Serial.println(objectString);
// }
// void sendTelemertry(String key, int value)
// {
//     DynamicJsonDocument data(200);
//     data[key] = value;
//     String objectString;
//     serializeJson(data, objectString);
//     client.publish("v1/devices/me/telemetry", objectString.c_str());
//     Serial.println(objectString);
// }
// ////////////////////////////////////////////////////////////////////
// void saveLocalStorage()
// {
//     DynamicJsonDocument data(1024);
//     // for (int i = 0; i < 16; i++)
//     // {
//     //     power[i] == true ? data["power_" + (String)(i)] = true : data["power_" + (String)(i)] = false;
//     // }
//     data[KEY_SET_TIME_A] = rle[1].time_set;
//     data[KEY_SET_TIME_B] = rle[2].time_set;
//     data[KEY_SET_TIME_C] = rle[3].time_set;
//     data[KEY_SET_TIME_D] = rle[4].time_set;
//     data[KEY_TIME_TUOI_PHAN] = time_tuoi_phan;

//     power[0] == true ? data["power_0"] = true : data["power_0"] = false;
//     power[1] == true ? data["power_1"] = true : data["power_1"] = false;
//     power[2] == true ? data["power_2"] = true : data["power_2"] = false;
//     power[3] == true ? data["power_3"] = true : data["power_3"] = false;
//     power[4] == true ? data["power_4"] = true : data["power_4"] = false;
//     power[5] == true ? data["power_5"] = true : data["power_5"] = false;
//     power[6] == true ? data["power_6"] = true : data["power_6"] = false;
//     power[7] == true ? data["power_7"] = true : data["power_7"] = false;
//     power[8] == true ? data["power_8"] = true : data["power_8"] = false;
//     power[9] == true ? data["power_9"] = true : data["power_9"] = false;
//     power[10] == true ? data["power_10"] = true : data["power_10"] = false;
//     power[11] == true ? data["power_11"] = true : data["power_11"] = false;
//     power[12] == true ? data["power_12"] = true : data["power_12"] = false;
//     power[13] == true ? data["power_13"] = true : data["power_13"] = false;
//     power[14] == true ? data["power_14"] = true : data["power_14"] = false;
//     power[15] == true ? data["power_15"] = true : data["power_15"] = false;
//     rle[1].pre_state == true ? data["power_1"] = true : data["power_1"] = false;
//     rle[2].pre_state == true ? data["power_2"] = true : data["power_2"] = false;
//     rle[3].pre_state == true ? data["power_3"] = true : data["power_3"] = false;
//     rle[4].pre_state == true ? data["power_4"] = true : data["power_4"] = false;

//     String objectString;
//     serializeJson(data, objectString);
//     writeFile(LITTLEFS, "/data/localStorage.json", objectString.c_str());
//     Serial.println(objectString);
// }
// void localStorageExport()
// {
//     String data = readFile(LITTLEFS, "/data/localStorage.json");
//     Serial.println("READ FILE: " + (String)data);
//     StaticJsonDocument<2048> doc;
//     DeserializationError error = deserializeJson(doc, data.c_str());
//     if (error)
//     {
//         Serial.println("deserializeJson failed");
//         Serial.println(error.f_str());
//         return;
//     }
//     else
//     {
//         rle[1].time_set = doc[KEY_SET_TIME_A];
//         rle[2].time_set = doc[KEY_SET_TIME_B];
//         rle[3].time_set = doc[KEY_SET_TIME_C];
//         rle[4].time_set = doc[KEY_SET_TIME_D];

//         time_tuoi_phan = doc[KEY_TIME_TUOI_PHAN].as<int>();

//         power[0] = doc["power_0"];
//         power[1] = doc["power_1"];
//         power[2] = doc["power_2"];
//         power[3] = doc["power_3"];
//         power[4] = doc["power_4"];
//         power[5] = doc["power_5"];
//         power[6] = doc["power_6"];
//         power[7] = doc["power_7"];
//         power[8] = doc["power_8"];
//         power[9] = doc["power_9"];
//         power[10] = doc["power_10"];
//         power[11] = doc["power_11"];
//         power[12] = doc["power_12"];
//         power[13] = doc["power_13"];
//         power[14] = doc["power_14"];
//         power[15] = doc["power_15"];
//         rle[1].pre_state = doc["power_16"];
//         rle[2].pre_state = doc["power_17"];
//         rle[3].pre_state = doc["power_18"];
//         rle[4].pre_state = doc["power_19"];

//         Serial.println("Export data success!!");
//     }
// }
// void fileInit()
// {
//     if (!LITTLEFS.begin(true))
//     {
//         Serial.println("LITTLEFS Mount Failed");
//         return;
//     }
// }
// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Writing file: %s\r\n", path);

//     File file = fs.open(path, FILE_WRITE);
//     if (!file)
//     {
//         Serial.println("- failed to open file for writing");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("- file written");
//     }
//     else
//     {
//         Serial.println("- write failed");
//     }
//     file.close();
// }
// String readFile(fs::FS &fs, const char *path)
// {
//     String data = "";
//     Serial.printf("Reading file: %s\r\n", path);

//     File file = fs.open(path);
//     if (!file || file.isDirectory())
//     {
//         Serial.println("- failed to open file for reading");
//     }

//     Serial.println("- read from file:");
//     while (file.available())
//     {
//         data += (char)file.read();
//     }
//     file.close();
//     return data;
// }
// void createDir(fs::FS &fs, const char *path)
// {
//     Serial.printf("Creating Dir: %s\n", path);
//     if (fs.mkdir(path))
//     {
//         Serial.println("Dir created");
//     }
//     else
//     {
//         Serial.println("mkdir failed");
//     }
// }
// //////////////////////////////////////////////////////////
// String httpGETRequest(const char *serverName)
// {
//     String payload;
//     WiFiClientSecure client;
//     client.setInsecure();
//     HTTPClient http;

//     String serverPath = serverName + (String)TOKEN;

//     // Your Domain name with URL path or IP address with path
//     http.begin(client, serverPath.c_str());

//     // Send HTTP GET request
//     int httpResponseCode = http.GET();

//     if (httpResponseCode > 0)
//     {
//         Serial.print("HTTP Response code: ");
//         Serial.println(httpResponseCode);
//         payload = http.getString();
//         Serial.println(payload);
//         writeFile(LITTLEFS, "/timer.josn", payload.c_str());
//     }
//     else
//     {
//         Serial.print("Error code: ");
//         Serial.println(httpResponseCode);
//     }
//     // Free resources
//     http.end();

//     return payload;
// }
// void timeInit()
// {
//     timeClient.begin();
//     //  timeClient.setTimeOffset(+7*60*60);
// }
// void jsonObjectTimer(String jsonString)
// {

//     Serial.println(jsonString);
//     StaticJsonDocument<2048> doc;
//     deserializeJson(doc, jsonString);
//     JsonArray array = doc["data"].as<JsonArray>();
//     number_of_timers = array.size();
//     for (int i = 0; i < number_of_timers; i++)
//     {
//         TIMER[i].id = doc["data"][i]["id"].as<int>();
//         TIMER[i].enable = doc["data"][i]["enable"].as<bool>();
//         TIMER[i].interval = doc["data"][i]["interval"].as<String>();
//         TIMER[i].time = doc["data"][i]["time"].as<String>();
//         TIMER[i].keyName[0] = "";
//         TIMER[i].keyName[1] = "";
//         TIMER[i].keyName[2] = "";
//         TIMER[i].keyName[3] = "";
//         TIMER[i].keyName[4] = "";
//         TIMER[i].keyName[5] = "";
//         for (int k = 0; k < 30; k++)
//         {
//             TIMER[i].keyName[k + 6] = "";
//         }

//         if (doc["data"][i]["action"].containsKey("set_power_dosing"))
//         {
//             String a = doc["data"][i]["action"]["set_power_dosing"];
//             TIMER[i].keyName[0] = "set_power_dosing";
//             a == "true" ? TIMER[i].state[0] = true : TIMER[i].state[0] = false;
//         }
//         if (doc["data"][i]["action"].containsKey("time_tuoi_phan"))
//         {
//             String a = doc["data"][i]["action"]["time_tuoi_phan"];
//             TIMER[i].keyName[1] = "time_tuoi_phan";
//             TIMER[i].value[0] = a.toInt();
//         }
//         if (doc["data"][i]["action"].containsKey("set_time_a"))
//         {
//             String a = doc["data"][i]["action"]["set_time_a"];
//             TIMER[i].keyName[2] = "set_time_a";
//             TIMER[i].value[1] = a.toInt();
//         }
//         if (doc["data"][i]["action"].containsKey("set_time_b"))
//         {
//             String a = doc["data"][i]["action"]["set_time_b"];
//             TIMER[i].keyName[3] = "set_time_b";
//             TIMER[i].value[2] = a.toInt();
//         }
//         if (doc["data"][i]["action"].containsKey("set_time_c"))
//         {
//             String a = doc["data"][i]["action"]["set_time_c"];
//             TIMER[i].keyName[4] = "set_time_c";
//             TIMER[i].value[3] = a.toInt();
//         }
//         if (doc["data"][i]["action"].containsKey("set_time_d"))
//         {
//             String a = doc["data"][i]["action"]["set_time_d"];
//             TIMER[i].keyName[5] = "set_time_d";
//             TIMER[i].value[4] = a.toInt();
//         }

//         for (int k = 0; k < 30; k++)
//         {
//             String tempKeyName = "power_" + (String)(k);
//             if (doc["data"][i]["action"].containsKey(tempKeyName))
//             {
//                 String a = doc["data"][i]["action"][tempKeyName];
//                 TIMER[i].keyName[k] = tempKeyName;
//                 a == "true" ? TIMER[i].state[k] = true : TIMER[i].state[k] = false;
//             }
//         }
//     }
// }
// void timerInit()
// {
//     Serial.println("Read data timer");
//     String data_timer = readFile(LITTLEFS, "/timer.json");
//     jsonObjectTimer(data_timer.c_str());
//     for (int i = 0; i < number_of_timers; i++)
//     {
//         Serial.println(TIMER[i].id);
//     }
// }
// void printLocalTime()
// {
//     timeClient.update();
//     time_t epochTime = timeClient.getEpochTime();
//     struct tm *timeInfo = gmtime((time_t *)&epochTime);
//     int monthDay = timeInfo->tm_mday;
//     Serial.print("Month day: ");
//     Serial.println(monthDay);

//     int currentMonth = timeInfo->tm_mon + 1;
//     Serial.print("Month: ");
//     Serial.println(currentMonth);

//     String currentMonthName = months[currentMonth - 1];
//     Serial.print("Month name: ");
//     Serial.println(currentMonthName);

//     int currentYear = timeInfo->tm_year + 1900;
//     Serial.print("Year: ");
//     Serial.println(currentYear);

//     // Print complete date:
//     String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
//     Serial.print("Current date: ");
//     Serial.println(currentDate);

//     Serial.println("");
// }
// void NTPTimeUpdate()
// {
//     static int nowMM = 100;
//     timeClient.update();
//     time_t epochTime = timeClient.getEpochTime();
//     struct tm *timeinfo = gmtime((time_t *)&epochTime);
//     String hour;
//     timeinfo->tm_hour > 9 ? hour = (String)timeinfo->tm_hour : hour = "0" + (String)timeinfo->tm_hour;
//     String _time;
//     timeinfo->tm_min > 9 ? _time = hour + ":" + (String)timeinfo->tm_min : _time = hour + ":0" + (String)timeinfo->tm_min;
//     String _dayOfWeek = (String)timeinfo->tm_wday;
//     //  Serial.print("time: ");
//     //     Serial.println(array);

//     for (int i = 0; i < number_of_timers; i++)
//     {

//         if ((TIMER[i].time == _time) && (strstr(TIMER[i].interval.c_str(), _dayOfWeek.c_str())))
//         {
//             if (nowMM != timeinfo->tm_min)
//             {
//                 nowMM = timeinfo->tm_min;
//                 if (TIMER[i].keyName[0] == "set_power_dosing")
//                 {
//                     // auto_by = TIMER[i].state[0];
//                     state_system = TIMER[i].state[0];
//                     state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
//                 }

//                 if (TIMER[i].keyName[1] == "time_tuoi_phan")
//                 {
//                     time_tuoi_phan = TIMER[i].value[0];
//                 }

//                 if (TIMER[i].keyName[2] == "set_time_a")
//                 {
//                     rle[1].time_set = TIMER[i].value[1];
//                 }
//                 if (TIMER[i].keyName[3] == "set_time_b")
//                 {
//                     rle[2].time_set = TIMER[i].value[2];
//                 }
//                 if (TIMER[i].keyName[4] == "set_time_c")
//                 {
//                     rle[3].time_set = TIMER[i].value[3];
//                 }
//                 if (TIMER[i].keyName[5] == "set_time_d")
//                 {
//                     rle[4].time_set = TIMER[i].value[4];
//                 }
//                 ///////////////////////////////////////////////////////////////

//                 for (int k = 0; k < 30; k++)
//                 {
//                     String keyName = "power_" + (String)(k);
//                     if (TIMER[i].keyName[k] == keyName)
//                     {
//                         // digitalWrite(coils_[k], TIMER[i].state[k]);
//                         // rl[k] = TIMER[i].state[k];
//                         // digitalWrite(RL[k], rl[k]);
//                         power[k] = TIMER[i].state[k];
//                         Serial.println("STATE RL: " + (String)k + "state: " + (String)TIMER[i].state[0]);
//                         sendTelemertry("power_0", power[0]);
//                         sendTelemertry("power_1", power[1]);
//                         sendTelemertry("power_2", power[2]);
//                         sendTelemertry("power_3", power[3]);
//                         sendTelemertry("power_4", power[4]);
//                         sendTelemertry("power_5", power[5]);
//                         sendTelemertry("power_6", power[6]);
//                         sendTelemertry("power_7", power[7]);
//                         sendTelemertry("power_8", power[8]);
//                         sendTelemertry("power_9", power[9]);
//                         sendTelemertry("power_10", power[10]);
//                         sendTelemertry("power_11", power[11]);
//                         sendTelemertry("power_12", power[12]);
//                         sendTelemertry("power_13", power[13]);
//                         sendTelemertry("power_14", power[14]);
//                         sendTelemertry("power_15", power[15]);
//                         sendTelemertry("power_16", rle[1].pre_state);
//                         sendTelemertry("power_17", rle[2].pre_state);
//                         sendTelemertry("power_18", rle[3].pre_state);
//                         sendTelemertry("power_19", rle[4].pre_state);
//                     }
//                 }
//             }
//         }
//         else
//         {
//         }
//     }
// }
// void jsonParseToStruct(String jsonString, TSchedule *m_schedule)
// {
//     Serial.println("============================");
//     Serial.println(jsonString);
//     StaticJsonDocument<4096> doc;
//     deserializeJson(doc, jsonString);
//     JsonArray array = doc["data"].as<JsonArray>();
//     _numOfSchedule = array.size();
//     Serial.println("size schedule: " + (String)_numOfSchedule);
//     for (int i = 0; i < _numOfSchedule; i++)
//     {
//         JsonObject actionObject = doc["data"][i]["action"].as<JsonObject>();
//         int num_of_object = actionObject.size();
//         m_schedule[i].action_count = num_of_object;
//         m_schedule[i].enable = doc["data"][i]["enable"].as<bool>();
//         m_schedule[i].interval = doc["data"][i]["interval"].as<String>();
//         m_schedule[i].time = doc["data"][i]["time"].as<String>();
//         int j = 0;
//         for (JsonPair kv : actionObject)
//         {
//             m_schedule[i].action[j].key = kv.key().c_str();
//             m_schedule[i].action[j].value = kv.value().as<String>();
//             j++;
//         }
//     }
// }
// void updateSchedule()
// {

//     timeClient.update();
//     TSchedule m_schedule;
//     time_t epochTime = timeClient.getEpochTime();
//     struct tm *timeinfo = gmtime((time_t *)&epochTime);
//     static int nowMM = 100;

//     String hour;
//     timeinfo->tm_hour > 9 ? hour = (String)timeinfo->tm_hour : hour = "0" + (String)timeinfo->tm_hour;
//     String _time;
//     timeinfo->tm_min > 9 ? _time = hour + ":" + (String)timeinfo->tm_min : _time = hour + ":0" + (String)timeinfo->tm_min;
//     String _dayOfWeek = (String)timeinfo->tm_wday;
//     int _mm = timeinfo->tm_min;

//     for (int i = 0; i < _numOfSchedule; i++) // Quét từng object hẹn giờ
//     {
//         m_schedule = _mSchedule[i];
//         if ((m_schedule.time == _time && m_schedule.enable == true) && (nowMM != _mm && strstr(m_schedule.interval.c_str(), _dayOfWeek.c_str())))
//         {
//             Serial.println("enter hen gio");
//             nowMM = timeinfo->tm_min;
//             for (int j = 0; j < m_schedule.action_count; j++)
//             {
//                 // van tuoi
//                 for (int k = 0; k < 5; k++)
//                 {
//                     String key = "power_" + (String)(k);
//                     if (m_schedule.action[j].key == key)
//                     {
//                         bool state = false;
//                         m_schedule.action[j].value == "true" ? state = true : state = false;
//                         rle[k].pre_state = m_schedule.action[j].value;
//                         Serial.println("key: " + (String)m_schedule.action[j].key);
//                         Serial.println("val: " + (String)m_schedule.action[j].value);
//                     }
//                 }
//                 // cai nay la van tuoi + bơm
//                 for (int k = 0; k < 16; k++)
//                 {
//                     String key = "power_" + (String)(k);
//                     if (m_schedule.action[j].key == key)
//                     {
//                         bool state = false;
//                         m_schedule.action[j].value == "true" ? state = true : state = false;
//                         power[k] = m_schedule.action[j].value;
//                         Serial.println("key: " + (String)m_schedule.action[j].key);
//                         Serial.println("val: " + (String)m_schedule.action[j].value);
//                     }
//                 }
//                 // cai nay la he thong
//                 if (m_schedule.action[j].key == "set_power_dosing")
//                 {
//                     bool state = false;
//                     // auto_by = TIMER[i].state[0];
//                     m_schedule.action[j].value == "true" ? state = true : state = false;
//                     state_system = m_schedule.action[j].value;
//                     state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
//                 }
//                 // cai nay la thoi gian tuoi phan

//                 if (m_schedule.action[j].key == "time_tuoi_phan")
//                 {
//                     time_tuoi_phan = m_schedule.action[j].value.toInt();
//                     Serial.println("key: " + (String)m_schedule.action[j].key);
//                     Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
//                 }
//                 // cai nay la thoi gian van A

//                 if (m_schedule.action[j].key == "set_time_a")
//                 {
//                     rle[0].time_set = m_schedule.action[j].value.toInt();
//                     Serial.println("key: " + (String)m_schedule.action[j].key);
//                     Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
//                 }
//                 // cai nay la thoi gian van B

//                 if (m_schedule.action[j].key == "set_time_b")
//                 {
//                     rle[1].time_set = m_schedule.action[j].value.toInt();
//                     Serial.println("key: " + (String)m_schedule.action[j].key);
//                     Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
//                 }
//                 // cai nay la thoi gian van c

//                 if (m_schedule.action[j].key == "set_time_c")
//                 {
//                     rle[2].time_set = m_schedule.action[j].value.toInt();
//                     Serial.println("key: " + (String)m_schedule.action[j].key);
//                     Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
//                 }
//                 // cai nay la thoi gian van d

//                 if (m_schedule.action[j].key == "set_time_d")
//                 {
//                     rle[3].time_set = m_schedule.action[j].value.toInt();
//                     Serial.println("key: " + (String)m_schedule.action[j].key);
//                     Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
//                 }
//             }
//         }
//     }
// }