
// #define CONTROL_RELAY(__X__, __Y__) RELAY.digitalWrite(__X__, __Y__)
// #define READ_RELAY(__X__) RELAY.digitalRead(__X__)
#define PATH_CONFIG "/config.json"
#define PATH_STORAGE_STATE "/localStorage.json"
#define PATH_TIMER "/timer.json"
#define PATH_FIRM_INFOR "/firmware_infor.json"
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define HTTP_SEVER "iot.viis.tech"
#define HTTP_PORT 443
// #define TOKEN "GCtItDk2cYNDaGnzGQYi"
// #define ID "43f99e80-a861-11ed-86eb-ad5a639611e9"
#define TOKEN device_config.token
#define ID device_config.id

// #define OPEN true
// #define CLOSE false

// #define BY_SENSOR false
// #define BY_SCHEDULE true

// #define AUTO true
// #define MANUAL false
// #define TIMEOUT_REM 10000
// #define MAX_OUTPUT 14
// #define TIMEOUT 60000 * 2

#include "ETH.h"
#include "Arduino.h"
// #include "PCF8575.h"
#include "../include/ML_16IO/IO_define.h"
#include "HardwareSerial.h"
#include "ModbusRTU.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include <HTTPClient.h>
#include <SPIFFS.h>
#include "ModbusIP_ESP8266.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "NTPClient.h"
#include "WiFiUdp.h"
#include "Update.h"

// bool state_rem_1,
//     state_rem_2 = false;
// typedef enum
// {
//     IDLE_STATE,
//     CLOSE_REM_1,
//     CLOSE_REM_2,
//     OPEN_REM_1,
//     OPEN_REM_2
// } state;
// state LOGIC_STATE;

AsyncWebServer server(80);

// HardwareSerial RS485(1);
// TwoWire I2Cone = TwoWire(0);
// TwoWire I2Ctwo = TwoWire(1);
// PCF8575 RELAY(&I2Ctwo, ADDR_RL, SDA, SCL);

IPAddress local_ip(192, 168, 1, 20);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (8, 8, 4, 4);

// bool auto_by = BY_SENSOR;
// bool mode = MANUAL;
struct timer
{
    int id;
    bool enable;
    String time;
    String interval;
    String keyName[6];
    bool state[6];
};
timer TIMER[50];
// struct sensor_set
// {
//     float set_temp_high = 0;
//     float set_temp_low = 0;
//     float set_humi_high = 0;
//     float set_humi_low = 0;
//     int set_lux_high = 0;
//     int set_lux_low = 0;
//     int set_lux_high_2 = 0;
//     int set_lux_low_2 = 0;
// } value_set_sensor;

// struct sensor_value
// {
//     float temp;
//     float humi;
//     float lux;
// } sensor;
/*HTTP API*/
#include <HttpClient.h>
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
String httpGETRequest(const char *serverName);

/*OTA API*/
struct firmware
{
    String firmInfor = "/api/v1/QFhCS8a76zYjIJVkoXuU/attributes";
    String fw_title;
    String fw_version;
    String bin;
    String fw_tag;

} mqtt_firmware;

long contentLength = 0;
bool isValidContentType = false;

typedef enum
{
    NEW_FIRM_AVAILABLE,
    NO_NEW_FIRM_AVAILABLE,
    ERROR
} OTA_status_check_t;
OTA_status_check_t checkFirmwareVersionHttp();
OTA_status_check_t checkFirmwareVersion();
String getHeaderValue(String header, String headerName);
void executeOTA();
void saveFirmware_infor();
/*API NTP*/

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
uint8_t number_of_timers = 0;
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
void NTPInit();
void NTPTimeUpdate();
void printLocalTime();
void jsonObjectTimer(String jsonString);
/*DEVICE API*/
struct config
{
    String token;
    String id;
    String local_ip;
    String gateway;
    String subnet;
    String dns1;
    String dns2;
    bool dhcp;
} device_config;

void exportDeviceConfig();
void executeOTA();
void saveFirmware_infor();
// void open_rem_1();
// void close_rem_1();
// void open_rem_2();
// void close_rem_2();
/*Logic SensornAPI*/
// void logic_sensor_humi();
// void logic_sensor_temp();
// void logic_sensor_lux();
/*ModbusIP API*/
// ModbusIP mbIP;
// // ModbusRTU mb1;
// void ModbusIpInit();
// void ModbusIpLoop();

/*Storge API*/
void saveLocalStoreSate();
void exportLocalStoreSate();
/*FILE API*/
void fileInit();
void writeFile(fs::FS &fs, const char *path, const char *message);
String readFile(fs::FS &fs, const char *path);
void createDir(fs::FS &fs, const char *path);
/*MQTT API*/
WiFiClient espClient;
PubSubClient client(espClient);
void mqttInit();
void on_message(const char *topic, byte *payload, unsigned int length);
void mqttLoop();
void mqttReconnect();
void mqttSendLog(String log);
void sendTelemertry(String key, bool value);
void sendTelemertry(String key, String value);
void sendTelemertry(String key, float value);
void sendTelemertry(String key, int value);
/*APP API*/
void appInit();
void appRun();

bool eth_connected = false;
void WiFiEvent(WiFiEvent_t event);

bool keyPressed = false;

/*Convert API*/

// uint32_t floatMidLitteEndianCDAB(uint16_t AB, uint16_t CD);
// float uint32ToFloat(uint32_t x) ;

void appInit()
{
    // Serial.print("nè");

    // for (int i = 0; i < 16; i++)
    // {
    //     RELAY.pinMode(i, OUTPUT);
    // }
    // RELAY.begin();
    Serial.begin(115200);
    // RS485.begin(9600, SERIAL_8N1, RS485_TX, RS485_RX);
    // mb1.begin(&RS485);
    // mb1.slave(1);

    fileInit();
    exportDeviceConfig();
    exportLocalStoreSate();
    WiFi.onEvent(WiFiEvent);
    // mach gateway //

    pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
    digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);

    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
    if (device_config.dhcp)
    {
        Serial.println("DHCP true");
        local_ip.fromString(device_config.local_ip);
        gateway.fromString(device_config.gateway);
        subnet.fromString(device_config.subnet);
        dns1.fromString(device_config.dns1);
        dns2.fromString(device_config.dns2);
        ETH.config(local_ip, gateway, subnet, dns1, dns2);
    }
    //       WiFi.begin("DalatTech", "mltech@2019");
    //   Serial.print("Connecting to WiFi ..");
    //   while (WiFi.status() != WL_CONNECTED) {
    //     Serial.print('.');
    //     delay(1000);
    //   }
    //   Serial.println(WiFi.localIP());
    // ModbusIpInit();
    NTPInit();
    mqttInit();
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Bộ điều khiển vườn LAN"); });

    AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
    server.begin();
}

// uint32_t floatMidLitteEndianCDAB(uint16_t AB, uint16_t CD) // cái này là ghép 2 thanh ghi 16 thành 32
// {
//   uint32_t CDAB = AB | (CD << 16);
//   return CDAB;
// }

// float uint32ToFloat(uint32_t x) //hàm này là chuyển uint32 ssang float (unti32 là 8byte)
// {
//   float y;
//   memcpy((uint8_t *)&y,(uint8_t *)&x,4);
//   return y;
// }

void appRun()
{
    Serial.print("nè1");
    // sensor.temp = (float)mbIP.Hreg(0) / 10.0;
    // sensor.humi = (float)mbIP.Hreg(1) / 10.0;
    // uint32_t lux_32t=floatMidLitteEndianCDAB((uint16_t)mbIP.Hreg(2),(uint16_t)mbIP.Hreg(3));
    // sensor.lux=uint32ToFloat(lux_32t);
    // Serial.println("LUX: "+(String)sensor.lux);
    // // Serial.println(sensor.temp);
    // // Serial.println(sensor.humi);
    // Serial.println(mbIP.Hreg(2),HEX);
    // Serial.println(mbIP.Hreg(3),HEX);
    // delay(1000);
    //sensor.lux = mbIP.Hreg(22);
    // if (mode == AUTO && auto_by == BY_SENSOR)
    // {
    //     // Serial.println("BY SENSOR");
    //     logic_sensor_lux();
    // }
    // else
    // {
    //     LOGIC_STATE = IDLE_STATE;
    // }
    NTPTimeUpdate();
    if (eth_connected || WiFi.isConnected())
    {
        mqttLoop();
        // printLocalTime();
    }
    // ModbusIpLoop();
}

void fileInit()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
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
        //    createDir(SPIFFS, "/data");
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

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case SYSTEM_EVENT_ETH_START:
        Serial.println("ETH Started");
        // set eth hostname here
        ETH.setHostname("esp32-ethernet");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        Serial.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.print(ETH.localIP());
        if (ETH.fullDuplex())
        {
            Serial.print(", FULL_DUPLEX");
        }
        Serial.print(", ");
        Serial.print(ETH.linkSpeed());
        Serial.println("Mbps");
        eth_connected = true;
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        eth_connected = false;
        break;
    case SYSTEM_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        eth_connected = false;
        break;
    default:
        break;
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
        if (strstr((char *)payload, "set_state") != NULL)
        {

            // for (int i = 2; i < 14; i++)
            // {
            //     String key = "power_" + (String)i;
            //     if (doc["params"].containsKey(key.c_str()))
            //     {
            //         bool state_relay = doc["params"][key];
            //         CONTROL_RELAY(i + 2, state_relay);
            //         sendTelemertry(key, (bool)(READ_RELAY(i + 2)));
            //         mbIP.Coil(i, state_relay);
            //     }
            // }
            // if (doc["params"].containsKey("power_0"))
            // {
            //     bool state_relay = doc["params"]["power_0"];
            //     mbIP.Coil(0, state_relay);
            //     if (state_relay)
            //     {
            //         open_rem_1();
            //     }
            //     else
            //     {
            //         close_rem_1();
            //     }
            //     sendTelemertry("power_0", state_rem_1);
            // }
            // if (doc["params"].containsKey("power_1"))
            // {
            //     bool state_relay = doc["params"]["power_1"];
            //     mbIP.Coil(1, state_relay);
            //     if (state_relay)
            //     {
            //         open_rem_2();
            //     }
            //     else
            //     {
            //         close_rem_2();
            //     }
            //     sendTelemertry("power_1", state_rem_2);
            // }
            // if (doc["params"].containsKey("mode"))
            // {
            //     bool state_mode = doc["params"]["mode"];
            //     mode = state_mode;
            //     sendTelemertry("mode", mode);
            // }
            // if (doc["params"].containsKey("set_lux_high"))
            // {
            //     int value = doc["params"]["set_lux_high"].as<int>();
            //     value_set_sensor.set_lux_high = value;
            //     sendTelemertry("set_lux_high", value_set_sensor.set_lux_high);
            //    // mbIP.Hreg(3, value_set_sensor.set_lux_high);
            // }
            // if (doc["params"].containsKey("set_lux_low"))
            // {
            //     int value = doc["params"]["set_lux_low"].as<int>();
            //     value_set_sensor.set_lux_low = value;
            //     sendTelemertry("set_lux_low", value_set_sensor.set_lux_low);
            //   //  mbIP.Hreg(4, value_set_sensor.set_lux_low);
            // }

            // if (doc["params"].containsKey("set_lux_high_2"))
            // {
            //     int value = doc["params"]["set_lux_high_2"].as<int>();
            //     value_set_sensor.set_lux_high_2 = value;
            //     sendTelemertry("set_lux_high_2", value_set_sensor.set_lux_high_2);
            //  //   mbIP.Hreg(5, value_set_sensor.set_lux_high_2);
            // }
            // if (doc["params"].containsKey("set_lux_low_2"))
            // {
            //     int value = doc["params"]["set_lux_low_2"].as<int>();
            //     value_set_sensor.set_lux_low_2 = value;
            //     sendTelemertry("set_lux_low_2", value_set_sensor.set_lux_low_2);
            //  //   mbIP.Hreg(6, value_set_sensor.set_lux_low_2);
            // }
            // if (doc["params"].containsKey("auto_by"))
            // {
            //     int value = doc["params"]["auto_by"].as<bool>();
            //     auto_by = value;
            //     sendTelemertry("auto_by", auto_by);
            //   //  mbIP.Hreg(7, auto_by);
            // }

            saveLocalStoreSate();
        }
    }
    else if (strstr((char *)payload, "update_schedule") != NULL)
    {
        jsonObjectTimer(httpGETRequest(serverName.c_str()));
    }
    if (doc.containsKey("fw_title"))
    {
        mqtt_firmware.fw_title = doc["fw_title"].as<String>();
    }
    if (doc.containsKey("fw_title"))
    {
        mqtt_firmware.fw_version = doc["fw_version"].as<String>();
    }
    if (doc.containsKey("fw_tag"))
    {
        mqtt_firmware.fw_tag = doc["fw_tag"].as<String>();
        if (checkFirmwareVersion() == NEW_FIRM_AVAILABLE)
        {
            executeOTA();
        }
    }
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    Serial.println(responseTopic.c_str());
}

void mqttLoop()
{
    static unsigned long lastTime;
    if (!client.connected())
    {
        mqttReconnect();
    }
    if (millis() - lastTime > 10000)
    {
        // sendTelemertry("humi_value", sensor.humi);
        // sendTelemertry("temp_value", sensor.temp);
        // sendTelemertry("lux_value", sensor.lux);
        lastTime = millis();
    }

    client.loop();
}
void mqttReconnect()
{
    static bool check_OTA = false;
    if ((!client.connected()))
    {
        Serial.println("Connecting to thingsboard...");
        if (client.connect(ID.c_str(), TOKEN.c_str(), NULL))
        {
            Serial.println("Connected");
            client.subscribe("v1/devices/me/rpc/request/+");
            client.subscribe("v1/devices/me/attributes");
            // sendTelemertrySysInfo();
            if (!check_OTA)
            {
                jsonObjectTimer(httpGETRequest(serverName.c_str()));
                if (checkFirmwareVersionHttp() == NEW_FIRM_AVAILABLE)
                {
                    executeOTA();
                }
                check_OTA = true;
            }
            sendTelemertry("IP", ETH.localIP().toString());
        }
        else
        {
            Serial.println("Connect fail");
            Serial.println(client.state());
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
void saveLocalStoreSate()
{
    DynamicJsonDocument data(512);

    // for (int i = 2; i < 14; i++)
    // {
    //     String key = "power_" + (String)i;

    //     data[key] = READ_RELAY(i + 2);
    // }
    // data["power_0"] = state_rem_1;
    // data["power_1"] = state_rem_2;

    // data["mode"] = mode;
    // data["auto_by"] = auto_by;
    // data["set_lux_high"] = value_set_sensor.set_lux_high;
    // data["set_lux_low"] = value_set_sensor.set_lux_low;
    // data["set_lux_high_2"] = value_set_sensor.set_lux_high_2;
    // data["set_lux_low_2"] = value_set_sensor.set_lux_low_2;

    String objectString;
    serializeJson(data, objectString);
    writeFile(SPIFFS, PATH_STORAGE_STATE, objectString.c_str());
    Serial.println("SAVE FILE: " + (String)objectString);
}
void exportLocalStoreSate()
{
    String data = readFile(SPIFFS, PATH_STORAGE_STATE);
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
        // for (int i = 2; i < 14; i++)
        // {
        //     String key = "power_" + (String)i;
        //     if (doc.containsKey(key.c_str()))
        //     {
        //         bool state_relay = doc[key];
        //         CONTROL_RELAY(i + 2, state_relay);
        //         mbIP.Coil(i, state_relay);
        //     }
        // }
        // if (doc.containsKey("power_0"))
        // {
        //     bool state_relay = doc["power_0"];
        //     if (state_relay)
        //     {
        //         open_rem_1();
        //     }
        //     else
        //     {
        //         close_rem_1();
        //     }
        // }
        // if (doc.containsKey("power_1"))
        // {
        //     bool state_relay = doc["power_1"];
        //     if (state_relay)
        //     {
        //         open_rem_2();
        //     }
        //     else
        //     {
        //         close_rem_2();
        //     }
        // }
        // mode = doc["mode"];
        // auto_by = doc["auto_by"];
        // value_set_sensor.set_lux_high = doc["set_lux_high"];
        // value_set_sensor.set_lux_low = doc["set_lux_low"];
        // value_set_sensor.set_lux_high_2 = doc["set_lux_high_2"];
        // value_set_sensor.set_lux_low_2 = doc["set_lux_low_2"];

        // mbIP.Hreg(3, value_set_sensor.set_lux_high);
        // mbIP.Hreg(4, value_set_sensor.set_lux_low);
        // mbIP.Hreg(5, value_set_sensor.set_lux_high_2);
        // mbIP.Hreg(6, value_set_sensor.set_lux_low_2);
    }
}
// void ModbusIpInit()
// {
//     mbIP.server();
//     mbIP.addHreg(0, 0, 30);
//     // mbIP.addCoil(0, 0, 16);
// }
// void ModbusIpLoop()
// {
//     mbIP.task();
// }
// void open_rem_1()
// {
//     state_rem_1 = true;
//     CONTROL_RELAY(1, false);
//     CONTROL_RELAY(0, true);
//     Serial.println("rèm 1 mở");
// }
// void close_rem_1()
// {
//     state_rem_1 = false;
//     CONTROL_RELAY(0, false);
//     CONTROL_RELAY(1, true);
//     Serial.println("rèm 1 đóng");
// }
// void open_rem_2()
// {
//     state_rem_2 = true;
//     CONTROL_RELAY(3, false);
//     CONTROL_RELAY(2, true);
//     Serial.println("rèm 2 mở");
// }
// void close_rem_2()
// {
//     state_rem_2 = false;
//     CONTROL_RELAY(2, false);
//     CONTROL_RELAY(3, true);
//     Serial.println("rèm 2 đóng");
// }

// void logic_sensor_lux()
// {
//     static unsigned long preTime = millis();

//     switch (LOGIC_STATE)
//     {
//     case IDLE_STATE:
//     {
//         if (value_set_sensor.set_lux_high < value_set_sensor.set_lux_low)
//         {
//             Serial.println("Invalid value set 1");
//         }
//         else
//         {
//             if (sensor.lux < value_set_sensor.set_lux_low)
//             {
//                 if (state_rem_1 == CLOSE)
//                 {
//                     open_rem_1();
//                     sendTelemertry("power_0", state_rem_1);
//                 }
//             }
//             else if (sensor.lux > value_set_sensor.set_lux_high)
//             {
//                 if (state_rem_1 == OPEN)
//                 {
//                     close_rem_1();
//                     sendTelemertry("power_0", state_rem_1);
//                 }
//             }
//         }
//         if (value_set_sensor.set_lux_high_2 < value_set_sensor.set_lux_low_2)
//         {
//             Serial.println("Invalid value set rem 2");
//         }
//         else
//         {
//             if (sensor.lux < value_set_sensor.set_lux_low_2)
//             {
//                 if (state_rem_2 == CLOSE)
//                 {
//                     open_rem_2();
//                     sendTelemertry("power_1", state_rem_2);
//                 }
//             }
//             else if (sensor.lux > value_set_sensor.set_lux_high_2)
//             {

//                 if (state_rem_2 == OPEN)
//                 {
//                     close_rem_2();
//                     sendTelemertry("power_1", state_rem_2);
//                 }
//             }
//         }
//         preTime = millis();
//         break;
//     }
//     case CLOSE_REM_1:
//     {
//         if (state_rem_1 == OPEN)
//         {
//             close_rem_1();
//             sendTelemertry("power_0", state_rem_1);
//         }
//         if (millis() - preTime > TIMEOUT_REM)
//         {
//             Serial.println("TIME OUT CLOSE REM 1");
//             LOGIC_STATE = IDLE_STATE;
//         }
//         break;
//     }
//     case CLOSE_REM_2:
//     {
//         if (state_rem_2 == OPEN)
//         {
//             close_rem_2();
//             sendTelemertry("power_1", state_rem_2);
//         }
//         if (millis() - preTime > TIMEOUT_REM)
//         {
//             Serial.println("TIME OUT CLOSE REM 2");
//             LOGIC_STATE = IDLE_STATE;
//         }
//         break;
//     }
//     case OPEN_REM_1:
//     {
//         if (state_rem_1 == CLOSE)
//         {
//             open_rem_1();
//             sendTelemertry("power_0", state_rem_1);
//         }
//         if (millis() - preTime > TIMEOUT_REM)
//         {
//             Serial.println("TIME OUT OPEN REM 1");
//             LOGIC_STATE = IDLE_STATE;
//         }
//         break;
//     }
//     case OPEN_REM_2:
//     {
//         if (state_rem_2 == CLOSE)
//         {
//             open_rem_2();
//             sendTelemertry("power_1", state_rem_2);
//         }
//         if (millis() - preTime > TIMEOUT_REM)
//         {
//             Serial.println("TIME OUT OPEN REM 2");
//             LOGIC_STATE = IDLE_STATE;
//         }
//         break;
//     }
//     }
// }
//------
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
        writeFile(SPIFFS, PATH_TIMER, payload.c_str());
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
void jsonObjectTimer(String jsonString)
{

    // Serial.println(jsonString);
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
        if (doc["data"][i]["action"].containsKey("auto_by"))
        {
            String a = doc["data"][i]["action"]["auto_by"];
            TIMER[i].keyName[0] = "auto_by";
            a == "true" ? TIMER[i].state[0] = true : TIMER[i].state[0] = false;
           // Serial.println("keyname: " + (String)TIMER[i].keyName[0] + " TIME: " + (String)TIMER[i].time + " state: " + (String)TIMER[i].state[0]);
        }

        for (int k = 1; k < 3; k++)
        {
            String tempKeyName = "power_" + (String)(k - 1);
            if (doc["data"][i]["action"].containsKey(tempKeyName))
            {
                String a = doc["data"][i]["action"][tempKeyName];
                TIMER[i].keyName[k] = tempKeyName;
                a == "true" ? TIMER[i].state[k] = true : TIMER[i].state[k] = false;
             //   Serial.println("keyname: " + (String)TIMER[i].keyName[k] + " TIME: " + (String)TIMER[i].time + " state: " + (String)TIMER[i].state[k]);
            }
        }
    }
}
void NTPInit()
{
    timeClient.begin();
    Serial.println("Read data timer");
    String data_timer = readFile(SPIFFS, PATH_TIMER);
    Serial.println(data_timer);
    jsonObjectTimer(data_timer.c_str());
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
    // Serial.println(_time);

    for (int i = 0; i < number_of_timers; i++)
    {

        if ((TIMER[i].time == _time) && (strstr(TIMER[i].interval.c_str(), _dayOfWeek.c_str())))
        {
            if (nowMM != timeinfo->tm_min)
            {
                nowMM = timeinfo->tm_min;

                for (int k = 1; k < 3; k++)
                {

                    String keyName = "power_" + (String)(k-1);
                    if (TIMER[i].keyName[0] == "auto_by")
                    {
                        auto_by = TIMER[i].state[0];
                        sendTelemertry("auto_by", auto_by);
                    }
                    if (TIMER[i].keyName[k] == keyName)
                    {
                        if (keyName == "power_0" && (auto_by == BY_SCHEDULE))
                        {
                            if (TIMER[i].state[k] == true)
                            {
                                open_rem_1();
                            }
                            else
                            {
                                close_rem_1();
                            }
                            sendTelemertry("power_0", state_rem_1);
                        }
                        if (keyName == "power_1" && (auto_by == BY_SCHEDULE))
                        {
                            if (TIMER[i].state[k] == true)
                            {
                                open_rem_2();
                            }
                            else
                            {
                                close_rem_2();
                            }
                            sendTelemertry("power_1", state_rem_2);
                        }
                        // digitalWrite(relayPin[k], TIMER[i].state[k]);
                        // Serial.println("STATE RL: " + (String)k + "state: " + (String)TIMER[i].state[0]);
                        // sendTelemertryRelay(k + 1);
                    }
                }
            }
        }
        else
        {
        }
    }
}

void exportDeviceConfig()
{
    String data = readFile(SPIFFS, PATH_CONFIG);
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
        if (doc.containsKey("token"))
            device_config.token = doc["token"].as<String>();
        if (doc.containsKey("id"))
            device_config.id = doc["id"].as<String>();
        if (doc.containsKey("local_ip"))
            device_config.local_ip = doc["local_ip"].as<String>();
        if (doc.containsKey("gateway"))
            device_config.gateway = doc["gateway"].as<String>();
        if (doc.containsKey("subnet"))
            device_config.subnet = doc["subnet"].as<String>();
        if (doc.containsKey("dns1"))
            device_config.dns1 = doc["dns1"].as<String>();
        if (doc.containsKey("dns2"))
            device_config.dns2 = doc["dns2"].as<String>();
        if (doc.containsKey("dhcp"))
            device_config.dhcp = doc["dhcp"].as<bool>();
    }
}
/////////////////////OTA
String getHeaderValue(String header, String headerName)
{
    return header.substring(strlen(headerName.c_str()));
}
OTA_status_check_t checkFirmwareVersionHttp()
{
    String firmInfor = "/api/v1/" + device_config.token + "/attributes";
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("Connecting to: " + String(HTTP_SEVER));
    // Connect to S3
    if (client.connect(HTTP_SEVER, HTTP_PORT))
    {
        // Connection Succeed.
        // Fecthing the bin
        Serial.println("Fetching Bin: " + String(firmInfor));

        // Get the contents of the bin file
        client.print(String("GET ") + firmInfor + " HTTP/1.1\r\n" +
                     "Host: " + HTTP_SEVER + "\r\n" +
                     "Cache-Control: no-cache\r\n" +
                     "Connection: close\r\n\r\n");

        // Check what is being sent
        //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
        //                 "Host: " + host + "\r\n" +
        //                 "Cache-Control: no-cache\r\n" +
        //                 "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (client.available() == 0)
        {
            if (millis() - timeout > 5000)
            {
                Serial.println("Client Timeout !");
                client.stop();
                return ERROR;
            }
        }
        // Once the response is available,
        // check stuff
        while (client.available())
        {
            // read line till /n
            String line = client.readStringUntil('\n');
            // remove space, to check if the line is end of headers
            line.trim();
            if (!line.length())
            {
                // headers ended
                break; // and get the OTA started
            }

            // Check if the HTTP Response is 200
            // else break and Exit Update
            if (line.startsWith("HTTP/1.1"))
            {
                if (line.indexOf("200") < 0)
                {
                    Serial.println("Got a non 200 status code from server. Exiting...");
                    break;
                }
            }
            // extract headers here
            // Start with content length
            if (line.startsWith("Content-Length: "))
            {
                contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
            }
            // Next, the content type
            if (line.startsWith("Content-Type: "))
            {
                String contentType = getHeaderValue(line, "Content-Type: ");
                Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/json")
                {
                    isValidContentType = true;
                }
            }
        }
    }
    else
    {
        Serial.println("Connection to " + String(HTTP_SEVER) + " failed. Please check your setup");
    }
    // Check what is the contentLength and if content type is `application/json`
    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    String body;
    while (client.available())
    {
        body = client.readStringUntil((char)255);
        // Serial.print(body);
    }
    // get body done, stop connection
    client.stop();

    /*Parsing body to get firmware infor*/
    DynamicJsonDocument newFirmJson(2048);
    DeserializationError newFirmJsonError = deserializeJson(newFirmJson, body);

    if (newFirmJsonError)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(newFirmJsonError.c_str());
        return ERROR;
    }
    mqtt_firmware.fw_title = newFirmJson["shared"]["fw_title"].as<String>();
    mqtt_firmware.fw_version = newFirmJson["shared"]["fw_version"].as<String>();

    /*Load old fimrware_infor from File System*/
    String current_fw_title;
    String current_fw_version;
    String current_fw_tag;
    String now_firmware_string = readFile(SPIFFS, PATH_FIRM_INFOR);
    Serial.println("READ FILE: " + (String)PATH_FIRM_INFOR);
    Serial.println(now_firmware_string);
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, now_firmware_string.c_str());
    if (error)
    {
        Serial.println("deserializeJson failed");
        Serial.println(error.f_str());
        return ERROR;
    }
    else
    {
        if (doc.containsKey("fw_title"))
        {
            current_fw_title = doc["fw_title"].as<String>();
        }
        if (doc.containsKey("fw_version"))
        {
            current_fw_version = doc["fw_version"].as<String>();
        }
        if (doc.containsKey("fw_tag"))
        {
            current_fw_tag = doc["fw_tag"].as<String>();
        }
        Serial.println("EXPORT OK");
    }
    if ((current_fw_title != mqtt_firmware.fw_title) || (current_fw_version != mqtt_firmware.fw_version))
    {
        Serial.println("New firmware available");
        mqtt_firmware.bin = "/api/v1/" + (String)TOKEN + "/firmware?&version=" + mqtt_firmware.fw_version + "&title=" + mqtt_firmware.fw_title;
        return NEW_FIRM_AVAILABLE;
    }
    if ((current_fw_title == mqtt_firmware.fw_title) && (current_fw_version == mqtt_firmware.fw_version))
    {
        Serial.println("No_firmware available");
        return NO_NEW_FIRM_AVAILABLE;
    }
}

OTA_status_check_t checkFirmwareVersion()
{
    String current_fw_title;
    String current_fw_version;
    String current_fw_tag;
    String now_firmware_string = readFile(SPIFFS, PATH_FIRM_INFOR);
    Serial.println("READ FILE: " + (String)PATH_FIRM_INFOR);
    Serial.println(now_firmware_string);
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, now_firmware_string.c_str());
    if (error)
    {
        Serial.println("deserializeJson failed");
        Serial.println(error.f_str());
        return ERROR;
    }
    else
    {
        if (doc.containsKey("fw_title"))
        {
            current_fw_title = doc["fw_title"].as<String>();
        }
        if (doc.containsKey("fw_version"))
        {
            current_fw_version = doc["fw_version"].as<String>();
        }
        if (doc.containsKey("fw_tag"))
        {
            current_fw_tag = doc["fw_tag"].as<String>();
        }
        Serial.println("EXPORT OK");
    }
    if ((current_fw_title != mqtt_firmware.fw_title) || (current_fw_version != mqtt_firmware.fw_version))
    {
        Serial.println("New firmware available");
        mqtt_firmware.bin = "/api/v1/" + (String)TOKEN + "/firmware?&version=" + mqtt_firmware.fw_version + "&title=" + mqtt_firmware.fw_title;
        return NEW_FIRM_AVAILABLE;
    }
    if ((current_fw_title == mqtt_firmware.fw_title) && (current_fw_version == mqtt_firmware.fw_version))
    {
        Serial.println("No_firmware available");
        return NO_NEW_FIRM_AVAILABLE;
    }
}
void executeOTA()
{
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("Connecting to: " + String(HTTP_SEVER));
    // Connect to S3
    if (client.connect(HTTP_SEVER, HTTP_PORT))
    {
        // Connection Succeed.
        // Fecthing the bin
        Serial.println("Fetching Bin: " + String(mqtt_firmware.bin));

        // Get the contents of the bin file
        client.print(String("GET ") + mqtt_firmware.bin + " HTTP/1.1\r\n" +
                     "Host: " + HTTP_SEVER + "\r\n" +
                     "Cache-Control: no-cache\r\n" +
                     "Connection: close\r\n\r\n");

        // Check what is being sent
        //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
        //                 "Host: " + host + "\r\n" +
        //                 "Cache-Control: no-cache\r\n" +
        //                 "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (client.available() == 0)
        {
            if (millis() - timeout > 5000)
            {
                Serial.println("Client Timeout !");
                client.stop();
                return;
            }
        }
        // Once the response is available,
        // check stuff

        /*
           Response Structure
            HTTP/1.1 200 OK
            x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
            x-amz-request-id: 2D56B47560B764EC
            Date: Wed, 14 Jun 2017 03:33:59 GMT
            Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
            ETag: "d2afebbaaebc38cd669ce36727152af9"
            Accept-Ranges: bytes
            Content-Type: application/octet-stream
            Content-Length: 357280
            Server: AmazonS3
            {{BIN FILE CONTENTS}}
        */
        while (client.available())
        {
            // read line till /n
            String line = client.readStringUntil('\n');
            // remove space, to check if the line is end of headers
            line.trim();

            // if the the line is empty,
            // this is end of headers
            // break the while and feed the
            // remaining `client` to the
            // Update.writeStream();
            if (!line.length())
            {
                // headers ended
                break; // and get the OTA started
            }

            // Check if the HTTP Response is 200
            // else break and Exit Update
            if (line.startsWith("HTTP/1.1"))
            {
                if (line.indexOf("200") < 0)
                {
                    Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
                    break;
                }
            }

            // extract headers here
            // Start with content length
            if (line.startsWith("Content-Length: "))
            {
                contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
                Serial.println("line: " + (String)line);
            }

            // Next, the content type
            if (line.startsWith("Content-Type: "))
            {
                String contentType = getHeaderValue(line, "Content-Type: ");
                Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/octet-stream")
                {
                    isValidContentType = true;
                }
            }
        }
    }
    else
    {
        // Connect to S3 failed
        // May be try?
        // Probably a choppy network?
        Serial.println("Connection to " + String(HTTP_SEVER) + " failed. Please check your setup");
        // retry??
        // executeOTA();
    }

    // Check what is the contentLength and if content type is `application/octet-stream`
    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    // check contentLength and content type
    if (contentLength && isValidContentType)
    {
        // Check if there is enough to OTA Update
        bool canBegin = Update.begin(contentLength);

        // If yes, begin
        if (canBegin)
        {
            Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
            // No activity would appear on the Serial monitor
            // So be patient. This may take 2 - 5mins to complete
            sendTelemertry("fw_state", (String) "INSTALLING");
            size_t written = Update.writeStream(client);

            if (written == contentLength)
            {
                Serial.println("Written : " + String(written) + " successfully");
            }
            else
            {
                Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
                // retry??
                // executeOTA();
            }

            if (Update.end())
            {

                Serial.println("OTA done!");
                if (Update.isFinished())
                {
                    sendTelemertry("fw_state", (String) "FINISHED");
                    Serial.println("Update successfully completed. Rebooting.");
                    DynamicJsonDocument data(200);
                    data["fw_title"] = mqtt_firmware.fw_title;
                    data["fw_version"] = mqtt_firmware.fw_version;
                    data["fw_tag"] = mqtt_firmware.fw_tag;
                    String objectString;
                    serializeJson(data, objectString);
                    writeFile(SPIFFS, PATH_FIRM_INFOR, objectString.c_str());
                    Serial.println("SAVE FILE: " + (String)objectString);
                    ESP.restart();
                }
                else
                {
                    Serial.println("Update not finished? Something went wrong!");
                }
            }
            else
            {
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
            }
        }
        else
        {
            // not enough space to begin OTA
            // Understand the partitions and
            // space availability
            Serial.println("Not enough space to begin OTA");
            client.flush();
        }
    }
    else
    {
        Serial.println("There was no content in the response");
        client.flush();
    }
}
