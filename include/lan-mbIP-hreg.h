
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "mQECxIw9xvGh1PxpKTFq"
#define ID "ae9b6ce0-5f74-11ed-99cb-4de8ebde04d6"

// #define BY_SENSOR (0)

#include "ETH.h"
#include "Arduino.h"
#include "../include/ML_16IO/IO_define.h"
#include "HardwareSerial.h"
#include "ModbusRTU.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include <HTTPClient.h>
#include "LITTLEFS.h"
#include "ModbusIP_ESP8266.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "NTPClient.h"
#include "WiFiUdp.h"


AsyncWebServer server(80);

IPAddress local_ip(192, 168, 1, 103);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (8, 8, 4, 4);

struct sensor_value
{
    float temp;
    float humi;
    float lux;
} sensor;

#include <HttpClient.h>
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
String httpGETRequest(const char *serverName);

ModbusIP mbIP;

void ModbusIpInit();
void ModbusIpLoop();


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

void appInit()
{

    
    pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
    digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
    ETH.config(local_ip, gateway, subnet, dns1, dns2);
    Serial.begin(9600);

    WiFi.onEvent(WiFiEvent);

    ModbusIpInit();

    mqttInit();
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Bộ điều khiển vườn LAN"); });

    AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
    server.begin();
}

void appRun()
{

    sensor.temp = (float)mbIP.Hreg(0) / 10.0;
    sensor.humi = (float)mbIP.Hreg(1) / 10.0;
    sensor.lux = mbIP.Hreg(2);

    if (eth_connected)
    {
        mqttLoop();

    }
    ModbusIpLoop();
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


        }
    }
    else if (strstr((char *)payload, "update_schedule") != NULL)
    {

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
        sendTelemertry("humi_value", sensor.humi);
        sendTelemertry("temp_value", sensor.temp);
        sendTelemertry("lux_value", sensor.lux);
        lastTime = millis();
    }

    client.loop();
}
void mqttReconnect()
{
    if ((!client.connected()))
    {
        Serial.println("Connecting to thingsboard...");
        if (client.connect(ID, TOKEN, NULL))
        {
            Serial.println("Connected");
            client.subscribe("v1/devices/me/rpc/request/+");
            client.subscribe("v1/devices/me/attributes");
            // sendTelemertrySysInfo();
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
    DynamicJsonDocument data(512);
    value == true ? data[key] = true : data[key] = false;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, String value)
{
    DynamicJsonDocument data(512);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, float value)
{
    DynamicJsonDocument data(512);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}
void sendTelemertry(String key, int value)
{
    DynamicJsonDocument data(512);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
}

void ModbusIpInit()
{
    mbIP.server();
    mbIP.addHreg(0, 0, 20);
}
void ModbusIpLoop()
{
    static bool preCoilsState[14];
    mbIP.task();
}
