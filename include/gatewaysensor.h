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
#include "SoftwareSerial.h"
////////////////////////////////////////////////////
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "JmXG2xFfL2eajkjlhN48"
#define ID "4d2b0b00-f8bc-11ed-b16a-6fb926720a05"
#define TIME_CHECK_WIFI 2000
#define TIME_SEND_SS 1000
////////////////////////////////////////////////////
#define ADDR_SET_1 0x1000
#define ADDR_SET_2 0x1001
#define ADDR_SET_3 0x1002
#define ADDR_SET_4 0x1003
#define ADDR_SET_5 0x1004
#define ADDR_SET_6 0x1005
#define ADDR_SET_7 0x1006
#define ADDR_SET_8 0x1007
#define ADDR_SET_9 0x1008
#define ADDR_SET_10 0x1009
#define ADDR_SET_11 0x1010
#define ADDR_SET_12 0x1011
#define ADDR_SET_13 0x1012
#define ADDR_SET_14 0x1013
#define ADDR_SET_15 0x1014
#define ADDR_SET_16 0x1015
#define ADDR_SET_17 0x1016
#define ADDR_SET_18 0x1017
#define ADDR_SET_19 0x1018
#define ADDR_SET_20 0x1019
#define ADDR_SET_21 0x1020
#define ADDR_SET_22 0x1021
#define ADDR_SET_23 0x1022
#define ADDR_SET_24 0x1023
#define ADDR_SET_25 0x1024
#define ADDR_SET_26 0x1025
#define ADDR_SET_27 0x1026
#define ADDR_SET_28 0x1027
#define ADDR_SET_29 0x1028
#define ADDR_SET_30 0x1029
#define ADDR_SET_31 0x1030
#define ADDR_SET_32 0x1031
#define ADDR_SET_33 0x1032
#define ADDR_SET_34 0x1033
#define ADDR_SET_35 0x1034
#define ADDR_SET_36 0x1035
#define ADDR_SET_37 0x1036
#define ADDR_SET_38 0x1037
#define ADDR_SET_39 0x1038
#define ADDR_SET_40 0x1039

/////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial DWIN_LCD(1);
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
AsyncWebServer server(80);
WiFiUDP ntpUDP;
const char *ssid = "Nam_NH";
const char *pass = "123456789";

//////////////////////////
// void DGUS_SendVal(int iAdr, float iVal);
void DGUS_SendVal(int iAdr, float fVal);
void dwinShowPage(int page);
void updateHMIState();

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
void switchPage();
////////////////////////////
IPAddress local_IP(192, 168, 1, 10);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

const int REG = 528; // Modbus Hreg Offset

IPAddress remote(192, 168, 1, 11);  // tu cam bien 1
IPAddress remote2(192, 168, 1, 12); // tu cam bien 2
const int LOOP_COUNT = 10;

ModbusIP mbIP;

float nhietdo[20];
float nhietdo2[20];

unsigned long timeoutbt = 0;
int Sw_page = 0;

void appInit()
{
    pinMode(2, INPUT_PULLUP);
    Serial.begin(9600);
    DWIN_LCD.begin(115200, SERIAL_8N1, 4, 5);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }
    // WiFi.begin(ssid, pass);
    mqttInit();
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wificonnect...");
        Serial.print(".");
        switchPage();
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

uint8_t show = LOOP_COUNT;
uint16_t data[20];
uint16_t data2[20];

void appRun()
{
    if (WL_CONNECTED)
    {
        mqtt_loop();
    }
    if (mbIP.isConnected(remote))
    { // Check if connection to Modbus Slave is established
        // Sensor1
        mbIP.readHreg(remote, 0, data, 10);
        for (int i = 1; i < 20; i++)
        {
            Serial.print("Nhiệt độ cảm biến ");
            Serial.print(i);
            Serial.print(": ");
            nhietdo[i] = data[i] / 100.0;
            Serial.print(nhietdo[i]);
            Serial.println(" độ C");
        }

        // Sensor2
        mbIP.readHreg(remote2, 0, data2, 10);
        for (int i = 1; i < 20; i++)
        {
            Serial.print("Nhiệt độ cảm biến ");
            Serial.print(i);
            Serial.print(": ");
            nhietdo2[i] = data2[i] / 100.0;
            Serial.print(nhietdo2[i]);
            Serial.println(" độ C");
        }

    }
    else
    {
        mbIP.connect(remote); // Try to connect if no connection
    }

    updateHMIState();
    switchPage();

    mbIP.task();
}

///////////////////////////////////////////////////////////////////////////////////////
void updateHMIState()
{
    static unsigned long timePrint = millis();

    if (millis() - timePrint > 1000)
    {
        // Sensor1
        DGUS_SendVal(ADDR_SET_1, nhietdo[1] * 100);
        DGUS_SendVal(ADDR_SET_2, nhietdo[2] * 100);
        DGUS_SendVal(ADDR_SET_3, nhietdo[3] * 100);
        DGUS_SendVal(ADDR_SET_4, nhietdo[4] * 100);
        DGUS_SendVal(ADDR_SET_5, nhietdo[5] * 100);
        DGUS_SendVal(ADDR_SET_6, nhietdo[6] * 100);
        DGUS_SendVal(ADDR_SET_7, nhietdo[7] * 100);
        DGUS_SendVal(ADDR_SET_8, nhietdo[8] * 100);
        DGUS_SendVal(ADDR_SET_9, nhietdo[9] * 100);
        DGUS_SendVal(ADDR_SET_10, nhietdo[10] * 100);
        DGUS_SendVal(ADDR_SET_11, nhietdo[11] * 100);
        DGUS_SendVal(ADDR_SET_12, nhietdo[12] * 100);
        DGUS_SendVal(ADDR_SET_13, nhietdo[13] * 100);
        DGUS_SendVal(ADDR_SET_14, nhietdo[14] * 100);
        DGUS_SendVal(ADDR_SET_15, nhietdo[15] * 100);
        DGUS_SendVal(ADDR_SET_16, nhietdo[16] * 100);
        DGUS_SendVal(ADDR_SET_17, nhietdo[17] * 100);
        DGUS_SendVal(ADDR_SET_18, nhietdo[18] * 100);
        DGUS_SendVal(ADDR_SET_19, nhietdo[19] * 100);
        DGUS_SendVal(ADDR_SET_20, nhietdo[20] * 100);
        // Sensor 2
        DGUS_SendVal(ADDR_SET_21, nhietdo2[1] * 100);
        DGUS_SendVal(ADDR_SET_22, nhietdo2[2] * 100);
        DGUS_SendVal(ADDR_SET_23, nhietdo2[3] * 100);
        DGUS_SendVal(ADDR_SET_24, nhietdo2[4] * 100);
        DGUS_SendVal(ADDR_SET_25, nhietdo2[5] * 100);
        DGUS_SendVal(ADDR_SET_26, nhietdo2[6] * 100);
        DGUS_SendVal(ADDR_SET_27, nhietdo2[7] * 100);
        DGUS_SendVal(ADDR_SET_28, nhietdo2[8] * 100);
        DGUS_SendVal(ADDR_SET_29, nhietdo2[9] * 100);
        DGUS_SendVal(ADDR_SET_30, nhietdo2[10] * 100);
        DGUS_SendVal(ADDR_SET_31, nhietdo2[21] * 100);
        DGUS_SendVal(ADDR_SET_32, nhietdo2[22] * 100);
        DGUS_SendVal(ADDR_SET_33, nhietdo2[23] * 100);
        DGUS_SendVal(ADDR_SET_34, nhietdo2[24] * 100);
        DGUS_SendVal(ADDR_SET_35, nhietdo2[25] * 100);
        DGUS_SendVal(ADDR_SET_36, nhietdo2[26] * 100);
        DGUS_SendVal(ADDR_SET_37, nhietdo2[27] * 100);
        DGUS_SendVal(ADDR_SET_38, nhietdo2[28] * 100);
        DGUS_SendVal(ADDR_SET_39, nhietdo2[29] * 100);
        DGUS_SendVal(ADDR_SET_40, nhietdo2[20] * 100);
        
        timePrint = millis();
    }
}

void dwinShowPage(int page)
{

    static int curentPgae = -1;
    if (curentPgae != page)
    {
        curentPgae = page;
        Serial.println("page: " + (String)curentPgae);
        byte bValL, bValH;
        bValL = page & 0xFF;
        bValH = (page >> 8) & 0xFF;
        DWIN_LCD.write(0x5A);
        DWIN_LCD.write(0xA5);
        DWIN_LCD.write(0x07);
        DWIN_LCD.write(0x82);
        DWIN_LCD.write(0x00);
        DWIN_LCD.write(0x84);
        DWIN_LCD.write(0x5A);
        DWIN_LCD.write(0x01);
        DWIN_LCD.write(bValH);
        DWIN_LCD.write(bValL);
    }
}

// void DGUS_SendVal(int iAdr, float iVal) // Send iVal for VP= iAdr to DGUS
// {
//     byte bAdrL, bAdrH, bValL, bValH;
//     bAdrL = iAdr & 0xFF;
//     bAdrH = (iAdr >> 8) & 0xFF;
//     bValL = (int)iVal & 0xFF;
//     bValH = ((int)iVal >> 8) & 0xFF;
//     DWIN_LCD.write(0x5A);
//     DWIN_LCD.write(0xA5);
//     DWIN_LCD.write(0x05);
//     DWIN_LCD.write(0x82);
//     DWIN_LCD.write(bAdrH);
//     DWIN_LCD.write(bAdrL);
//     DWIN_LCD.write(bValH);
//     DWIN_LCD.write(bValL);
// }
void DGUS_SendVal(int iAdr, float fVal) // Send fVal for VP= iAdr to DGUS
{
    byte bAdrL, bAdrH, bValL, bValH;
    bAdrL = iAdr & 0xFF;
    bAdrH = (iAdr >> 8) & 0xFF;
    bValL = (int)fVal & 0xFF;
    bValH = ((int)fVal >> 8) & 0xFF;
    DWIN_LCD.write(0x5A);
    DWIN_LCD.write(0xA5);
    DWIN_LCD.write(0x05);
    DWIN_LCD.write(0x82);
    DWIN_LCD.write(bAdrH);
    DWIN_LCD.write(bAdrL);
    DWIN_LCD.write(bValH);
    DWIN_LCD.write(bValL);
}

void switchPage()
{
    if (digitalRead(2) == 0)
    {
        if (millis() - timeoutbt > 500)
        {
            Sw_page++;
            if (Sw_page > 4)
            {
                Sw_page = 0;
            }
            dwinShowPage(Sw_page);
            Serial.println("PAGE : " + String(Sw_page));
            timeoutbt = millis();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
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
    }
    else if (strstr((char *)payload, "update_schedule") != NULL)
    {
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
        switchPage();
    }
    if (millis() - lastTime > 60000)
    {

        sendTelemertry("temp_value1", nhietdo[1]);
        sendTelemertry("temp_value2", nhietdo[2]);
        sendTelemertry("temp_value3", nhietdo[3]);
        sendTelemertry("temp_value4", nhietdo[4]);
        sendTelemertry("temp_value5", nhietdo[5]);
        sendTelemertry("temp_value6", nhietdo[6]);
        sendTelemertry("temp_value7", nhietdo[7]);
        sendTelemertry("temp_value8", nhietdo[8]);
        sendTelemertry("temp_value9", nhietdo[9]);
        sendTelemertry("temp_value10", nhietdo[10]);
        sendTelemertry("temp_value11", nhietdo[11]);
        sendTelemertry("temp_value12", nhietdo[12]);
        sendTelemertry("temp_value13", nhietdo[13]);
        sendTelemertry("temp_value14", nhietdo[14]);
        sendTelemertry("temp_value15", nhietdo[15]);
        sendTelemertry("temp_value16", nhietdo[16]);
        sendTelemertry("temp_value17", nhietdo[17]);
        sendTelemertry("temp_value18", nhietdo[18]);
        sendTelemertry("temp_value19", nhietdo[19]);
        sendTelemertry("temp_value20", nhietdo[20]);
        sendTelemertry("temp_value21", nhietdo2[1]);
        sendTelemertry("temp_value22", nhietdo2[2]);
        sendTelemertry("temp_value23", nhietdo2[3]);
        sendTelemertry("temp_value24", nhietdo2[4]);
        sendTelemertry("temp_value25", nhietdo2[5]);
        sendTelemertry("temp_value26", nhietdo2[6]);
        sendTelemertry("temp_value27", nhietdo2[7]);
        sendTelemertry("temp_value28", nhietdo2[8]);
        sendTelemertry("temp_value29", nhietdo2[9]);
        sendTelemertry("temp_value30", nhietdo2[10]);
        sendTelemertry("temp_value31", nhietdo2[11]);
        sendTelemertry("temp_value32", nhietdo2[12]);
        sendTelemertry("temp_value33", nhietdo2[13]);
        sendTelemertry("temp_value34", nhietdo2[14]);
        sendTelemertry("temp_value35", nhietdo2[15]);
        sendTelemertry("temp_value36", nhietdo2[16]);
        sendTelemertry("temp_value37", nhietdo2[17]);
        sendTelemertry("temp_value38", nhietdo2[18]);
        sendTelemertry("temp_value39", nhietdo2[19]);
        sendTelemertry("temp_value40", nhietdo2[20]);

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