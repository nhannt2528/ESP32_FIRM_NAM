#include "Arduino.h"
#include "ETH.h"
#include "ModbusRTU.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "time.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "FS.h"
#include <LITTLEFS.h>
// #include "AiEsp32RotaryEncoder.h"
#include "SoftwareSerial.h"
#include "FS.h"
#include "LITTLEFS.h"
// #include "SimpleKalmanFilter.h"
#include <AsyncElegantOTA.h>
#define FLOW_CALIBRATION 8.2
#define TIME_CHECK_WIFI 2000
HardwareSerial DWIN_LCD(1);
// SimpleKalmanFilter adc_fillter(2, 2, 0.001);
int home_seting_page[5] = {6, 8, 10, 15, 20};
#define PLC_ID 1

#define FLOW_SR_1 35
#define FLOW_SR_2 32
#define FLOW_SR_3 33
#define FLOW_SR_4 4
#define ADC_VP 36

#define SPINKLER 0
#define DRIP 1

#define PAGE_STOP_MODE_FLOW 0
#define PAGE_RUN_MODE_FLOW 1
#define PAGE_STOP_MODE_TIME 2
#define PAGE_RUN_MODE_TIME 3

#define PAGE_SETING_MODE_TUOI 6
#define PAGE_SETING_MODE_DOSING 8
#define PAGE_SETING_MODE_FLOW 10
#define PAGE_SETING_MODE_TIME 15
#define PAGE_SETING_TIME_VALVE 20

#define MODE_TIME 0x00 // switch valve by time
#define MODE_FLOW 0x01 // switch valve by flow

#define ADDR_SET_FLOW_1 0x1000
#define ADDR_SET_FLOW_2 0x1001
#define ADDR_SET_FLOW_3 0x1002
#define ADDR_SET_FLOW_4 0x1003

#define ADDR_SET_TIME_1 0x1004
#define ADDR_SET_TIME_2 0x1005
#define ADDR_SET_TIME_3 0x1006
#define ADDR_SET_TIME_4 0x1007

#define ADDR_SET_TIME_PUMP_MASTER 0x1010
#define ADDR_SET_TIME_PUMP_SLAVE 0x1011
#define ADDR_SET_VAN_TUOI_1 0x1012
#define ADDR_SET_VAN_TUOI_2 0x1013
#define ADDR_SET_VAN_TUOI_3 0x1014
#define ADDR_SET_VAN_TUOI_4 0x1015
#define ADDR_SET_VAN_TUOI_5 0x1016

#define ADDR_SET_VAN_TUOI_6 0x1017
#define ADDR_SET_VAN_TUOI_7 0x1018
#define ADDR_SET_VAN_TUOI_8 0x1019
#define ADDR_SET_VAN_TUOI_9 0x101A
#define ADDR_SET_VAN_TUOI_10 0x101B

#define ADDR_PRESURE_VALUE 0x100B

#define FLOW_SENSOR_1 0
#define FLOW_SENSOR_2 1
#define FLOW_SENSOR_3 2
#define FLOW_SENSOR_4 3

#define SET_FLOW_1 4
#define SET_FLOW_2 5
#define SET_FLOW_3 6
#define SET_FLOW_4 7
#define SET_TIME_VALVE_1 8
#define SET_TIME_VALVE_2 9
#define SET_TIME_VALVE_3 10
#define SET_TIME_VALVE_4 11

#define SET_TIME_VALVE_TUOI_1 12
#define SET_TIME_VALVE_TUOI_2 13
#define SET_TIME_VALVE_TUOI_3 14
#define SET_TIME_VALVE_TUOI_4 15
#define SET_TIME_VALVE_TUOI_5 16
#define SET_TIME_VALVE_TUOI_6 17
#define SET_TIME_VALVE_TUOI_7 18
#define SET_TIME_VALVE_TUOI_8 19
#define SET_TIME_VALVE_TUOI_9 20
#define SET_TIME_VALVE_TUOI_10 21

#define SET_MODE_DOSING 22 // châm phân theo thời gian hoặc là lưu lượng
#define SET_MODE_TUOI 23   // chạy theo tưới nhỏ giọt hoặc tưới phun sương
#define SET_STATR_STOP 24

#define SET_TIME_PUMP_MASTER 25
#define SET_TIME_PUMP_SLAVE 26

#define PATH_STORAGE "/data/localStorage.json"

#define KEY_SET_TIME_A "set_time_a"
#define KEY_SET_TIME_B "set_time_b"
#define KEY_SET_TIME_C "set_time_c"
#define KEY_SET_TIME_D "set_time_d"

#define KEY_SET_FLOW_A "set_flow_a"
#define KEY_SET_FLOW_B "set_flow_b"
#define KEY_SET_FLOW_C "set_flow_c"
#define KEY_SET_FLOW_D "set_flow_d"

#define KEY_SET_MODE_DOSING "set_mode_dosing"
#define KEY_SET_MODE_TUOI "set_mode_tuoi"

#define KEY_POWER_DOSING "set_power_dosing"
#define KEY_POWER_1 "set_power_1"
#define KEY_POWER_2 "set_power_2"
#define KEY_POWER_3 "set_power_3"
#define KEY_POWER_4 "set_power_4"
#define KEY_POWER_5 "set_power_5"
#define KEY_POWER_6 "set_power_6"
#define KEY_POWER_7 "set_power_7"
#define KEY_POWER_8 "set_power_8"
#define KEY_POWER_9 "set_power_9"
#define KEY_POWER_10 "set_power_10"
#define KEY_POWER_11 "set_power_11"

#define KEY_SET_TIME_TUOI_1 "set_time_tuoi_1"
#define KEY_SET_TIME_TUOI_2 "set_time_tuoi_2"
#define KEY_SET_TIME_TUOI_3 "set_time_tuoi_3"
#define KEY_SET_TIME_TUOI_4 "set_time_tuoi_4"
#define KEY_SET_TIME_TUOI_5 "set_time_tuoi_5"
#define KEY_SET_TIME_TUOI_6 "set_time_tuoi_6"
#define KEY_SET_TIME_TUOI_7 "set_time_tuoi_7"
#define KEY_SET_TIME_TUOI_8 "set_time_tuoi_8"
#define KEY_SET_TIME_TUOI_9 "set_time_tuoi_9"
#define KEY_SET_TIME_TUOI_10 "set_time_tuoi_10"

#define KEY_SET_HIGH_PRESURE "set_high_presure"
#define KEY_SET_LOW_PRESURE "set_low_presure"

#define ETH_ADDR 1
#define ETH_POWER_PIN -1 // Do not use it, it can cause conflict during the software reset.
#define ETH_POWER_PIN_ALTERNATIVE 14
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 34
#define ROTARY_ENCODER_BUTTON_PIN 15
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 8

#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "UH0nmY05l2XXQWlQYC1A"
#define ID "b309bf80-66f3-11ed-99cb-4de8ebde04d6"
AsyncWebServer server(80);
ModbusRTU mb;
WiFiClient espClient;
PubSubClient client(espClient);
// AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

IPAddress local_ip(192, 168, 1, 134);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (8, 8, 4, 4);
static bool eth_connected = false;

float set_high_presure = 0;
float set_low_presure = 0;
bool power[11];
uint16_t data_set[30];
const char *ssid = "Nep_2.4";
const char *pass = "0964190692";
typedef enum
{
    IDLE_STATE,
    SET_COILS_STATE,
    SET_HREG_DATA
} state;
state STATE;
typedef enum
{
    HOME_PAGE,
    SETING_PAGE,
    RUN_PAGE,
    SETING_MODE_TUOI,
    SETING_MODE_DOSING,
    SETING_MODE_FLOW,
    SETING_MODE_TIME,
    SETING_TIME
} PAGE;
PAGE DWIN;
void IRAM_ATTR readEncoderISR()
{
    // rotaryEncoder.readEncoder_ISR();
}
struct flow_sen
{
    volatile int flow_frequency;
    float vol = 0.0, l_minute;
    unsigned char flowsensor = 2; // Sensor Input
    unsigned long currentTime;
    unsigned long cloopTime;
    int flow_l_m = 0;
} flow_sensor[4];

void flow_1()
{
    flow_sensor[0].flow_frequency++;
}
void flow_2()
{
    flow_sensor[1].flow_frequency++;
}
void flow_3()
{
    flow_sensor[2].flow_frequency++;
}
void flow_4()
{
    flow_sensor[3].flow_frequency++;
}
float presure_value = 0;
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);
float readPresureSensor();
void flow_read(flow_sen &flow_sensor, int num);
void fileInit();
void writeFile(fs::FS &fs, const char *path, const char *message);
String readFile(fs::FS &fs, const char *path);
void createDir(fs::FS &fs, const char *path);

void localStorageExport();
void saveLocalStore();

void DGUS_SendVal(int iAdr, int iVal);
void dwinShowPage(int page);
void updateHMIState();
void sendTelemertry(String key, bool value);
void sendTelemertry(String key, uint16_t value);
void sendTelemertry(String key, float value);
void sendTelemertry(String key, String value);

void mqttInit();
void mqttConnect();
void on_message(const char *topic, byte *payload, unsigned int length);
void mqttLoop();
void mqttReconnect();
void rotary_onButtonClick();
void rotary_loop();
void WiFiEvent(WiFiEvent_t event);
void updateState();
void ETHInit();
void appInit();
void appRun();
void updateHMIState_2();
void wifiConnect();
void OTA_init();
void appInit()
{
    Serial.begin(9600);
    Serial2.begin(9600);
    pinMode(FLOW_SR_1, INPUT);
    pinMode(FLOW_SR_2, INPUT);
    pinMode(FLOW_SR_3, INPUT);
    pinMode(FLOW_SR_4, INPUT);
    pinMode(ADC_VP, INPUT);
    analogSetAttenuation(ADC_6db);

    fileInit();
  //  WiFi.begin(ssid, pass);
    // createDir(LITTLEFS,"/data");
    // saveLocalStore();
    localStorageExport();
    DWIN_LCD.begin(115200, SERIAL_8N1, 12, 2);
     ETHInit();
    mb.begin(&Serial2);
    mb.master();
    // rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    bool circleValues = false;
    rotaryEncoder.setAcceleration(0);
    delay(5000);
    attachInterrupt(digitalPinToInterrupt(FLOW_SR_1), flow_1, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SR_2), flow_2, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SR_3), flow_3, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SR_4), flow_4, RISING);
    sei();
    mqttInit();
    OTA_init();
    delay(100);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_1, data_set[SET_TIME_VALVE_1]);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_2, data_set[SET_TIME_VALVE_2]);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_3, data_set[SET_TIME_VALVE_3]);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_4, data_set[SET_TIME_VALVE_4]);
    delay(10);
    data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_STOP_MODE_FLOW) : dwinShowPage(PAGE_STOP_MODE_TIME);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_1, data_set[SET_TIME_VALVE_TUOI_1]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_2, data_set[SET_TIME_VALVE_TUOI_2]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_3, data_set[SET_TIME_VALVE_TUOI_3]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_4, data_set[SET_TIME_VALVE_TUOI_4]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_5, data_set[SET_TIME_VALVE_TUOI_5]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_6, data_set[SET_TIME_VALVE_TUOI_6]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_7, data_set[SET_TIME_VALVE_TUOI_7]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_8, data_set[SET_TIME_VALVE_TUOI_8]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_9, data_set[SET_TIME_VALVE_TUOI_9]);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_10, data_set[SET_TIME_VALVE_TUOI_10]);
}
void appRun()
{
 //  wifiConnect();

    static unsigned long preTime = millis();
    if (millis() - preTime > 100)
    {
        presure_value = readPresureSensor();
        DGUS_SendVal(ADDR_PRESURE_VALUE, presure_value * 10);
        preTime = millis();
    }
    updateHMIState();
    updateState();
    if (WiFi.isConnected() || eth_connected)
    {
        mqttLoop();
    }
}
void updateState()
{
    static unsigned long preTime = millis();
    if (millis() - preTime > 10)
    {
        if (!mb.slave())
        {
            switch (STATE)
            {
            case IDLE_STATE:
            {
                STATE = SET_COILS_STATE;
                break;
            }
            case SET_COILS_STATE:
            {
                mb.writeCoil(PLC_ID, 7, power, 11);
                STATE = SET_HREG_DATA;
                break;
            }
            case SET_HREG_DATA:
            {
                mb.writeHreg(PLC_ID, 0, data_set, 25);
                STATE = IDLE_STATE;
                break;
            }
            }
        }
        mb.task();
        yield();
        preTime = millis();
    }
}
void ETHInit()
{
    pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
    digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
    ETH.config(local_ip, gateway, subnet, dns1, dns2);
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
        if (doc["params"].containsKey(KEY_SET_TIME_A))
        {
            data_set[SET_TIME_VALVE_1] = doc["params"][KEY_SET_TIME_A].as<int>();
            sendTelemertry(KEY_SET_TIME_A, data_set[SET_TIME_VALVE_1]);
            DGUS_SendVal(ADDR_SET_TIME_1, data_set[SET_TIME_VALVE_1]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_B))
        {
            data_set[SET_TIME_VALVE_2] = doc["params"][KEY_SET_TIME_B].as<int>();
            sendTelemertry(KEY_SET_TIME_B, data_set[SET_TIME_VALVE_2]);
            DGUS_SendVal(ADDR_SET_TIME_2, data_set[SET_TIME_VALVE_2]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_C))
        {
            data_set[SET_TIME_VALVE_3] = doc["params"][KEY_SET_TIME_C].as<int>();
            sendTelemertry(KEY_SET_TIME_C, data_set[SET_TIME_VALVE_3]);
            DGUS_SendVal(ADDR_SET_TIME_3, data_set[SET_TIME_VALVE_3]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_D))
        {
            data_set[SET_TIME_VALVE_4] = doc["params"][KEY_SET_TIME_D].as<int>();
            sendTelemertry(KEY_SET_TIME_D, data_set[SET_TIME_VALVE_4]);
            DGUS_SendVal(ADDR_SET_TIME_4, data_set[SET_TIME_VALVE_4]);
        }
        if (doc["params"].containsKey(KEY_SET_FLOW_A))
        {
            data_set[SET_FLOW_1] = doc["params"][KEY_SET_FLOW_A].as<int>();
            sendTelemertry(KEY_SET_FLOW_A, data_set[SET_FLOW_1]);
            DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
        }
        if (doc["params"].containsKey(KEY_SET_FLOW_B))
        {
            data_set[SET_FLOW_2] = doc["params"][KEY_SET_FLOW_B].as<int>();
            sendTelemertry(KEY_SET_FLOW_B, data_set[SET_FLOW_2]);
            DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
        }
        if (doc["params"].containsKey(KEY_SET_FLOW_C))
        {
            data_set[SET_FLOW_3] = doc["params"][KEY_SET_FLOW_C].as<int>();
            sendTelemertry(KEY_SET_FLOW_C, data_set[SET_FLOW_3]);
            DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
        }
        if (doc["params"].containsKey(KEY_SET_FLOW_D))
        {
            data_set[SET_FLOW_4] = doc["params"][KEY_SET_FLOW_D].as<int>();
            sendTelemertry(KEY_SET_FLOW_D, data_set[SET_FLOW_4]);
            DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
        }
        if (doc["params"].containsKey(KEY_POWER_DOSING))
        {
            data_set[SET_STATR_STOP] = doc["params"][KEY_POWER_DOSING].as<bool>();
            data_set[SET_STATR_STOP] ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
            data_set[SET_STATR_STOP] ? DWIN = RUN_PAGE : DWIN = HOME_PAGE;
        }
        if (doc["params"].containsKey(KEY_POWER_1))
        {
            power[0] = doc["params"][KEY_POWER_1].as<bool>();
            sendTelemertry(KEY_POWER_1, power[0]);
        }
        if (doc["params"].containsKey(KEY_POWER_2))
        {
            power[1] = doc["params"][KEY_POWER_2].as<bool>();
            sendTelemertry(KEY_POWER_2, power[1]);
        }
        if (doc["params"].containsKey(KEY_POWER_3))
        {
            power[2] = doc["params"][KEY_POWER_3].as<bool>();
            sendTelemertry(KEY_POWER_3, power[2]);
        }
        if (doc["params"].containsKey(KEY_POWER_4))
        {
            power[3] = doc["params"][KEY_POWER_4].as<bool>();
            sendTelemertry(KEY_POWER_4, power[3]);
        }
        if (doc["params"].containsKey(KEY_POWER_5))
        {
            power[4] = doc["params"][KEY_POWER_5].as<bool>();
            sendTelemertry(KEY_POWER_5, power[4]);
        }
        if (doc["params"].containsKey(KEY_POWER_6))
        {
            power[5] = doc["params"][KEY_POWER_6].as<bool>();
            sendTelemertry(KEY_POWER_6, power[5]);
        }
        if (doc["params"].containsKey(KEY_POWER_7))
        {
            power[6] = doc["params"][KEY_POWER_7].as<bool>();
            sendTelemertry(KEY_POWER_7, power[6]);
        }
        if (doc["params"].containsKey(KEY_POWER_8))
        {
            power[7] = doc["params"][KEY_POWER_8].as<bool>();
            sendTelemertry(KEY_POWER_8, power[7]);
        }
        if (doc["params"].containsKey(KEY_POWER_9))
        {
            power[8] = doc["params"][KEY_POWER_9].as<bool>();
            sendTelemertry(KEY_POWER_9, power[8]);
        }
        if (doc["params"].containsKey(KEY_POWER_10))
        {
            power[9] = doc["params"][KEY_POWER_10].as<bool>();
            sendTelemertry(KEY_POWER_10, power[9]);
        }
        if (doc["params"].containsKey(KEY_POWER_11))
        {
            power[10] = doc["params"][KEY_POWER_11].as<bool>();
            sendTelemertry(KEY_POWER_11, power[10]);
        }
        if (doc["params"].containsKey(KEY_SET_MODE_TUOI))
        {
            data_set[SET_MODE_TUOI] = doc["params"][KEY_SET_MODE_TUOI].as<int>();
            sendTelemertry(KEY_SET_MODE_TUOI, data_set[SET_MODE_TUOI]);
        }
        if (doc["params"].containsKey(KEY_SET_MODE_DOSING))
        {
            data_set[SET_MODE_DOSING] = doc["params"][KEY_SET_MODE_DOSING].as<int>();
            sendTelemertry(KEY_SET_MODE_DOSING, data_set[SET_MODE_DOSING]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_1))
        {
            data_set[SET_TIME_VALVE_TUOI_1] = doc["params"][KEY_SET_TIME_TUOI_1].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_1, data_set[SET_TIME_VALVE_TUOI_1]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_2))
        {
            data_set[SET_TIME_VALVE_TUOI_2] = doc["params"][KEY_SET_TIME_TUOI_2].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_2, data_set[SET_TIME_VALVE_TUOI_2]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_3))
        {
            data_set[SET_TIME_VALVE_TUOI_3] = doc["params"][KEY_SET_TIME_TUOI_3].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_3, data_set[SET_TIME_VALVE_TUOI_3]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_4))
        {
            data_set[SET_TIME_VALVE_TUOI_4] = doc["params"][KEY_SET_TIME_TUOI_4].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_4, data_set[SET_TIME_VALVE_TUOI_4]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_5))
        {
            data_set[SET_TIME_VALVE_TUOI_5] = doc["params"][KEY_SET_TIME_TUOI_5].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_5, data_set[SET_TIME_VALVE_TUOI_5]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_6))
        {
            data_set[SET_TIME_VALVE_TUOI_6] = doc["params"][KEY_SET_TIME_TUOI_6].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_6, data_set[SET_TIME_VALVE_TUOI_6]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_7))
        {
            data_set[SET_TIME_VALVE_TUOI_7] = doc["params"][KEY_SET_TIME_TUOI_7].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_7, data_set[SET_TIME_VALVE_TUOI_7]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_8))
        {
            data_set[SET_TIME_VALVE_TUOI_8] = doc["params"][KEY_SET_TIME_TUOI_8].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_8, data_set[SET_TIME_VALVE_TUOI_8]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_9))
        {
            data_set[SET_TIME_VALVE_TUOI_9] = doc["params"][KEY_SET_TIME_TUOI_9].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_9, data_set[SET_TIME_VALVE_TUOI_9]);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_TUOI_10))
        {
            data_set[SET_TIME_VALVE_TUOI_10] = doc["params"][KEY_SET_TIME_TUOI_10].as<int>();
            sendTelemertry(KEY_SET_TIME_TUOI_10, data_set[SET_TIME_VALVE_TUOI_10]);
        }
        if (doc["params"].containsKey(KEY_SET_HIGH_PRESURE))
        {
            set_high_presure = doc["params"][KEY_SET_HIGH_PRESURE].as<float>();
            sendTelemertry(KEY_SET_HIGH_PRESURE, set_high_presure);
        }
        if (doc["params"].containsKey(KEY_SET_LOW_PRESURE))
        {
            set_low_presure = doc["params"][KEY_SET_LOW_PRESURE].as<float>();
            sendTelemertry(KEY_SET_LOW_PRESURE, set_low_presure);
        }
        saveLocalStore();
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
    if (millis() - lastTime > 10000)
    {
        sendTelemertry("presure_value", presure_value);
        lastTime = millis();
    }
    if (!client.connected())
    {
        mqttReconnect();
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
            sendTelemertry(KEY_SET_FLOW_A, data_set[SET_FLOW_1]);
            sendTelemertry(KEY_SET_FLOW_B, data_set[SET_FLOW_2]);
            sendTelemertry(KEY_SET_FLOW_C, data_set[SET_FLOW_3]);
            sendTelemertry(KEY_SET_FLOW_D, data_set[SET_FLOW_4]);
            sendTelemertry(KEY_SET_TIME_A, data_set[SET_TIME_VALVE_1]);
            sendTelemertry(KEY_SET_TIME_B, data_set[SET_TIME_VALVE_2]);
            sendTelemertry(KEY_SET_TIME_C, data_set[SET_TIME_VALVE_3]);
            sendTelemertry(KEY_SET_TIME_D, data_set[SET_TIME_VALVE_4]);
            sendTelemertry(KEY_SET_MODE_DOSING, data_set[SET_MODE_DOSING]);
            sendTelemertry(KEY_SET_MODE_TUOI, data_set[SET_MODE_TUOI]);

            sendTelemertry(KEY_SET_TIME_TUOI_1, data_set[SET_TIME_VALVE_TUOI_1]);
            sendTelemertry(KEY_SET_TIME_TUOI_2, data_set[SET_TIME_VALVE_TUOI_2]);
            sendTelemertry(KEY_SET_TIME_TUOI_3, data_set[SET_TIME_VALVE_TUOI_3]);
            sendTelemertry(KEY_SET_TIME_TUOI_4, data_set[SET_TIME_VALVE_TUOI_4]);
            sendTelemertry(KEY_SET_TIME_TUOI_5, data_set[SET_TIME_VALVE_TUOI_5]);
            sendTelemertry(KEY_SET_TIME_TUOI_6, data_set[SET_TIME_VALVE_TUOI_6]);
            sendTelemertry(KEY_SET_TIME_TUOI_7, data_set[SET_TIME_VALVE_TUOI_7]);
            sendTelemertry(KEY_SET_TIME_TUOI_8, data_set[SET_TIME_VALVE_TUOI_8]);
            sendTelemertry(KEY_SET_TIME_TUOI_9, data_set[SET_TIME_VALVE_TUOI_9]);
            sendTelemertry(KEY_SET_TIME_TUOI_10, data_set[SET_TIME_VALVE_TUOI_10]);

            sendTelemertry(KEY_POWER_DOSING, data_set[SET_STATR_STOP]);
            sendTelemertry(KEY_POWER_1, power[0]);
            sendTelemertry(KEY_POWER_2, power[1]);
            sendTelemertry(KEY_POWER_3, power[2]);
            sendTelemertry(KEY_POWER_4, power[3]);
            sendTelemertry(KEY_POWER_5, power[4]);
            sendTelemertry(KEY_POWER_6, power[5]);
            sendTelemertry(KEY_POWER_7, power[6]);
            sendTelemertry(KEY_POWER_8, power[7]);
            sendTelemertry(KEY_POWER_9, power[8]);
            sendTelemertry(KEY_POWER_10, power[9]);
            sendTelemertry(KEY_POWER_11, power[10]);
            sendTelemertry("IP", WiFi.localIP().toString());
        }
        else
        {
            Serial.println("Connect fail");
            Serial.println(client.state());
        }
    }
}
void sendTelemertry(String key, uint16_t value)
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
void sendTelemertry(String key, String value)
{
    DynamicJsonDocument data(512);
    data[key] = value;
    String objectString;
    serializeJson(data, objectString);
    client.publish("v1/devices/me/telemetry", objectString.c_str());
    Serial.println(objectString);
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
void rotary_onButtonClick()
{
    static unsigned long lastTimePressed = 0;
    // ignore multiple press in that time milliseconds
    if (millis() - lastTimePressed < 500)
    {
        return;
    }
    lastTimePressed = millis();
    Serial.print("button pressed ");
    Serial.print(millis());
    Serial.println(" milliseconds after restart");
}

// void rotary_loop()
// {
//     // dont print anything unless value changed
//     if (rotaryEncoder.encoderChanged())
//     {
//         Serial.print("Value: ");
//         Serial.println(rotaryEncoder.readEncoder());
//         Serial.println(rotaryEncoder.getAcceleration());
//     }
//     if (rotaryEncoder.isEncoderButtonClicked(100))
//     {
//         rotary_onButtonClick();
//     }
// }
void updateHMIState()
{
    //   Serial.println(data_set[SET_MODE_DOSING]);
    switch (DWIN)
    {
    case HOME_PAGE:
    {
        data_set[SET_STATR_STOP] = 0;
        flow_sensor[0].vol=0;
        flow_sensor[1].vol=0;
        flow_sensor[2].vol=0;
        flow_sensor[3].vol=0;


        data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_STOP_MODE_FLOW) : dwinShowPage(PAGE_STOP_MODE_TIME);
        if (rotaryEncoder.isEncoderButtonDown())
        {
            unsigned long preTime = millis();
            DWIN = RUN_PAGE;
            while (rotaryEncoder.isEncoderButtonDown())
            {
                if (millis() - preTime > 5000)
                {
                    DWIN = SETING_PAGE;
                    rotaryEncoder.setBoundaries(0, 4, true);
                    rotaryEncoder.setEncoderValue(0);
                    DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
                    delay(10);
                    DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
                    delay(10);
                    DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
                    delay(10);
                    DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
                    // dwinShowPage()
                    break;
                }
            }
        }
        break;
    case SETING_PAGE:
    {

        int pos = rotaryEncoder.readEncoder();
        if (pos == 0)
        {
            //  if(data_set[SET_MODE_TUOI])
            data_set[SET_MODE_TUOI] == DRIP ? dwinShowPage(PAGE_SETING_MODE_TUOI + 1) : dwinShowPage(PAGE_SETING_MODE_TUOI);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setEncoderValue(0);
                rotaryEncoder.setBoundaries(0, 1, true);
                Serial.println("SETING MODE TUOI");
                DWIN = SETING_MODE_TUOI;
            }
        }
        else if (pos == 1)
        {
            data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_SETING_MODE_DOSING) : dwinShowPage(PAGE_SETING_MODE_DOSING + 1);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setEncoderValue(0);
                rotaryEncoder.setBoundaries(0, 1, true);
                Serial.println("SETING MODE DOSING");
                DWIN = SETING_MODE_DOSING;
            }
        }
        else if (pos == 2)
        {
            dwinShowPage(PAGE_SETING_MODE_FLOW);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setEncoderValue(1);
                rotaryEncoder.setBoundaries(0, 4, true);
                Serial.println("SETING MODE FLOW");
                DWIN = SETING_MODE_FLOW;
            }
        }
        else if (pos == 3)
        {
            dwinShowPage(PAGE_SETING_MODE_TIME);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setEncoderValue(1);
                rotaryEncoder.setBoundaries(0, 4, true);
                Serial.println("SETING MODE TIME");
                DWIN = SETING_MODE_TIME;
            }
        }
        else if (pos == 4)
        {
            dwinShowPage(PAGE_SETING_TIME_VALVE);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setEncoderValue(1);
                rotaryEncoder.setBoundaries(0, 12, true);
                Serial.println("SETING TIME");
                DWIN = SETING_TIME;
            }
        }
        if (rotaryEncoder.isEncoderButtonDown())
        {
            unsigned long preTime = millis();
            while (rotaryEncoder.isEncoderButtonDown())
            {
                if (millis() - preTime > 5000)
                {
                    DWIN = HOME_PAGE;
                    saveLocalStore();
                    // rotaryEncoder.setBoundaries(0, 4, true);
                    // rotaryEncoder.setEncoderValue(0);

                    // dwinShowPage()
                    break;
                }
            }
        }

        break;
    }
    case RUN_PAGE:
    {

        static unsigned long timePrint = millis();
        static int _state = 0;

        data_set[FLOW_SENSOR_1] = flow_sensor[0].vol * 100;
        data_set[FLOW_SENSOR_2] = flow_sensor[1].vol * 100;
        data_set[FLOW_SENSOR_3] = flow_sensor[2].vol * 100;
        data_set[FLOW_SENSOR_4] = flow_sensor[3].vol * 100;

        // sendTelemertry("volum_1",data_set[FLOW_SENSOR_1]);
        // sendTelemertry("volum_2",data_set[FLOW_SENSOR_2]);
        // sendTelemertry("volum_3",data_set[FLOW_SENSOR_3]);
        // sendTelemertry("volum_4",data_set[FLOW_SENSOR_4]);
        // int flowA = flow_sensor[0].l_minute * 100;
        // int flowB = flow_sensor[1].l_minute * 100;
        // int flowC = flow_sensor[2].l_minute * 100;
        // int flowD = flow_sensor[3].l_minute * 100;
        // Serial.println("FLOW a: "+(String)flow_sensor[0].flow_l_m + " FLOW B: "+(String)flow_sensor[1].flow_l_m+" FLOW C: "+(String)flow_sensor[2].flow_l_m+" FLOW D: "+(String)flow_sensor[3].flow_l_m);

        if (millis() - timePrint > 1000)
        {
            flow_read(flow_sensor[0], 1);
            flow_read(flow_sensor[1], 2);
            flow_read(flow_sensor[2], 3);
            flow_read(flow_sensor[3], 4);

            DGUS_SendVal(ADDR_SET_FLOW_1, flow_sensor[0].flow_l_m);

            DGUS_SendVal(ADDR_SET_FLOW_2, flow_sensor[1].flow_l_m);

            DGUS_SendVal(ADDR_SET_FLOW_3, flow_sensor[2].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_4, flow_sensor[3].flow_l_m);
            timePrint = millis();
        }

        data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_RUN_MODE_FLOW) : dwinShowPage(PAGE_RUN_MODE_TIME);
        data_set[SET_STATR_STOP] = 1;
        // Serial.println("RUN PGAE");
        if (rotaryEncoder.isEncoderButtonClicked(100))
        {
            flow_sensor[0].vol = 0;
            flow_sensor[1].vol = 0;
            flow_sensor[2].vol = 0;
            flow_sensor[3].vol = 0;
            DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
            delay(10);
            DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
            delay(10);
            DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
            delay(10);
            DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
            DWIN = HOME_PAGE;
        }
        break;
    }
    case SETING_MODE_TUOI:
    {
        int pos = rotaryEncoder.readEncoder();
        dwinShowPage(PAGE_SETING_MODE_TUOI + pos);
        Serial.println(pos);
        if (rotaryEncoder.isEncoderButtonClicked(100))
        {
            data_set[SET_MODE_TUOI] = pos;
            DWIN = SETING_PAGE;
            rotaryEncoder.setBoundaries(0, 4, true);
            rotaryEncoder.setEncoderValue(1);
        }

        break;
    }
    case SETING_MODE_DOSING:
    {
        int pos = rotaryEncoder.readEncoder();
        dwinShowPage(PAGE_SETING_MODE_DOSING + pos);
        if (rotaryEncoder.isEncoderButtonClicked(100))
        {
            data_set[SET_MODE_DOSING] = pos;
            DWIN = SETING_PAGE;
            rotaryEncoder.setBoundaries(0, 4, true);
            rotaryEncoder.setEncoderValue(2);
        }
        break;
    }
    case SETING_MODE_FLOW:
    {
        static int pos = 0;

        switch (pos)
        {
        case 0:
        {
            int temp = rotaryEncoder.readEncoder();
            dwinShowPage(PAGE_SETING_MODE_FLOW + temp);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {

                rotaryEncoder.setBoundaries(0, 9999, true);
                if (temp == 1)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_FLOW_1]);
                    pos = 1;
                }
                else if (temp == 2)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_FLOW_2]);
                    pos = 2;
                }
                else if (temp == 3)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_FLOW_3]);
                    pos = 3;
                }
                else if (temp == 4)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_FLOW_4]);
                    pos = 4;
                }
                else if (temp == 0)
                {
                    DWIN = SETING_PAGE;
                    rotaryEncoder.setBoundaries(0, 4, true);
                    rotaryEncoder.setEncoderValue(3);
                    pos = 0;
                }
            }
            break;
        }
        case 1:
        {
            DGUS_SendVal(ADDR_SET_FLOW_1, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_FLOW_1] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_FLOW_A, data_set[SET_FLOW_1]);

                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(2);
                pos = 0;
            }
            break;
        }
        case 2:
        {
            DGUS_SendVal(ADDR_SET_FLOW_2, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_FLOW_2] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_FLOW_B, data_set[SET_FLOW_2]);

                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(3);
                pos = 0;
            }
            break;
        }
        case 3:
        {
            DGUS_SendVal(ADDR_SET_FLOW_3, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_FLOW_3] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_FLOW_C, data_set[SET_FLOW_3]);

                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(4);
                pos = 0;
            }
            break;
        }
        case 4:
        {
            DGUS_SendVal(ADDR_SET_FLOW_4, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_FLOW_4] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_FLOW_D, data_set[SET_FLOW_4]);

                DWIN = SETING_PAGE;
                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(3);
                pos = 0;
            }
            break;
        }
        }
        break;
    }
    case SETING_MODE_TIME:
    {
        static int pos = 0;

        switch (pos)
        {
        case 0:
        {
            int temp = rotaryEncoder.readEncoder();
            dwinShowPage(PAGE_SETING_MODE_TIME + temp);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {

                rotaryEncoder.setBoundaries(0, 9999, true);
                if (temp == 1)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_1]);
                    pos = 1;
                }
                else if (temp == 2)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_2]);
                    pos = 2;
                }
                else if (temp == 3)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_3]);
                    pos = 3;
                }
                else if (temp == 4)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_4]);
                    pos = 4;
                }
                else if (temp == 0)
                {
                    DWIN = SETING_PAGE;
                    rotaryEncoder.setBoundaries(0, 4, true);
                    rotaryEncoder.setEncoderValue(4);
                    pos = 0;
                }
            }
            break;
        }
        case 1:
        {
            DGUS_SendVal(ADDR_SET_TIME_1, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_1] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_A, data_set[SET_TIME_VALVE_1]);
                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(2);
                pos = 0;
            }
            break;
        }
        case 2:
        {
            DGUS_SendVal(ADDR_SET_TIME_2, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_2] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_B, data_set[SET_TIME_VALVE_2]);
                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(3);
                pos = 0;
            }
            break;
        }
        case 3:
        {
            DGUS_SendVal(ADDR_SET_TIME_3, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_3] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_C, data_set[SET_TIME_VALVE_3]);
                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(4);
                pos = 0;
            }
            break;
        }
        case 4:
        {
            DGUS_SendVal(ADDR_SET_TIME_4, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_4] = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_D, data_set[SET_TIME_VALVE_4]);
                DWIN = SETING_PAGE;
                rotaryEncoder.setBoundaries(0, 4, true);
                rotaryEncoder.setEncoderValue(4);
                pos = 0;
            }
            break;
        }
        }
        break;
    }
    case SETING_TIME:
    {
        static int pos = 0;

        switch (pos)
        {
        case 0:
        {
            int temp = rotaryEncoder.readEncoder();
            dwinShowPage(PAGE_SETING_TIME_VALVE + temp);
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                rotaryEncoder.setBoundaries(0, 9999, true);
                if (temp == 1)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_PUMP_MASTER]);
                    pos = temp;
                }
                else if (temp == 2)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_PUMP_SLAVE]);
                    pos = temp;
                }
                else if (temp == 3)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_1]);
                    pos = temp;
                }
                else if (temp == 4)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_2]);
                    pos = temp;
                }
                else if (temp == 5)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_3]);
                    pos = temp;
                }
                else if (temp == 6)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_4]);
                    pos = temp;
                }
                else if (temp == 7)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_5]);
                    pos = temp;
                }
                else if (temp == 8)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_6]);
                    pos = temp;
                }
                else if (temp == 9)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_7]);
                    pos = temp;
                }
                else if (temp == 10)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_8]);
                    pos = temp;
                }
                else if (temp == 11)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_9]);
                    pos = temp;
                }
                else if (temp == 12)
                {
                    rotaryEncoder.setEncoderValue(data_set[SET_TIME_VALVE_TUOI_10]);
                    pos = temp;
                }
                else
                {
                    DWIN = SETING_PAGE;
                    rotaryEncoder.setBoundaries(0, 4, true);
                    rotaryEncoder.setEncoderValue(1);
                    pos = 0;
                }
            }
            break;
        }
        case 1:
        {
            DGUS_SendVal(ADDR_SET_TIME_PUMP_MASTER, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_PUMP_MASTER] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(2);
                pos = 0;
            }
            break;
        }
        case 2:
        {
            DGUS_SendVal(ADDR_SET_TIME_PUMP_SLAVE, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_PUMP_SLAVE] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(3);
                pos = 0;
            }
            break;
        }
        case 3:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_1, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_1] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(4);
                pos = 0;
            }
            break;
        }
        case 4:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_2, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_2] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(5);
                pos = 0;
            }
            break;
        }
        case 5:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_3, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_3] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(6);
                pos = 0;
            }
            break;
        }
        case 6:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_4, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_4] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(7);
                pos = 0;
            }
            break;
        }
        case 7:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_5, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_5] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(8);
                pos = 0;
            }
            break;
        }
        case 8:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_6, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_6] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(9);
                pos = 0;
            }
            break;
        }
        case 9:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_7, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_7] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(10);
                pos = 0;
            }
            break;
        }
        case 10:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_8, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_8] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(11);
                pos = 0;
            }
            break;
        }
        case 11:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_9, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_9] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(12);
                pos = 0;
            }
            break;
        }

        case 12:
        {
            DGUS_SendVal(ADDR_SET_VAN_TUOI_10, rotaryEncoder.readEncoder());
            if (rotaryEncoder.isEncoderButtonClicked(100))
            {
                data_set[SET_TIME_VALVE_TUOI_10] = rotaryEncoder.readEncoder();
                rotaryEncoder.setBoundaries(0, 12, true);
                rotaryEncoder.setEncoderValue(0);
                pos = 0;
            }
            break;
        }
        }
        break;
    }
    }
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
void DGUS_SendVal(int iAdr, int iVal) // Send iVal for VP= iAdr to DGUS
{
    // static unsigned long timeSend=millis();
    // if(millis()-timeSend>100){
    byte bAdrL, bAdrH, bValL, bValH;
    bAdrL = iAdr & 0xFF;
    bAdrH = (iAdr >> 8) & 0xFF;
    bValL = iVal & 0xFF;
    bValH = (iVal >> 8) & 0xFF;
    DWIN_LCD.write(0x5A);
    DWIN_LCD.write(0xA5);
    DWIN_LCD.write(0x05);
    DWIN_LCD.write(0x82);
    DWIN_LCD.write(bAdrH);
    DWIN_LCD.write(bAdrL);
    DWIN_LCD.write(bValH);
    DWIN_LCD.write(bValL);
    // timeSend=millis();
    // }
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
void saveLocalStore()
{
    DynamicJsonDocument data(2048);
    data[KEY_SET_TIME_A] = data_set[SET_TIME_VALVE_1];
    data[KEY_SET_TIME_B] = data_set[SET_TIME_VALVE_2];
    data[KEY_SET_TIME_C] = data_set[SET_TIME_VALVE_3];
    data[KEY_SET_TIME_D] = data_set[SET_TIME_VALVE_4];
    data[KEY_SET_FLOW_A] = data_set[SET_FLOW_1];
    data[KEY_SET_FLOW_B] = data_set[SET_FLOW_2];
    data[KEY_SET_FLOW_C] = data_set[SET_FLOW_3];
    data[KEY_SET_FLOW_D] = data_set[SET_FLOW_4];
    data[KEY_SET_TIME_TUOI_1] = data_set[SET_TIME_VALVE_TUOI_1];
    data[KEY_SET_TIME_TUOI_2] = data_set[SET_TIME_VALVE_TUOI_2];
    data[KEY_SET_TIME_TUOI_3] = data_set[SET_TIME_VALVE_TUOI_3];
    data[KEY_SET_TIME_TUOI_4] = data_set[SET_TIME_VALVE_TUOI_4];
    data[KEY_SET_TIME_TUOI_5] = data_set[SET_TIME_VALVE_TUOI_5];
    data[KEY_SET_TIME_TUOI_6] = data_set[SET_TIME_VALVE_TUOI_6];
    data[KEY_SET_TIME_TUOI_7] = data_set[SET_TIME_VALVE_TUOI_7];
    data[KEY_SET_TIME_TUOI_8] = data_set[SET_TIME_VALVE_TUOI_8];
    data[KEY_SET_TIME_TUOI_9] = data_set[SET_TIME_VALVE_TUOI_9];
    data[KEY_SET_TIME_TUOI_10] = data_set[SET_TIME_VALVE_TUOI_10];
    data[KEY_SET_MODE_DOSING] = data_set[SET_MODE_DOSING];
    data[KEY_SET_MODE_TUOI] = data_set[SET_MODE_TUOI];
    data[KEY_SET_HIGH_PRESURE] = set_high_presure;
    data[KEY_SET_LOW_PRESURE] = set_low_presure;
    String objectString;
    serializeJson(data, objectString);
    writeFile(LITTLEFS, PATH_STORAGE, objectString.c_str());
    Serial.println("SAVE FILE: " + (String)objectString);
}
void localStorageExport()
{
    String data = readFile(LITTLEFS, PATH_STORAGE);
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
        data_set[SET_TIME_VALVE_1] = doc[KEY_SET_TIME_A];
        data_set[SET_TIME_VALVE_2] = doc[KEY_SET_TIME_B];
        data_set[SET_TIME_VALVE_3] = doc[KEY_SET_TIME_C];
        data_set[SET_TIME_VALVE_4] = doc[KEY_SET_TIME_D];

        data_set[SET_FLOW_1] = doc[KEY_SET_FLOW_A];
        data_set[SET_FLOW_2] = doc[KEY_SET_FLOW_B];
        data_set[SET_FLOW_3] = doc[KEY_SET_FLOW_C];
        data_set[SET_FLOW_4] = doc[KEY_SET_FLOW_D];

        data_set[SET_TIME_VALVE_TUOI_1] = doc[KEY_SET_TIME_TUOI_1];
        data_set[SET_TIME_VALVE_TUOI_2] = doc[KEY_SET_TIME_TUOI_2];
        data_set[SET_TIME_VALVE_TUOI_3] = doc[KEY_SET_TIME_TUOI_3];
        data_set[SET_TIME_VALVE_TUOI_4] = doc[KEY_SET_TIME_TUOI_4];
        data_set[SET_TIME_VALVE_TUOI_5] = doc[KEY_SET_TIME_TUOI_5];
        data_set[SET_TIME_VALVE_TUOI_6] = doc[KEY_SET_TIME_TUOI_6];
        data_set[SET_TIME_VALVE_TUOI_7] = doc[KEY_SET_TIME_TUOI_7];
        data_set[SET_TIME_VALVE_TUOI_8] = doc[KEY_SET_TIME_TUOI_8];
        data_set[SET_TIME_VALVE_TUOI_9] = doc[KEY_SET_TIME_TUOI_9];
        data_set[SET_TIME_VALVE_TUOI_10] = doc[KEY_SET_TIME_TUOI_10];

        data_set[SET_MODE_DOSING] = doc[KEY_SET_MODE_DOSING];
        data_set[SET_MODE_TUOI] = doc[KEY_SET_MODE_TUOI];
        set_high_presure = doc[KEY_SET_HIGH_PRESURE].as<float>();
        set_low_presure = doc[KEY_SET_LOW_PRESURE].as<float>();

        Serial.println("Export data success!!");
    }
}
void flow_read(flow_sen &flow_sensor, int num)
{

    if (flow_sensor.flow_frequency != 0)
    {
        // Serial.print("SENSOR " + (String)num + " : ");
        flow_sensor.l_minute = (flow_sensor.flow_frequency / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
        flow_sensor.flow_l_m = flow_sensor.l_minute * 1000;
        // Serial.print("Rate: ");
        // Serial.print(flow_sensor.l_minute);
        // Serial.print(" L/M");
        flow_sensor.l_minute = flow_sensor.l_minute / 60;
        flow_sensor.vol = flow_sensor.vol + flow_sensor.l_minute;
        // Serial.print("Vol:");
        // Serial.print(flow_sensor.vol);
        // Serial.println(" L");
        //flow_sensor.l_minute=flow_sensor.l_minute*10;
        flow_sensor.flow_frequency = 0; // Reset Counter
    }
    else
    {
        // Serial.println("SENSOR " + (String)num + " : ");
        flow_sensor.flow_l_m = 0;
        // Serial.print(" flow rate = 0 ");
        // Serial.print("Rate: ");
        // Serial.print(flow_sensor.flow_frequency);
        // Serial.print(" L/M");
        // Serial.print("Vol:");
        // Serial.print(flow_sensor.vol);
        // Serial.print(" L");
        // Serial.println("");
    }
}

float readPresureSensor()
{
    static float out_value_lowpass=0;
    float adc=analogRead(ADC_VP);
    float estimated_value = adc_fillter.updateEstimate(adc);
    float presure = 0;
   // Serial.println((String)estimated_value+",");

    //Serial.println("estimated: " + (String)estimated_value);
    presure = mapfloat(estimated_value, 186.68, 1848.06, 0.0, 10.0);
    out_value_lowpass=0.8*out_value_lowpass+(1-0.8)*presure;
  //  Serial.println((String)adc+","+ (String)estimated_value+","+ String(presure)+","+(String)out_value_lowpass);
    if (out_value_lowpass < 0)
    {
        out_value_lowpass = 0;
    }
    // Serial.println("current: " + (String)presure);
    return out_value_lowpass;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
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
void OTA_init()
{

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Hi! This is a sample response."); });

    AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
    server.begin();
}