
// RL1 bơm
// RL2
// RL3 van_a
// RL4 vanb
// RL5 vanc
// RL6 vand

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
#include <SPIFFS.h>
#include "AiEsp32RotaryEncoder.h"
#include "SoftwareSerial.h"
#include "ModbusIP_ESP8266.h"
#define INTERRUPT_ATTR IRAM_ATTR

#include "NTPClient.h"
#include "WiFiUdp.h"
#include "SimpleKalmanFilter.h"
#include <ML_ModbusRtuMaster.h>
#define FLOW_CALIBRATION 8.2
#define TIME_CHECK_WIFI 2000
#define TIME_OFFSET 60000
HardwareSerial DWIN_LCD(1);
SimpleKalmanFilter adc_fillter(2, 2, 0.001);
int home_seting_page[5] = {6, 8, 10, 15, 20};
#define MAX_ACTION 50
#define PLC_ID 1

#define MAX_OUTPUT 20

#define SLAVE_ID 1
// #define FLOW_SR_1 35
// #define FLOW_SR_2 32
#define digitalInPin 33
int digitalValue = 0;
// #define FLOW_SR_4 4
#define ADC_VP 36

#define TUOI_PHAN 0
#define TUOI_NUOC 1

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

#define PATH_CONFIG "/config.json"
#define PATH_STORAGE_STATE "/localStorage.json"
#define PATH_TIMER "/timer.json"
#define PATH_FIRM_INFOR "/firmware_infor.json"

#define KEY_TIME_TUOI_NUOC "time_tuoi_nuoc"
#define KEY_TIME_TUOI_PHAN "time_tuoi_phan"
#define KEY_TIME_TOGGLE "time_cha_toggle"

#define Power_er "power_er"

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

#define POWER_1 "power_1"
#define POWER_2 "power_2"
#define POWER_3 "power_3"
#define POWER_4 "power_4"
#define POWER_5 "power_5"
#define POWER_6 "power_6"

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
#define KEY_POWER_12 "set_power_12"
#define KEY_POWER_13 "set_power_13"
#define KEY_POWER_14 "set_power_14"
#define KEY_POWER_15 "set_power_15"
#define KEY_POWER_16 "set_power_16"
#define KEY_POWER_17 "set_power_17"

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

#define van_a 2
#define van_b 3
#define van_c 4
#define van_d 5
bool flag_save = false;
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "UH0nmY05l2XXQWlQYC1A"
#define ID "b309bf80-66f3-11ed-99cb-4de8ebde04d6"
// #define TOKEN "mQECxIw9xvGh1PxpKTFq"
// #define ID "ae9b6ce0-5f74-11ed-99cb-4de8ebde04d6"
AsyncWebServer server(80);
ModbusRTU mb;
ModbusIP mbIP;
WiFiClient espClient;
PubSubClient client(espClient);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
int time_tuoi_nuoc = 100;
IPAddress local_ip(192, 168, 1, 134);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (8, 8, 4, 4);
static bool eth_connected = false;
bool pump_chinh = false;
float set_high_presure = 0;
float set_low_presure = 0;
bool state_van_a = false;
bool state_van_b = false;
bool state_van_c = false;
unsigned long previousMillis = 0;
bool state_van_d = false;
bool power[20];
bool power1[10];

unsigned long timeOffVan = 0;
bool preStatePump = false;
int stateOffSys = 0;

bool autoby = false;
bool state_system = false;
uint16_t flow_ip[4];
int time_tuoi_phan = 100;
int power_er = 0;
// int time_tuoi_toggle =100;
IPAddress remote(192, 168, 4, 4);
int time_cha_toggle = 100;
uint16_t data_set[30];
const char *ssid = "THANH-1";
const char *pass = "0964190692";
bool preCoilsState[20];
// const char *ssid = "DalatTech";
// const char *pass = "mltech@2019";

// struct  van_control
// {
//  unsigned long preTime
// };

//================Nhan sua
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
//

struct valve_control
{
    unsigned long lastTime;
    bool pre_state;
    bool now_state;
    int time_set;
} vale_control[4];
struct van_tuoi
{
    unsigned long lastTime;
    bool pre_state;
    bool new_state;
    int timeSet;
} valve_tuoi[10];

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

typedef enum
{
    IDLE_STATE,
    START_STATE,
    STOP_STATE,
    PHAN_STATE,
    NUOC_STATE
} state;
state dosing_state;
void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
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

void wifiAP();
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
bool mode_tuoi = false;
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
void sendTelemertry(String key, int value);

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
//////////////////////
void NTPInit();
void timeInit();
void printLocalTime();
void NTPTimeUpdate();
void jsonObjectTimer(String jsonString);
void timerInit();
void updateTelemertry();
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
String httpGETRequest(const char *serverName);
/////////////////////////////////////////
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const long gmtOffset_sec = +6 * 60 * 60;
const int daylightOffset_sec = 3600;
uint8_t number_of_timers = 0;
uint16_t IP_FLOW[4];
//////////////////////////////
struct timer
{
    int id;
    bool enable;
    String time;
    String interval;
    String keyName[30];
    bool state[30];
    int value[10];
};
timer TIMER[100];
//////////////////////////////////////////
void appInit()
{
    delay(5000);

    Serial.begin(115200);
    Serial2.begin(9600);

    DWIN_LCD.begin(115200, SERIAL_8N1, 12, 2);

    fileInit();
    // wifiAP();
    // ETHInit();
    //  pinMode(FLOW_SR_1, INPUT);
    //  pinMode(FLOW_SR_2, INPUT);
    //  pinMode(FLOW_SR_3, INPUT);
    //  pinMode(FLOW_SR_4, INPUT);
    //  pinMode(ADC_VP, INPUT);
    //  analogSetAttenuation(ADC_6db);

    // Serial.println("dâkfa");
    WiFi.begin(ssid, pass);
    if (!WiFi.config(local_ip, gateway, subnet, dns1, dns2))
    {
        Serial.println("STA Failed to configure");
    }

    // Connect to Wi-Fi network with SSID and password
    // Serial.print("Connecting to ");
    // Serial.println(ssid);
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(500);
    //     Serial.print(".");
    // }
    // // Print local IP address and start web server
    // Serial.println("");
    // Serial.println("WiFi connected.");/
    // Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());
    server.begin();
    //  createDir(SPIFFS, "/data");
    // saveLocalStore();
    mbIP.server();
    mbIP.addCoil(0, false, 20);
    mbIP.addHreg(0, 0, 10);
    mb.addHreg(0, 0, 10);
    localStorageExport();
    NTPInit();

    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    bool circleValues = false;
    rotaryEncoder.setAcceleration(0);
    mqttInit();

    ML_RtuMaster.begin(&Serial2);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
    delay(10);
    DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_1, vale_control[0].time_set);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_2, vale_control[1].time_set);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_3, vale_control[2].time_set);
    delay(10);
    DGUS_SendVal(ADDR_SET_TIME_4, vale_control[3].time_set);
    delay(10);
    data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_STOP_MODE_FLOW) : dwinShowPage(PAGE_STOP_MODE_TIME);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_1, valve_tuoi[0].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_2, valve_tuoi[1].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_3, valve_tuoi[2].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_4, valve_tuoi[3].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_5, valve_tuoi[4].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_6, valve_tuoi[5].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_7, valve_tuoi[6].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_8, valve_tuoi[7].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_9, valve_tuoi[8].timeSet);
    delay(10);
    DGUS_SendVal(ADDR_SET_VAN_TUOI_10, valve_tuoi[9].timeSet);
}
int power_err = 1;
void appRun()
{
    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= 2000))
    {
        Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillis = currentMillis;
    }
    // Serial.println("ghello");
    // static bool testMB=true;
    //       if (mbIP.isConnected(remote)) {   // Check if connection to Modbus Slave is established
    //         mbIP.readHreg(remote, 0,flow_ip,4);
    //         mbIP.writeCoil(remote,0,testMB);
    //         testMB=!testMB;

    //         Serial.println(flow_ip[0]);  // Initiate Read Coil from Modbus Slave
    //       } else {
    //         mbIP.connect(remote);           // Try to connect if no connection
    //       }
    updateSchedule();
    static unsigned long preTime = millis();
    // if (millis() - preTime > 1000)
    // {
    //     digitalValue = digitalRead(digitalInPin);
    //     Serial.print("digital = ");
    //     Serial.print(digitalValue);
    //     preTime = millis();
    // }
    // wait 2 milliseconds before the next loop for the analog-to-digital
    // wifiConnect();
    // NTPTimeUpdate();
    // static unsigned long preTime = millis();
    if (millis() - preTime > 100)
    {
        presure_value = readPresureSensor();
        DGUS_SendVal(ADDR_PRESURE_VALUE, presure_value * 10);
        preTime = millis();
    }
    updateHMIState();

    updateState();
    updateTelemertry();
    if (WiFi.isConnected() || eth_connected)
    {
        mqttLoop();
    }
    mbIP.task();
}
void updateState()
{

    static uint16_t flow_data[5];
    // mb.readHreg(2,0,1);
    static unsigned long preTime = millis();
    static int state_modbus = 0;

    // mbIP.writeCoil(remote,0,power)
    if (millis() - preTime >= 10)
    {
        state_modbus++;
        switch (state_modbus)
        {
        case 17: // 500 ms 1 la
        {
            static bool pump_err = false;
            static bool pre_pump_err = false;
            ML_RtuMaster.readHoldingRegisters(2, 0, 5, flow_data);
            for (int i = 0; i < 4; i++)
            {
                flow_sensor[i].flow_l_m = flow_data[i] * 10;
                //   Serial.println("Flow " + (String)i + " : " + (String)flow_sensor[i].flow_l_m);
            }
            power_err = flow_data[4]; /// trang thai bom loi
            if (power_err == 0)
            {
                pump_err = true;
                if (pump_err != pre_pump_err)
                {
                    sendTelemertry(Power_er, pump_err);
                    pre_pump_err = pump_err;
                }

                // state_system = false;
                //  sendTelemertry(KEY_POWER_DOSING, state_system);
                //  power1[5] = false;

                // for (int i = 0; i < 4; i++)
                // {
                //     vale_control[i].pre_state = false;
                // }
            }
            else
            {
                pump_err = false;
                if (pump_err != pre_pump_err)
                {
                    sendTelemertry(Power_er, pump_err);
                    pre_pump_err = pump_err;
                }
            }
            preTime = millis();
            state_modbus = 0;
            break;
        }
        case 1:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_a, vale_control[0].pre_state);
            break;
        }
        case 2:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_b, vale_control[1].pre_state);
            break;
        }
        case 3:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_c, vale_control[2].pre_state);
            break;
        }
        case 4:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_d, vale_control[3].pre_state);
            break;
        }
        case 5:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, power1[5]);
            break;
        }
        case 6:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 6, power[0]);
            break;
        }
        case 7:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 7, power[1]);
            break;
        }
        case 8:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 8, power[2]);
            break;
        }
        case 9:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 9, power[3]);
            break;
        }
        case 10:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 10, power[4]);
            break;
        }
        case 11:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 11, power[5]);
            break;
        }
        case 12:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 12, power[6]);
            break;
        }
        case 13:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 13, power[7]);
            break;
        }
        case 14:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 14, power[8]);
            break;
        }
        case 15:
        {
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, 15, power[9]);
            break;
        }

            // }
        }
    }

    switch (stateOffSys)
    {
    case 0:
    {
        if (power1[5] != preStatePump)
        {
            preStatePump = power1[5];
            if (power1[5] == false)
            {
                timeOffVan = millis();
                stateOffSys = 1;
            }
        }
        break;
    }
    case 1:
    {
        if (millis() - timeOffVan >= 5000)
        {
            Serial.println("off van");
            for (int i = 0; i < 16; i++)
            {
                power[i] = false;
                // mbIP.Coil(i, false);
                stateOffSys = 0;
            }
        }
        break;
    }
    }
    for (int i = 0; i < 16; i++)
    {
        String key = "set_power_" + (String)(i + 1);
        if (preCoilsState[i] != power[i])
        {
            preCoilsState[i] = power[i];
            sendTelemertry(key, preCoilsState[i]);
        }
    }

    //     if (millis() - preTime > 500)
    //     {
    //         Serial.println("TIME OUT TASK 2");
    //         preTime = millis();
    //         state_modbus++;
    //     }
    //     break;
    // }
    // case 2:
    // {
    //     if (millis() - preTime > 100)
    //     {
    //         Serial.println("TIME OUT TASK waitr");
    //         preTime = millis();
    //         state_modbus = 0;
    //     }
    //     break;
    // }
    // }

    // if (flag_save == true)
    // {
    //     saveLocalStore();
    //     flag_save = false;
    // }
    // static unsigned long pre_time_tuoi_nuoc = millis();
    static unsigned long pre_time_tuoi_phan = millis();
    static unsigned long timeStart = millis();

    switch (dosing_state)
    {
    case IDLE_STATE:
    {
        // Serial.println("IDLE STATE");
        if (state_system == true)
        {

            // for (int i = 0; i < 10; i++)
            // {
            //     ML_RtuMaster.writeSingleCoil(SLAVE_ID, i + 6, power[i]);
            // }
            if (state_system == true)
            {
                //       Serial.println("start");
                timeStart = millis();
                dosing_state = START_STATE;
            }
        }
        break;
    }
    case START_STATE:
    {
        // Serial.println("STATRT STATE");

        power1[5] = true;
        // ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, power1[5]);
        mode_tuoi = TUOI_PHAN;
        if (mode_tuoi == TUOI_PHAN)
        {

            pre_time_tuoi_phan = millis();

            if (millis() - timeStart > 10000)
            {
                for (int i = 0; i < 4; i++)
                {

                    // vale_control[i].pre_state == true ? vale_control[i].pre_state = false : vale_control[i].pre_state = true;
                    vale_control[i].time_set == 0 ? vale_control[i].pre_state = 0 : false;
                    vale_control[i].pre_state = true;
                    // ML_RtuMaster.writeSingleCoil(SLAVE_ID, i + 2, vale_control[i].pre_state);
                    vale_control[i].lastTime = millis();
                }
                dosing_state = PHAN_STATE;
            }
        }
        DWIN = RUN_PAGE;
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
        // ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, power1[5]);

        for (int i = 0; i < 4; i++)
        {
            if (millis() - vale_control[i].lastTime >= vale_control[i].time_set * TIME_OFFSET)
            {

                // vale_control[i].pre_state == true ? vale_control[i].pre_state = false : vale_control[i].pre_state = true;
                vale_control[i].time_set == 0 ? vale_control[i].pre_state = 0 : false;

                vale_control[i].pre_state = false;
                // ML_RtuMaster.writeSingleCoil(SLAVE_ID, i + 2, vale_control[i].pre_state);
            }
        }

        if (millis() - pre_time_tuoi_phan >= time_tuoi_phan * TIME_OFFSET || mode_tuoi == TUOI_NUOC || state_system == false)
        {
            //    Serial.println("BREAK OUT PHANC_STATE");
            state_system = false;
            sendTelemertry(KEY_POWER_DOSING, state_system);
            power1[5] = false;
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, false);
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_a, false);
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_b, false);
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_c, false);
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_d, false);
            // sendTelemertry(POWER_1, cha);
            // sendTelemertry(POWER_2, chb);
            // sendTelemertry(POWER_3, chc);
            // sendTelemertry(POWER_4, chd);
            // sendTelemertry(POWER_5, pump);
            for (int i = 0; i < 4; i++)
            {
                vale_control[i].pre_state = false;
            }
            pre_value = false;
            dosing_state = IDLE_STATE;
            sendTelemertry(KEY_POWER_DOSING, state_system);
            DWIN = HOME_PAGE;
        }
        break;
    }
    case NUOC_STATE:
    {

        // for (int i = 2; i <= 5; i++)
        // {
        //     ML_RtuMaster.writeSingleCoil(SLAVE_ID, i, false);
        // }
        // // sendTelemertry(POWER_1, cha);
        // // sendTelemertry(POWER_2, chb);
        // // sendTelemertry(POWER_3, chc);
        // // sendTelemertry(POWER_4, chd);
        // ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, true);

        // // sendTelemertry(POWER_5, pump);
        // if (millis() - pre_time_tuoi_nuoc >= time_tuoi_nuoc * TIME_OFFSET || mode_tuoi == TUOI_PHAN || state_system == false)
        // {
        //     Serial.println("BREAK OUT NUOC_STATE");
        //     state_system = false;
        //     sendTelemertry(KEY_POWER_DOSING, state_system);
        //     power1[5] = false;
        //     // for(int i=0;i<10;i++){
        //     //     power[i]=false;
        //     // }
        //     ML_RtuMaster.writeSingleCoil(SLAVE_ID, 0, false);
        //     dosing_state = IDLE_STATE;
        //     DWIN = HOME_PAGE;
        // }
        break;
    }
    }

    // static unsigned long preTime = millis();
    // if (millis() - preTime > 10)
    // {
    // }
}
void updateTelemertry()
{
    static bool prePump = false;
    static unsigned long timeSendFlow = 0;
    if (prePump != power1[5])
    {
        prePump = power1[5];
        sendTelemertry(POWER_5, power1[5]);
    }
    if ((flow_sensor[0].flow_l_m > 0) || flow_sensor[1].flow_l_m > 0 || flow_sensor[2].flow_l_m > 0 || (flow_sensor[3].flow_l_m > 0))
    {
        if (millis() - timeSendFlow > 5000)
        {

            for (int i = 0; i < 4; i++)
            {
                String key = "flow_" + (String)(i + 1);
                sendTelemertry(key, flow_sensor[i].flow_l_m);
            }
            timeSendFlow = millis();
        }
    }
    for (int i = 0; i < 4; i++)
    {
        String key = "power_" + (String)(i + 1);

        if (vale_control[i].pre_state != vale_control[i].now_state)
        {
            vale_control[i].now_state = vale_control[i].pre_state;
            sendTelemertry(key, vale_control[i].pre_state);
        }
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
    StaticJsonDocument<512> doc;
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

        if (doc["params"].containsKey(KEY_TIME_TUOI_NUOC))
        {
            time_tuoi_nuoc = doc["params"][KEY_TIME_TUOI_NUOC].as<int>();
            sendTelemertry(KEY_TIME_TUOI_NUOC, time_tuoi_nuoc);
        }
        if (doc["params"].containsKey(KEY_TIME_TUOI_PHAN))
        {
            time_tuoi_phan = doc["params"][KEY_TIME_TUOI_PHAN].as<int>();
            sendTelemertry(KEY_TIME_TUOI_PHAN, time_tuoi_phan);
        }
        // if (doc["params"].containsKey(Power_er))
        // {
        //     power_err = doc["params"][Power_er].as<int>();
        //     sendTelemertry(Power_er, power_err);
        // }
        // if (doc["params"].containsKey(KEY_TIME_TOGGLE))
        // {
        //     time_cha_toggle = doc["params"][KEY_TIME_TOGGLE].as<int>();
        //     time_cha_toggle < 500 ? time_cha_toggle = 500 : time_cha_toggle;
        //     sendTelemertry(KEY_TIME_TOGGLE, time_cha_toggle);
        // }

        if (doc["params"].containsKey(KEY_SET_TIME_A))
        {
            vale_control[0].time_set = doc["params"][KEY_SET_TIME_A].as<int>();
            sendTelemertry(KEY_SET_TIME_A, vale_control[0].time_set);
            DGUS_SendVal(ADDR_SET_TIME_1, vale_control[0].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_B))
        {
            vale_control[1].time_set = doc["params"][KEY_SET_TIME_B].as<int>();
            sendTelemertry(KEY_SET_TIME_B, vale_control[1].time_set);
            DGUS_SendVal(ADDR_SET_TIME_2, vale_control[1].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_C))
        {
            vale_control[2].time_set = doc["params"][KEY_SET_TIME_C].as<int>();
            sendTelemertry(KEY_SET_TIME_C, vale_control[2].time_set);
            DGUS_SendVal(ADDR_SET_TIME_3, vale_control[2].time_set);
        }
        if (doc["params"].containsKey(KEY_SET_TIME_D))
        {
            vale_control[3].time_set = doc["params"][KEY_SET_TIME_D].as<int>();
            sendTelemertry(KEY_SET_TIME_D, vale_control[3].time_set);
            DGUS_SendVal(ADDR_SET_TIME_4, vale_control[3].time_set);
        }
        // if (doc["params"].containsKey(KEY_SET_FLOW_A))
        // {
        //     data_set[SET_FLOW_1] = doc["params"][KEY_SET_FLOW_A].as<int>();
        //     sendTelemertry(KEY_SET_FLOW_A, data_set[SET_FLOW_1]);
        //     DGUS_SendVal(ADDR_SET_FLOW_1, data_set[SET_FLOW_1]);
        // }
        // if (doc["params"].containsKey(KEY_SET_FLOW_B))
        // {
        //     data_set[SET_FLOW_2] = doc["params"][KEY_SET_FLOW_B].as<int>();
        //     sendTelemertry(KEY_SET_FLOW_B, data_set[SET_FLOW_2]);
        //     DGUS_SendVal(ADDR_SET_FLOW_2, data_set[SET_FLOW_2]);
        // }
        // if (doc["params"].containsKey(KEY_SET_FLOW_C))
        // {
        //     data_set[SET_FLOW_3] = doc["params"][KEY_SET_FLOW_C].as<int>();
        //     sendTelemertry(KEY_SET_FLOW_C, data_set[SET_FLOW_3]);
        //     DGUS_SendVal(ADDR_SET_FLOW_3, data_set[SET_FLOW_3]);
        // }
        // if (doc["params"].containsKey(KEY_SET_FLOW_D))
        // {
        //     data_set[SET_FLOW_4] = doc["params"][KEY_SET_FLOW_D].as<int>();
        //     sendTelemertry(KEY_SET_FLOW_D, data_set[SET_FLOW_4]);
        //     DGUS_SendVal(ADDR_SET_FLOW_4, data_set[SET_FLOW_4]);
        // }
        if (doc["params"].containsKey(KEY_POWER_DOSING))
        {
            state_system = doc["params"][KEY_POWER_DOSING].as<bool>();
            state_system ? sendTelemertry(KEY_POWER_DOSING, true) : sendTelemertry(KEY_POWER_DOSING, false);
            state_system ? DWIN = RUN_PAGE : DWIN = HOME_PAGE;
        }

        if (doc["params"].containsKey(POWER_1))
        {
            vale_control[0].pre_state = doc["params"][POWER_1].as<bool>();
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_a, vale_control[0].pre_state);
            sendTelemertry(POWER_1, vale_control[0].pre_state);
        }
        if (doc["params"].containsKey(POWER_2))
        {
            vale_control[1].pre_state = doc["params"][POWER_2].as<bool>();
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_b, vale_control[1].pre_state);
            sendTelemertry(POWER_2, vale_control[1].pre_state);
        }
        if (doc["params"].containsKey(POWER_3))
        {
            vale_control[2].pre_state = doc["params"][POWER_3].as<bool>();
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_c, vale_control[2].pre_state);

            sendTelemertry(POWER_3, vale_control[2].pre_state);
        }
        if (doc["params"].containsKey(POWER_4))
        {
            vale_control[3].pre_state = doc["params"][POWER_4].as<bool>();
            ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_d, vale_control[3].pre_state);

            sendTelemertry(POWER_4, vale_control[3].pre_state);
        }
        if (doc["params"].containsKey(POWER_5))
        {
            power1[5] = doc["params"][POWER_5].as<bool>();
            sendTelemertry(POWER_5, power1[5]);
            // if (power1[5] == false)
            // {
            //     for (int i = 0; i < 10; i++)
            //     {
            //         power[i] = false;
            //     }
            // }
        }
        if (doc["params"].containsKey(POWER_6))
        {
            autoby = doc["params"][POWER_6].as<bool>();
            // ML_RtuMaster.writeSingleCoil(SLAVE_ID, van_a, autoby);
            sendTelemertry(POWER_6, autoby);
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
            mbIP.Coil(0, power[10]);
            sendTelemertry(KEY_POWER_11, power[10]);
        }
        if (doc["params"].containsKey(KEY_POWER_12))
        {
            power[11] = doc["params"][KEY_POWER_12].as<bool>();
            mbIP.Coil(1, power[11]);
            sendTelemertry(KEY_POWER_12, power[11]);
        }
        if (doc["params"].containsKey(KEY_POWER_13))
        {
            power[12] = doc["params"][KEY_POWER_13].as<bool>();
            mbIP.Coil(2, power[12]);
            sendTelemertry(KEY_POWER_13, power[12]);
        }
        if (doc["params"].containsKey(KEY_POWER_14))
        {
            power[13] = doc["params"][KEY_POWER_14].as<bool>();
            mbIP.Coil(3, power[13]);
            sendTelemertry(KEY_POWER_14, power[13]);
        }
        if (doc["params"].containsKey(KEY_POWER_15))
        {
            power[14] = doc["params"][KEY_POWER_15].as<bool>();
            mbIP.Coil(4, power[14]);
            sendTelemertry(KEY_POWER_15, power[14]);
        }
        if (doc["params"].containsKey(KEY_POWER_16))
        {
            power[15] = doc["params"][KEY_POWER_16].as<bool>();
            mbIP.Coil(5, power[15]);
            sendTelemertry(KEY_POWER_16, power[15]);
        }
        if (doc["params"].containsKey(KEY_POWER_17))
        {
            power[16] = doc["params"][KEY_POWER_17].as<bool>();

            sendTelemertry(KEY_POWER_17, power[16]);
        }
        if (doc["params"].containsKey(KEY_SET_MODE_TUOI))
        {
            mode_tuoi = doc["params"][KEY_SET_MODE_TUOI].as<bool>();
            sendTelemertry(KEY_SET_MODE_TUOI, (bool)mode_tuoi);
        }
        saveLocalStore();
    }
    else if (strstr((char *)payload, "update_schedule") != NULL)
    {
        // jsonObjectTimer(httpGETRequest(serverName.c_str()));
        httpGETRequest(serverName.c_str());
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
            sendTelemertry(KEY_SET_TIME_A, vale_control[0].time_set);
            sendTelemertry(KEY_SET_TIME_B, vale_control[1].time_set);
            sendTelemertry(KEY_SET_TIME_C, vale_control[2].time_set);
            sendTelemertry(KEY_SET_TIME_D, vale_control[3].time_set);
            sendTelemertry(KEY_SET_MODE_DOSING, data_set[SET_MODE_DOSING]);
            sendTelemertry(KEY_SET_MODE_TUOI, mode_tuoi);

            sendTelemertry(KEY_SET_TIME_TUOI_1, valve_tuoi[0].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_2, valve_tuoi[1].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_3, valve_tuoi[2].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_4, valve_tuoi[3].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_5, valve_tuoi[4].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_6, valve_tuoi[5].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_7, valve_tuoi[6].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_8, valve_tuoi[7].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_9, valve_tuoi[8].timeSet);
            sendTelemertry(KEY_SET_TIME_TUOI_10, valve_tuoi[9].timeSet);

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
            sendTelemertry(KEY_POWER_12, power[11]);
            sendTelemertry(KEY_POWER_13, power[12]);
            sendTelemertry(KEY_POWER_14, power[13]);
            sendTelemertry(KEY_POWER_15, power[14]);
            sendTelemertry(KEY_POWER_16, power[15]);
            sendTelemertry(KEY_POWER_17, power[16]);
            sendTelemertry(POWER_1, vale_control[0].pre_state);
            sendTelemertry(POWER_2, vale_control[1].pre_state);
            sendTelemertry(POWER_3, vale_control[2].pre_state);
            sendTelemertry(POWER_4, vale_control[3].pre_state);
            sendTelemertry(POWER_5, power1[5]);
            sendTelemertry(POWER_6, autoby);
            // sendTelemertry(POWER_1, enabale_cha);
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
void sendTelemertry(String key, int value)
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

void rotary_loop()
{
    // dont print anything unless value changed
    if (rotaryEncoder.encoderChanged())
    {
        Serial.print("Value: ");
        Serial.println(rotaryEncoder.readEncoder());
        Serial.println(rotaryEncoder.getAcceleration());
    }
    if (rotaryEncoder.isEncoderButtonClicked(100))
    {
        rotary_onButtonClick();
    }
}
unsigned long timeUpdateHMI = 0;
void updateHMIState()
{
    //   Serial.println(data_set[SET_MODE_DOSING]);
    switch (DWIN)
    {
    case HOME_PAGE:
    {
        state_system = 0;

        data_set[FLOW_SENSOR_1] = flow_sensor[0].flow_l_m;
        data_set[FLOW_SENSOR_2] = flow_sensor[1].flow_l_m;
        data_set[FLOW_SENSOR_3] = flow_sensor[2].flow_l_m;
        data_set[FLOW_SENSOR_4] = flow_sensor[3].flow_l_m;
        // Serial.println("FLOW a: "+(String)flow_sensor[0].flow_l_m + " FLOW B: "+(String)flow_sensor[1].flow_l_m+" FLOW C: "+(String)flow_sensor[2].flow_l_m+" FLOW D: "+(String)flow_sensor[3].flow_l_m);
    }
        if (millis() - timeUpdateHMI > 500)
        {
            DGUS_SendVal(ADDR_SET_FLOW_1, flow_sensor[0].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_2, flow_sensor[1].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_3, flow_sensor[2].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_4, flow_sensor[3].flow_l_m);
            timeUpdateHMI = millis();
        }

        data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_STOP_MODE_FLOW) : dwinShowPage(PAGE_STOP_MODE_TIME);
        if (rotaryEncoder.isEncoderButtonDown())
        {
            unsigned long preTime = millis();
            DWIN = RUN_PAGE;
            while (rotaryEncoder.isEncoderButtonDown())
            {
                if (millis() - preTime > 1000)
                {
                    DWIN = SETING_PAGE;
                    rotaryEncoder.setBoundaries(0, 4, true);
                    rotaryEncoder.setEncoderValue(0);
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
            //  if(mode_tuoi)
            mode_tuoi == TUOI_PHAN ? dwinShowPage(PAGE_SETING_MODE_TUOI + 1) : dwinShowPage(PAGE_SETING_MODE_TUOI);
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
        data_set[FLOW_SENSOR_1] = flow_sensor[0].flow_l_m;
        data_set[FLOW_SENSOR_2] = flow_sensor[1].flow_l_m;
        data_set[FLOW_SENSOR_3] = flow_sensor[2].flow_l_m;
        data_set[FLOW_SENSOR_4] = flow_sensor[3].flow_l_m;
        if (millis() - timeUpdateHMI > 500)
        {
            DGUS_SendVal(ADDR_SET_FLOW_1, flow_sensor[0].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_2, flow_sensor[1].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_3, flow_sensor[2].flow_l_m);
            DGUS_SendVal(ADDR_SET_FLOW_4, flow_sensor[3].flow_l_m);
            timeUpdateHMI = millis();
        }

        data_set[SET_MODE_DOSING] == MODE_FLOW ? dwinShowPage(PAGE_RUN_MODE_FLOW) : dwinShowPage(PAGE_RUN_MODE_TIME);
        state_system = 1;
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
            mode_tuoi = pos;
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
                vale_control[0].time_set = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_A, vale_control[0].time_set);
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
                vale_control[1].time_set = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_B, vale_control[1].time_set);
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
                vale_control[2].time_set = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_C, vale_control[2].time_set);
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
                vale_control[3].time_set = rotaryEncoder.readEncoder();
                sendTelemertry(KEY_SET_TIME_D, vale_control[3].time_set);
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
                    rotaryEncoder.setEncoderValue(valve_tuoi[0].timeSet);
                    pos = temp;
                }
                else if (temp == 4)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[1].timeSet);
                    pos = temp;
                }
                else if (temp == 5)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[2].timeSet);
                    pos = temp;
                }
                else if (temp == 6)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[3].timeSet);
                    pos = temp;
                }
                else if (temp == 7)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[4].timeSet);
                    pos = temp;
                }
                else if (temp == 8)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[5].timeSet);
                    pos = temp;
                }
                else if (temp == 9)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[6].timeSet);
                    pos = temp;
                }
                else if (temp == 10)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[7].timeSet);
                    pos = temp;
                }
                else if (temp == 11)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[8].timeSet);
                    pos = temp;
                }
                else if (temp == 12)
                {
                    rotaryEncoder.setEncoderValue(valve_tuoi[9].timeSet);
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
                valve_tuoi[0].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[1].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[2].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[3].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[4].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[5].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[6].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[7].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[8].timeSet = rotaryEncoder.readEncoder();
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
                valve_tuoi[9].timeSet = rotaryEncoder.readEncoder();
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
    DynamicJsonDocument data(512);
    data[KEY_SET_TIME_A] = vale_control[0].time_set;
    data[KEY_SET_TIME_B] = vale_control[1].time_set;
    data[KEY_SET_TIME_C] = vale_control[2].time_set;
    data[KEY_SET_TIME_D] = vale_control[3].time_set;
    // data[KEY_SET_FLOW_A] = data_set[SET_FLOW_1];
    // data[KEY_SET_FLOW_B] = data_set[SET_FLOW_2];
    // data[KEY_SET_FLOW_C] = data_set[SET_FLOW_3];
    // data[KEY_SET_FLOW_D] = data_set[SET_FLOW_4];
    data[KEY_SET_MODE_TUOI] = mode_tuoi;
    data[KEY_TIME_TUOI_PHAN] = time_tuoi_phan;

    String objectString;
    serializeJson(data, objectString);
    writeFile(SPIFFS, PATH_STORAGE_STATE, objectString.c_str());
    // Serial.println("SAVE FILE: " + (String)objectString);
}
void localStorageExport()
{
    String data = readFile(SPIFFS, PATH_STORAGE_STATE);
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
        vale_control[0].time_set = doc[KEY_SET_TIME_A];
        vale_control[1].time_set = doc[KEY_SET_TIME_B];
        vale_control[2].time_set = doc[KEY_SET_TIME_C];
        vale_control[3].time_set = doc[KEY_SET_TIME_D];

        // data_set[SET_FLOW_1] = doc[KEY_SET_FLOW_A];
        // data_set[SET_FLOW_2] = doc[KEY_SET_FLOW_B];
        // data_set[SET_FLOW_3] = doc[KEY_SET_FLOW_C];
        // data_set[SET_FLOW_4] = doc[KEY_SET_FLOW_D];

        mode_tuoi = doc[KEY_SET_MODE_TUOI];

        time_tuoi_phan = doc[KEY_TIME_TUOI_PHAN].as<int>();

        Serial.println("Export data success!!");
    }
}

float readPresureSensor()
{
    static float out_value_lowpass = 0;
    float adc = analogRead(ADC_VP);
    float estimated_value = adc_fillter.updateEstimate(adc);
    float presure = 0;
    // Serial.println((String)estimated_value+",");

    // Serial.println("estimated: " + (String)estimated_value);
    presure = mapfloat(estimated_value, 186.68, 1848.06, 0.0, 10.0);
    out_value_lowpass = 0.8 * out_value_lowpass + (1 - 0.8) * presure;
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

///////////////////////////////////////////////
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
        // writeFile(SPIFFS, "/timer.josn", payload.c_str());
        writeFile(SPIFFS, PATH_TIMER, payload.c_str());
    }
    else
    {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
    jsonParseToStruct(payload, _mSchedule);
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
        for (int k = 0; k < 10; k++)
        {
            TIMER[i].keyName[k + 6] = "";
        }
        for (int h = 1; h < 6; h++)
        {

            TIMER[i].keyName[h + 16] = "";
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
        for (int k = 0; k < 16; k++)
        {
            String tempKeyName = "set_power_" + (String)(k + 1);
            if (doc["data"][i]["action"].containsKey(tempKeyName))
            {
                String a = doc["data"][i]["action"][tempKeyName];
                TIMER[i].keyName[k + 6] = tempKeyName;
                a == "true" ? TIMER[i].state[k + 6] = true : TIMER[i].state[k + 6] = false;
            }
        }
        for (int h = 1; h < 6; h++)
        {
            String tempKeyName = "power_" + (String)(h);
            if (doc["data"][i]["action"].containsKey(tempKeyName))
            {
                String a = doc["data"][i]["action"][tempKeyName];

                TIMER[i].keyName[h + 22] = tempKeyName;
                a == "true" ? TIMER[i].state[h + 22] = true : TIMER[i].state[h + 22] = false;
            }
        }
    }
}
// void timerInit()
// {
//     Serial.println("Read data timer");
//     String data_timer = readFile(SPIFFS, PATH_TIMER);
//     jsonObjectTimer(data_timer.c_str());
//     for (int i = 0; i < number_of_timers; i++)
//     {
//         Serial.println(TIMER[i].id);
//     }
// }
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
                    state_system ? DWIN = RUN_PAGE : DWIN = HOME_PAGE;
                }

                if (TIMER[i].keyName[1] == "time_tuoi_phan")
                {
                    time_tuoi_phan = TIMER[i].value[0];
                }

                if (TIMER[i].keyName[2] == "set_time_a")
                {
                    vale_control[0].time_set = TIMER[i].value[1];
                }
                if (TIMER[i].keyName[3] == "set_time_b")
                {
                    vale_control[1].time_set = TIMER[i].value[2];
                }
                if (TIMER[i].keyName[4] == "set_time_c")
                {
                    vale_control[2].time_set = TIMER[i].value[3];
                }
                if (TIMER[i].keyName[5] == "set_time_d")
                {
                    vale_control[3].time_set = TIMER[i].value[4];
                }

                for (int k = 0; k < 16; k++)
                {

                    String keyName = "set_power_" + (String)(k + 1);
                    if (TIMER[i].keyName[k + 6] == keyName)
                    {
                        Serial.print("keyname:" + keyName);
                        Serial.print("power[i]:" + power[k]);
                        power[k] = TIMER[i].state[k + 6];
                        if (k >= 10)
                        {
                            mbIP.Coil(k - 10, TIMER[i].state[k + 6]);
                            // Serial.println("coill "+(String))
                        }
                        //  ML_RtuMaster.writeSingleCoil(SLAVE_ID, k + 6, TIMER[i].state[k + 6]);
                        sendTelemertry(keyName, power[k]);
                    }
                }
                for (int h = 1; h < 6; h++)
                {
                    String keyName1 = "power_" + (String)(h);
                    if (TIMER[i].keyName[h + 22] == keyName1)
                    {
                        Serial.println("keyname:" + keyName1);
                        Serial.print("power[i]:" + power1[h]);
                        // if(keyName1=="power_1")
                        if (h >= 5)
                        {
                            power1[h] = TIMER[i].state[h + 22];
                        }
                        else
                        {
                            vale_control[h - 1].pre_state = TIMER[i].state[h + 22];
                        }
                        //    ML_RtuMaster.writeSingleCoil(SLAVE_ID, h, TIMER[i].state[h + 16]);
                        sendTelemertry(keyName1, power1[h]);
                    }
                }
            }
        }
        else
        {
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
void wifiAP()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("DOSING", "mltech@2019");
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
    // for (int i = 0; i < _numOfSchedule; i++)
    // {
    //     Serial.println("time: " + (String)m_schedule[i].time);
    //     Serial.println("enable: " + (String)m_schedule[i].enable);
    //     Serial.println("interval: " + (String)m_schedule[i].interval);
    //     Serial.println("action count: " + (String)m_schedule[i].action_count);

    //     for (int j = 0; j < m_schedule[i].action_count; j++)
    //     {
    //         Serial.println("key: " + (String)m_schedule[i].action[j].key);
    //         Serial.println("val: " + (String)m_schedule[i].action[j].value);
    //     }
    // }
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
                for (int k = 0; k < 4; k++)
                {
                    String key = "power_" + (String)(k + 1);
                    if (m_schedule.action[j].key == key)
                    {
                        bool state = false;
                        m_schedule.action[j].value == "true" ? state = true : state = false;
                        vale_control[k].pre_state = m_schedule.action[j].value;
                        Serial.println("key: " + (String)m_schedule.action[j].key);
                        Serial.println("val: " + (String)m_schedule.action[j].value);
                    }
                }
                // ccai nay la bom
                if (m_schedule.action[j].key == "power_5")
                {
                    bool state = false;
                    m_schedule.action[j].value == "true" ? state = true : state = false;
                    power1[5] = m_schedule.action[j].value;
                }
                // cai nay la van tuoi
                for (int k = 0; k < 10; k++)
                {
                    String key = "set_power_" + (String)(k + 1);
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
                    state_system ? DWIN = RUN_PAGE : DWIN = HOME_PAGE;
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
                    vale_control[0].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van B

                if (m_schedule.action[j].key == "set_time_b")
                {
                    vale_control[1].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van c

                if (m_schedule.action[j].key == "set_time_c")
                {
                    vale_control[2].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
                // cai nay la thoi gian van d

                if (m_schedule.action[j].key == "set_time_d")
                {
                    vale_control[3].time_set = m_schedule.action[j].value.toInt();
                    Serial.println("key: " + (String)m_schedule.action[j].key);
                    Serial.println("val: " + (String)m_schedule.action[j].value.toInt());
                }
            }
        }
    }
}