#include "Arduino.h"
#include "ETH.h"
#include "../include/ML_16IO/IO_define.h"
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

////////////////////////////////////////////////////
#define THINGS_BOARD_SERVER "mqtt.viis.tech"
// #define TOKEN "VmNkJ3v5DNwho5NCU1zK"
// #define ID "ec40d1c0-61d6-11ed-99cb-4de8ebde04d6"
#define TOKEN "mQECxIw9xvGh1PxpKTFq"
#define ID "ae9b6ce0-5f74-11ed-99cb-4de8ebde04d6"
#define TIME_CHECK_WIFI 2000
#define TIME_SEND_SS 60000
////////////////////////////////////////////////////
ModbusIP mbIP;

ModbusRTU mb; // ModbusIP object
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  return true;
}

/////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient client(espClient);
String serverName = "https://iot.viis.tech/api/viis/schedule/device?accessToken=";
AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 60 * 60);
const char *ssid = "  ";
const char *pass = "mltech@2019";
// const char *ssid = "MLTECH_SHOP";
// const char *pass = "mltech@2019";


bool coils[11];
bool button = false;
#define DC 0x01
byte reg_rtu[8] = {DC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t ModRTU_CRC(byte buf[], int len);
void set_rst_coil(Stream *port, int addr, bool state);
void printByteArray(String name, byte arr[], int len); // in thanh ghi

void updateState();
void updatesensor1();
void updatesensor2();
void setsensor();
void wifiConnect();
bool eth_connected = false;
void WiFiEvent(WiFiEvent_t event);
void mqtt_sendTelemertry();
void sendTelemertry(String key, bool value);
void sendTelemertry(String key, float value);
void sendTelemertry(String key, int value);
void saveLocalStorage();
void localStorageExport();
void mqttInit();
void on_message(const char *topic, byte *payload, unsigned int length);
void mqtt_loop();
void mqtt_reconnect();
void ModbusIpInit();
void ModbusIpLoop();
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
IPAddress local_IP(192, 168, 0, 108); // tu sam 2
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

float nhietdo;
float doam;
float lux;
/////////////////////////////////////////////////////////////
uint32_t floatMidLitteEndianCDAB(uint16_t AB, uint16_t CD);
float uint32ToFloat(uint32_t x);

typedef enum
{
  IDLE_STATE,
  WRITE_PUMP_1,
  WRITE_PUMP_2,
  WRITE_PUMP_3,
  WRITE_PUMP_4,
  WRITE_PUMP_5,
  WRITE_PUMP_6,
  WRITE_PUMP_7,
  WRITE_PUMP_8,
  WRITE_PUMP_9,
  WRITE_PUMP_10,
  WRITE_PUMP_11,

} state;
state STATE;
//////////////////////////////////////////////
struct set
{
  float temp_Low = 0;
  float temp_High = 0;
  float hum_Low = 0;
  float hum_High = 0;
  int light_Low = 0;
  int light_High = 0;
  int light_High1 = 0;
  int fre = 0;
  String device;
  bool state = false;

} set;
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
  // pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
  // digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
  // ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH
  // ETH.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.onEvent(WiFiEvent);
  Serial.begin(115200);
  fileInit();
  timerInit();
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial.println("STA Failed to configure");
  }
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
  mb.client();
  Serial2.begin(9600, SERIAL_8N1);
  mb.begin(&Serial2);
  mb.master();
  localStorageExport();
  ModbusIpInit();

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
  nhietdo = (float)mbIP.Hreg(0) / 100;
  doam = (float)mbIP.Hreg(1) / 100;
  uint32_t lux_32t = floatMidLitteEndianCDAB((uint16_t)mbIP.Hreg(22), (uint16_t)mbIP.Hreg(23));
  lux = uint32ToFloat(lux_32t);
  // Serial.println("LUX: " + (String)lux);
  // Serial.println(sensor.temp);
  // Serial.println(sensor.humi);
  // Serial.println(mbIP.Hreg(22), HEX);
  // Serial.println(mbIP.Hreg(23), HEX);
  // delay(1000);
  Serial.println("TEMP: " + (String)nhietdo);
  Serial.println("HUMI: " + (String)doam);
  Serial.println("LUX: " + (String)lux);
  delay(1000);
  mqtt_sendTelemertry();
  if ((button == true))
  {
    setsensor();
    sendTelemertry("power_12", button);
  }
  // if (WL_CONNECTED)
  // {
  //   mqtt_loop();
  // }
  if (eth_connected || WL_CONNECTED)
    {
        mqtt_loop();

    }
  updateState();
  ModbusIpLoop();
  NTPTimeUpdate();
}
void updateState()
{
  static unsigned long rateTime;
  if (millis() - rateTime > 100)
  {
    if (!mb.slave())
    {
      switch (STATE)
      {
      case IDLE_STATE:
      {
        STATE = WRITE_PUMP_1;
        // mb.writeCoil(1, 0, coils, 11);
        break;
      }
      case WRITE_PUMP_1:
      {
        set_rst_coil(&Serial2, 0, coils[0]);
        STATE = WRITE_PUMP_2;
        break;
      }
      case WRITE_PUMP_2:
      {
        set_rst_coil(&Serial2, 1, coils[1]);
        STATE = WRITE_PUMP_3;
        break;
      }
      case WRITE_PUMP_3:
      {
        set_rst_coil(&Serial2, 2, coils[2]);
        STATE = WRITE_PUMP_4;
        break;
      }
      case WRITE_PUMP_4:
      {
        set_rst_coil(&Serial2, 3, coils[3]);
        STATE = WRITE_PUMP_5;
        break;
      }
      case WRITE_PUMP_5:
      {
        set_rst_coil(&Serial2, 4, coils[4]);
        STATE = WRITE_PUMP_6;
        break;
      }
      case WRITE_PUMP_6:
      {
        set_rst_coil(&Serial2, 5, coils[5]);
        STATE = WRITE_PUMP_7;
        break;
      }
      case WRITE_PUMP_7:
      {
        set_rst_coil(&Serial2, 6, coils[6]);
        STATE = WRITE_PUMP_8;
        break;
      }
      case WRITE_PUMP_8:
      {
        set_rst_coil(&Serial2, 7, coils[7]);
        STATE = WRITE_PUMP_9;
        break;
      }
      case WRITE_PUMP_9:
      {
        set_rst_coil(&Serial2, 10, coils[8]);
        STATE = WRITE_PUMP_10;
        break;
      }
      case WRITE_PUMP_10:
      {
        set_rst_coil(&Serial2, 11, coils[9]);
        STATE = WRITE_PUMP_11;
        break;
      }
      case WRITE_PUMP_11:
      {
        set_rst_coil(&Serial2, 12, coils[10]);
        STATE = WRITE_PUMP_1;
        break;
      }
      }
    }
    mb.task();
    yield();
    rateTime = millis();
  }
}
void set_rst_coil(Stream *port, int addr, bool state)
{

  reg_rtu[3] = addr;
  state == true ? reg_rtu[4] = 0xff : reg_rtu[4] = 0x00;
  uint16_t CRC = ModRTU_CRC(reg_rtu, 6);
  reg_rtu[6] = lowByte(CRC);
  reg_rtu[7] = highByte(CRC);

  port->write(reg_rtu, 8);
  printByteArray("TX: ", reg_rtu, 8);
}
// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte buf[], int len)
{
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {            // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else         // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}
void printByteArray(String name, byte arr[], int len)
{
  Serial.print(name + " : [");
  for (int i = 0; i < len; i++)
  {
    Serial.print(" ");
    if (arr[i] <= 10)
    {
      Serial.print(0);
      Serial.print(arr[i], HEX);
    }
    else
    {
      Serial.print(arr[i], HEX);
    }
  }
  Serial.println(" ]");
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
    writeFile(LITTLEFS, "/data/localStorage.json", json);
    if (doc["params"].containsKey("power_1"))
    {
      coils[0] = doc["params"]["power_1"].as<bool>();
      sendTelemertry("power_1", coils[0]);
    }
    if (doc["params"].containsKey("power_2"))
    {
      coils[1] = doc["params"]["power_2"].as<bool>();
      sendTelemertry("power_2", coils[1]);
    }
    if (doc["params"].containsKey("power_3"))
    {
      coils[2] = doc["params"]["power_3"].as<bool>();
      sendTelemertry("power_3", coils[2]);
    }
    if (doc["params"].containsKey("power_4"))
    {
      coils[3] = doc["params"]["power_4"].as<bool>();
      sendTelemertry("power_4", coils[3]);
    }
    if (doc["params"].containsKey("power_5"))
    {
      coils[4] = doc["params"]["power_5"].as<bool>();
      sendTelemertry("power_5", coils[4]);
    }
    if (doc["params"].containsKey("power_6"))
    {
      coils[5] = doc["params"]["power_6"].as<bool>();
      sendTelemertry("power_6", coils[5]);
    }
    if (doc["params"].containsKey("power_7"))
    {
      coils[6] = doc["params"]["power_7"].as<bool>();
      sendTelemertry("power_7", coils[6]);
    }
    if (doc["params"].containsKey("power_8"))
    {
      coils[7] = doc["params"]["power_8"].as<bool>();
      sendTelemertry("power_8", coils[7]);
    }
    if (doc["params"].containsKey("power_9"))
    {
      coils[8] = doc["params"]["power_9"].as<bool>();
      sendTelemertry("power_9", coils[8]);
    }
    if (doc["params"].containsKey("power_10"))
    {
      coils[9] = doc["params"]["power_10"].as<bool>();
      sendTelemertry("power_10", coils[9]);
    }
    if (doc["params"].containsKey("power_11"))
    {
      coils[10] = doc["params"]["power_11"].as<bool>();
      sendTelemertry("power_11", coils[10]);
    }
    if (doc["params"].containsKey("power_12"))
    {
      button = doc["params"]["power_12"].as<bool>();
      sendTelemertry("power_12", button);
    }
    if (doc["params"].containsKey("set_TEMP_LOW"))
    {
      set.temp_Low = doc["params"]["set_TEMP_LOW"].as<int>();
      sendTelemertry("set_TEMP_LOW", set.temp_Low);
    }
    if (doc["params"].containsKey("set_TEMP_HIGH"))
    {
      set.temp_High = doc["params"]["set_TEMP_HIGH"];
      sendTelemertry("set_TEMP_HIGH", set.temp_High);
    }
    if (doc["params"].containsKey("set_HUM_LOW"))
    {
      set.hum_Low = doc["params"]["set_HUM_LOW"];
      sendTelemertry("set_HUM_LOW", set.hum_Low);
    }
    if (doc["params"].containsKey("set_HUM_HIGH"))
    {
      set.hum_High = doc["params"]["set_HUM_HIGH"];
      sendTelemertry("set_HUM_HIGH", set.hum_High);
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
    if (doc["params"].containsKey("set_LUX_HIGH1"))
    {
      set.light_High1 = doc["params"]["set_LUX_HIGH1"];
      sendTelemertry("set_LUX_HIGH", set.light_High1);
    }
    if (doc["params"].containsKey("set_FRE"))
    {
      set.fre = doc["params"]["set_FRE"];
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
      sendTelemertry("power_1", coils[0]);
      sendTelemertry("power_2", coils[1]);
      sendTelemertry("power_3", coils[2]);
      sendTelemertry("power_4", coils[3]);
      sendTelemertry("power_5", coils[4]);
      sendTelemertry("power_6", coils[5]);
      sendTelemertry("power_7", coils[6]);
      sendTelemertry("power_8", coils[7]);
      sendTelemertry("power_9", coils[8]);
      sendTelemertry("power_10", coils[9]);
      sendTelemertry("power_11", coils[10]);
      sendTelemertry("power_12", button);
      mqtt_sendTelemertry();
    }
    else
    {
      Serial.println("Connect fail");
      Serial.println(client.state());

      //  delay(2000);
    }
  }
}
void mqtt_sendTelemertry()
{
  DynamicJsonDocument data(1024);
  // data["state"]=digitalRead(Q1);
  data["nhietdo"] = nhietdo;
  data["doam"] = doam;
  data["light"] = lux;
  data["set_TEMP_LOW"] = set.temp_Low;
  data["set_TEMP_HIGH"] = set.temp_High;
  data["set_HUM_LOW"] = set.hum_Low;
  data["set_HUM_HIGH"] = set.hum_High;
  data["set_LUX_LOW"] = set.light_Low;
  data["set_LUX_HIGH"] = set.light_High;
  data["set_LUX_HIGH1"] = set.light_High1;
  data["set_FRE"] = set.fre;
  String objectString;
  serializeJson(data, objectString);
  //  Serial.println(objectString);
  client.publish("v1/devices/me/telemetry", objectString.c_str());
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
  DynamicJsonDocument data(1024);
  data[key] = value;
  String objectString;
  serializeJson(data, objectString);
  client.publish("v1/devices/me/telemetry", objectString.c_str());
  Serial.println(objectString);
}
void setsensor()
{

  if ((set.light_Low > lux))
  {
    set_rst_coil(&Serial2, 0, coils[0] = true);
    set_rst_coil(&Serial2, 1, coils[1] = false);
    set_rst_coil(&Serial2, 2, coils[2] = true);
    set_rst_coil(&Serial2, 3, coils[3] = false);
    sendTelemertry("power_1", coils[0]);
    sendTelemertry("power_2", coils[1]);
    sendTelemertry("power_3", coils[2]);
    sendTelemertry("power_4", coils[3]);
  }
  if ((set.light_High < lux))
  {
    set_rst_coil(&Serial2, 2, coils[2] = false);
    set_rst_coil(&Serial2, 3, coils[3] = true);
    sendTelemertry("power_3", coils[2]);
    sendTelemertry("power_4", coils[3]);
  }
  if ((set.light_High1 < lux))
  {
    set_rst_coil(&Serial2, 0, coils[0] = false);
    set_rst_coil(&Serial2, 1, coils[1] = true);
    sendTelemertry("power_1", coils[0]);
    sendTelemertry("power_2", coils[1]);
  }
  if ((set.temp_Low > nhietdo))
  {
    set_rst_coil(&Serial2, 7, coils[7] = false);
    set_rst_coil(&Serial2, 10, coils[8] = false);
    set_rst_coil(&Serial2, 11, coils[9] = false);
    set_rst_coil(&Serial2, 12, coils[10] = false);
    sendTelemertry("power_8", coils[7]);
    sendTelemertry("power_9", coils[8]);
    sendTelemertry("power_10", coils[9]);
    sendTelemertry("power_11", coils[10]);
  }
  if ((nhietdo > set.temp_High))
  {
    set_rst_coil(&Serial2, 7, coils[7] = true);
     set_rst_coil(&Serial2, 10, coils[8] = true);
    set_rst_coil(&Serial2, 11, coils[9] = true);
    set_rst_coil(&Serial2, 12, coils[10] = true);
    sendTelemertry("power_8", coils[7]);
    sendTelemertry("power_9", coils[8]);
    sendTelemertry("power_10", coils[9]);
    sendTelemertry("power_11", coils[10]);
  }
  // if ((set.hum_Low > doam))
  // {
  //   set_rst_coil(&Serial2, 10, coils[8] = true);
  //   sendTelemertry("power_9", coils[8]);
  // }
  // if ((doam > set.hum_High))
  // {
  //   set_rst_coil(&Serial2, 10, coils[8] = false);
  //   sendTelemertry("power_9", coils[8]);
  // }
}
////////////////////////////////////////////////////////////////////
void saveLocalStorage()
{
  DynamicJsonDocument data(1024);
  coils[0] == true ? data["power_1"] = true : data["power_1"] = false;
  coils[1] == true ? data["power_2"] = true : data["power_2"] = false;
  coils[2] == true ? data["power_3"] = true : data["power_3"] = false;
  coils[3] == true ? data["power_4"] = true : data["power_4"] = false;
  coils[4] == true ? data["power_5"] = true : data["power_5"] = false;
  coils[5] == true ? data["power_6"] = true : data["power_6"] = false;
  coils[6] == true ? data["power_7"] = true : data["power_7"] = false;
  coils[7] == true ? data["power_8"] = true : data["power_8"] = false;
  coils[8] == true ? data["power_9"] = true : data["power_9"] = false;
  coils[9] == true ? data["power_10"] = true : data["power_10"] = false;
  coils[10] == true ? data["power_11"] = true : data["power_11"] = false;
  button == true ? data["power_12"] = true : data["power_12"] = false;
  data["set_TEMP_LOW"] = set.temp_Low;
  data["set_TEMP_HIGH"] = set.temp_High;
  data["set_HUM_LOW"] = set.hum_Low;
  data["set_HUM_HIGH"] = set.hum_High;
  data["set_LUX_LOW"] = set.light_Low;
  data["set_LUX_HIGH"] = set.light_High;
  data["set_LUX_HIGH1"] = set.light_High1;
  data["set_FRE"] = set.fre;
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
    coils[0] = doc["power_1"];
    coils[1] = doc["power_2"];
    coils[2] = doc["power_3"];
    coils[3] = doc["power_4"];
    coils[4] = doc["power_5"];
    coils[5] = doc["power_6"];
    coils[6] = doc["power_7"];
    coils[7] = doc["power_8"];
    coils[8] = doc["power_9"];
    coils[9] = doc["power_10"];
    coils[10] = doc["power_11"];
    button = doc["power_12"];

    set.temp_Low = doc["set_TEMP_LOW"];
    set.temp_High = doc["set_TEMP_HIGH"];
    set.hum_Low = doc["set_HUM_LOW"];
    set.hum_High = doc["set_HUM_HIGH"];
    set.light_Low = doc["set_LUX_LOW"];
    set.light_High = doc["set_LUX_HIGH"];
    set.light_High1 = doc["set_LUX_HIGH1"];
    set.fre = doc["set_FRE"];

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
    for (int k = 0; k < 11; k++)
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

        for (int k = 0; k < 11; k++)
        {
          String keyName = "power_" + (String)(k + 1);
          if (TIMER[i].keyName[k] == keyName)
          {
            // digitalWrite(coils_[k], TIMER[i].state[k]);
            coils[k] = TIMER[i].state[k];
            Serial.println("STATE RL: " + (String)k + "state: " + (String)TIMER[i].state[0]);
            sendTelemertry("power_1", coils[0]);
            sendTelemertry("power_2", coils[1]);
            sendTelemertry("power_3", coils[2]);
            sendTelemertry("power_4", coils[3]);
            sendTelemertry("power_5", coils[4]);
            sendTelemertry("power_6", coils[5]);
            sendTelemertry("power_7", coils[6]);
            sendTelemertry("power_8", coils[7]);
            sendTelemertry("power_9", coils[8]);
            sendTelemertry("power_10", coils[9]);
            sendTelemertry("power_11", coils[10]);
            sendTelemertry("power_12", button);
          }
        }
      }
    }
    else
    {
    }
  }
}
