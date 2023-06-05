#include "ML_Ethernet.h"

#define DEBUG 1
#ifdef DEBUG
#define DB_PRINTLN(x) Serial.println(x)
#define DB_PRINT(x) Serial.print(x)
#else
#define DB_PRINTLN(x)
#define DB_PRINT(x)
#endif

static bool eth_connected = false;
void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case SYSTEM_EVENT_ETH_START:
        DB_PRINTLN("ETH Started");
        // set eth hostname here
        ETH.setHostname("MLTECH_ESP32");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        DB_PRINTLN("ETH Connected");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        DB_PRINT("ETH MAC: ");
        DB_PRINT(ETH.macAddress());
        DB_PRINT(", IPv4: ");
        DB_PRINT(ETH.localIP());
        if (ETH.fullDuplex())
        {
            DB_PRINT(", FULL_DUPLEX");
        }
        DB_PRINT(", ");
        DB_PRINT(ETH.linkSpeed());
        DB_PRINTLN("Mbps");
        eth_connected = true;
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        DB_PRINTLN("ETH Disconnected");
        eth_connected = false;
        break;
    case SYSTEM_EVENT_ETH_STOP:
        DB_PRINTLN("ETH Stopped");
        eth_connected = false;
        break;
    default:
        break;
    }
}
void ML_Ethernet::init()
{
#if defined EXTERNAL_CLOCK

    pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
    digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
#endif

    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
}
void ML_Ethernet::init(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress primaryDNS, IPAddress secondaryDNS)
{
    _local_IP = local_ip;
    _gateway = gateway;
    _subnet = subnet;
    _dns1 = primaryDNS;
    _dns2 = secondaryDNS;
#if defined EXTERNAL_CLOCK

    pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
    digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);
#endif
    WiFi.onEvent(WiFiEvent);
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

    ETH.config(_local_IP, _gateway, _subnet, _dns1, _dns2);
}
bool ML_Ethernet::ethernetIsConnected()
{
    return eth_connected;
}

void ML_Ethernet::setDHCP(bool state)
{
    _DHCP = state;
}
IPAddress ML_Ethernet::getLocalIP(){
    return ETH.localIP();
}
