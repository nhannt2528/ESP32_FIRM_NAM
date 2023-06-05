#ifndef _ML_ETHERNET_H_
#define _ML_ETHERNET_H_
#include "Arduino.h"
#include "ETH.h"

// #ifdef ETH_CLK_MODE
// #undef ETH_CLK_MODE
// #endif
#define INTERNAL_CLOCK

#if defined INTERNAL_CLOCK

#define ETH_ADDR 1
#define ETH_POWER_PIN -1
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#endif

#if defined EXTERNAL_CLOCK
#define ETH_ADDR 1
#define ETH_POWER_PIN -1
#define ETH_POWER_PIN_ALTERNATIVE 17
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

#endif
class ML_Ethernet
{
private:
    IPAddress _local_IP;
    IPAddress _gateway;
    IPAddress _subnet;
    IPAddress _dns1;
    IPAddress _dns2;
    bool _DHCP = false;
    Print *_consolog;

public:
    void init();
    void init(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress primaryDNS, IPAddress secondaryDNS);
    bool ethernetIsConnected();
    void setDHCP(bool state);
    void setStaticIP(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress primaryDNS, IPAddress secondaryDNS);
    String getIPconfig();
    IPAddress getLocalIP();
};
#endif