#ifndef _IO_DEFINE_H_
#define _IO_DEFINE_H_
#define ETH_ADDR 1
#define ETH_POWER_PIN -1 // Do not use it, it can cause conflict during the software reset.
// #define ETH_POWER_PIN_ALTERNATIVE 14
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// #define SDA 5
// #define SCL 4
// #define RS485_TX    13
// #define RS485_RX    12
// #define ADDR_RL 0x24
// #define ADDR_IN 0x25
#endif