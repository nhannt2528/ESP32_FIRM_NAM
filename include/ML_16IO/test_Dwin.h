
#include <Arduino.h>
#include "SoftwareSerial.h"
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

HardwareSerial DWIN_LCD(1);


void DGUS_SendVal(int iAdr, float fVal);
void dwinShowPage(int page);
void updateHMIState();


unsigned long timeoutbt = 0;
int Sw_page = 0;

void appInit()
{
    pinMode(2, INPUT_PULLUP);
    Serial.begin(9600);
    DWIN_LCD.begin(115200, SERIAL_8N1, 4, 5);

}

void appRun()
{
   
    if (digitalRead(2) == 0)
    {
        if (millis() - timeoutbt > 100)
        {
            Sw_page++;
            dwinShowPage(Sw_page);
            Serial.println("PAGE : " + String(Sw_page));
            if (Sw_page > 3)
            {
                Sw_page = 0;
            }
            timeoutbt = millis();
        }
    }
    updateHMIState();


}

///////////////////////////////////////////////////////////////////////////////////////
void updateHMIState()
{
    static unsigned long timePrint = millis();

    if (millis() - timePrint > 1000)
    {
        // Sensor1
        DGUS_SendVal(ADDR_SET_1, random(0,100));
        DGUS_SendVal(ADDR_SET_2,random(0,100));
        DGUS_SendVal(ADDR_SET_3, random(0,100));
        DGUS_SendVal(ADDR_SET_4, random(0,100));
        DGUS_SendVal(ADDR_SET_5, random(0,100));
        DGUS_SendVal(ADDR_SET_6, random(0,100));
        DGUS_SendVal(ADDR_SET_7, random(0,100));
        DGUS_SendVal(ADDR_SET_8, random(0,100));
        DGUS_SendVal(ADDR_SET_9, random(0,100));
        DGUS_SendVal(ADDR_SET_10, random(0,100));
        // Sensor 2
        DGUS_SendVal(ADDR_SET_11, random(0,100));
        DGUS_SendVal(ADDR_SET_12, random(0,100));
        DGUS_SendVal(ADDR_SET_13, random(0,100));
        DGUS_SendVal(ADDR_SET_14, random(0,100));
        DGUS_SendVal(ADDR_SET_15, random(0,100));
        DGUS_SendVal(ADDR_SET_16, random(0,100));
        DGUS_SendVal(ADDR_SET_17, random(0,100));
        DGUS_SendVal(ADDR_SET_18, random(0,100));
        DGUS_SendVal(ADDR_SET_19,random(0,100));
        DGUS_SendVal(ADDR_SET_20, random(0,100));
        // Sensor 3
        DGUS_SendVal(ADDR_SET_21, random(0,100));
        DGUS_SendVal(ADDR_SET_22, random(0,100));
        DGUS_SendVal(ADDR_SET_23,random(0,100));
        DGUS_SendVal(ADDR_SET_24, random(0,100));
        DGUS_SendVal(ADDR_SET_25, random(0,100));
        DGUS_SendVal(ADDR_SET_26, random(0,100));
        DGUS_SendVal(ADDR_SET_27, random(0,100));
        DGUS_SendVal(ADDR_SET_28, random(0,100));
        DGUS_SendVal(ADDR_SET_29, random(0,100));
        DGUS_SendVal(ADDR_SET_30, random(0,100));
        // Sensor 4
        DGUS_SendVal(ADDR_SET_31, random(0,100));
        DGUS_SendVal(ADDR_SET_32, random(0,100));
        DGUS_SendVal(ADDR_SET_33, random(0,100));
        DGUS_SendVal(ADDR_SET_34, random(0,100));
        DGUS_SendVal(ADDR_SET_35, random(0,100));
        DGUS_SendVal(ADDR_SET_36, random(0,100));
        DGUS_SendVal(ADDR_SET_37, random(0,100));
        DGUS_SendVal(ADDR_SET_38, random(0,100));
        DGUS_SendVal(ADDR_SET_39, random(0,100));
        DGUS_SendVal(ADDR_SET_40, random(0,100));
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

////////////////////////////////////////////////////////////////////
