

#include "Arduino.h"
#include "SoftwareSerial.h"
#include <ModbusRTU.h>
ModbusRTU mb;

HardwareSerial DWIN_LCD(1);

#define SLAVE_ID 1
#define ADDR_SET_FLOW_1 0x1000
#define ADDR_SET_FLOW_2 0x1001
#define ADDR_SET_FLOW_3 0x1002
#define ADDR_SET_FLOW_4 0x1003

void DGUS_SendVal(int iAdr, int iVal);
void dwinShowPage(int page);
void updateHMIState();
uint16_t data[7];
//////////////////////////////////////////
void appInit()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1);
    mb.begin(&Serial2);
    DWIN_LCD.begin(115200, SERIAL_8N1, 12, 2);
    mb.master();
}
void appRun()
{

    updateHMIState();
    static unsigned long preTime = millis();
    if (millis() - preTime > 100)
    {
        if (!mb.slave())
        {
            mb.readHreg(1, 0, data, 7);
            Serial.println(data[0]);
        }
    mb.task();
    yield();
    preTime = millis();
    }
}
void updateHMIState()
{
    static unsigned long timePrint = millis();

    if (millis() - timePrint > 1000)
    {
        DGUS_SendVal(ADDR_SET_FLOW_1, 1);
        DGUS_SendVal(ADDR_SET_FLOW_2, 2);
        DGUS_SendVal(ADDR_SET_FLOW_3, 3);
        DGUS_SendVal(ADDR_SET_FLOW_4, 4);
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

void DGUS_SendVal(int iAdr, int iVal) // Send iVal for VP= iAdr to DGUS
{
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
}
