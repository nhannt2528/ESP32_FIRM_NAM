#ifndef _ML_DWIN_H_
#define _ML_DWIN_H_
#include "stdint.h"
#include "HardwareSerial.h"
#include "Arduino.h"
class MLDwin
{
private:
    HardwareSerial *_uart;

public:
    MLDwin(/* args */);
    MLDwin(HardwareSerial *_mydwin);
    ~MLDwin();
    void dwinShowPage(int page);
    void DGUS_SendVal(int iAdr, int iVal);
};
#endif