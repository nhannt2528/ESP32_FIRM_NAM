#include "MLDwin.h"
MLDwin::MLDwin(/* args */)
{
}
MLDwin::MLDwin(HardwareSerial *_serial)
{
    _uart = _serial;
}
MLDwin::~MLDwin()
{
}

void MLDwin::dwinShowPage(int page)
{
    static int curentPgae = -1;
    if (curentPgae != page)
    {
        curentPgae = page;
        Serial.println("page: " + (String)curentPgae);
        byte bValL, bValH;
        bValL = page & 0xFF;
        bValH = (page >> 8) & 0xFF;
        _uart->write(0x5A);
        _uart->write(0xA5);
        _uart->write(0x07);
        _uart->write(0x82);
        _uart->write(0x00);
        _uart->write(0x84);
        _uart->write(0x5A);
        _uart->write(0x01);
        _uart->write(bValH);
        _uart->write(bValL);
    }
}
void MLDwin::DGUS_SendVal(int iAdr, int iVal)
{
    byte bAdrL, bAdrH, bValL, bValH;
    bAdrL = iAdr & 0xFF;
    bAdrH = (iAdr >> 8) & 0xFF;
    bValL = iVal & 0xFF;
    bValH = (iVal >> 8) & 0xFF;
    _uart->write(0x5A);
    _uart->write(0xA5);
    _uart->write(0x05);
    _uart->write(0x82);
    _uart->write(bAdrH);
    _uart->write(bAdrL);
    _uart->write(bValH);
    _uart->write(bValL);
}
