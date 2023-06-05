#ifndef _ML_DOSING_GATEWAY_H_
#define _ML_DOSING_GATEWAY_H_
#include "ML_Ethernet.h"
class MLDosingGateway
{
private:
ML_Ethernet *ethernet;
public:
    MLDosingGateway(/* args */);
    ~MLDosingGateway();
    void init();
};
#endif