#include "MLDosingGateway.h"
MLDosingGateway::MLDosingGateway(/* args */)
{
    ethernet = new ML_Ethernet();
}

MLDosingGateway::~MLDosingGateway()
{

}
void MLDosingGateway::init(){

   ethernet->init();

}
