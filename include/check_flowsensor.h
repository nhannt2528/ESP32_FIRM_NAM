
#include <FlowMeter.h> // https://github.com/sekdiy/FlowMeter

FlowSensorProperties MySensor = {30.0f, 7.0f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor2 = {30.0f, 7.0f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor3 = {30.0f, 6.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
FlowSensorProperties MySensor4 = {30.0f, 7.0f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
// connect a flow meter to an interrupt pin (see notes on your Arduino model for pin numbers)
FlowMeter *Meter1;
FlowMeter *Meter2;
FlowMeter *Meter3;
FlowMeter *Meter4;

const unsigned long period = 1000;
long lastTime = 0;
void Meter1ISR()
{
  Meter1->count();
}
void Meter2ISR()
{
  Meter2->count();
}

void Meter3ISR()
{
  Meter3->count();
}

void Meter4ISR()
{
  Meter4->count();
}

void appInit()
{
  // prepare serial communication
  Serial.begin(9600);
  // get a new FlowMeter instance for an uncalibrated flow sensor and let them attach their 'interrupt service handler' (ISR) on every rising edge
  Meter1 = new FlowMeter(digitalPinToInterrupt(25), MySensor, Meter1ISR, FALLING); /////////// io25 trên esp
  Meter2 = new FlowMeter(digitalPinToInterrupt(26), MySensor2, Meter2ISR, FALLING);////////// có trở kéo dùng FALLING
  Meter3 = new FlowMeter(digitalPinToInterrupt(27), MySensor3, Meter3ISR, RISING);/////////// không có trở kéo dùng RISING
  Meter4 = new FlowMeter(digitalPinToInterrupt(32), MySensor4, Meter4ISR, RISING);
}
void appRun()
{

  long currentTime = millis();
  long duration = currentTime - lastTime;
  if (duration >= period)
  {
    Meter1->tick(period);
    Meter2->tick(period);
    Meter3->tick(period);
    Meter4->tick(period);
    //////////////////////////////
       Serial.println("Meter 1 currently " + String(Meter1->getCurrentFlowrate()) + " l/min, " + String(Meter1->getTotalVolume()) + " l total.");
       Serial.println("Meter 2 currently " + String(Meter2->getCurrentFlowrate()) + " l/min, " + String(Meter2->getTotalVolume()) + " l total.");
       Serial.println("Meter 3 currently " + String(Meter3->getCurrentFlowrate()) + " l/min, " + String(Meter3->getTotalVolume()) + " l total.");
       Serial.println("Meter 4 currently " + String(Meter4->getCurrentFlowrate()) + " l/min, " + String(Meter4->getTotalVolume()) + " l total.");
    lastTime = currentTime;
  }
}
