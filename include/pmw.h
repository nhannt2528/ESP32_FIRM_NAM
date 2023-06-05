#include <Arduino.h>
// int ENA = 33;
int IN1 = 33;
int IN2 = 32;

const int frequency = 500;
const int pwm_channel1 = 0;
const int pwm_channel2 = 0;
const int resolution = 8;

void appInit();
void appRun();
void changeSpeed();
void setDirection();

void appInit()
{
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  ledcSetup(pwm_channel1, frequency, resolution);
  ledcAttachPin(IN1, pwm_channel1);
  ledcSetup(pwm_channel2, frequency, resolution);
  ledcAttachPin(IN2, pwm_channel2);
}

void appRun()
{
  setDirection();

  // changeSpeed();
}

void setDirection()
{

  ledcWrite(pwm_channel1, 100);
  ledcWrite(pwm_channel2, 100);
  delay(5000);
  ledcWrite(pwm_channel1, 250);
  ledcWrite(pwm_channel2, 250);
  delay(5000);
}

void changeSpeed()
{
  for (int i = 0; i < 256; i++)
  {
    ;
    ledcWrite(pwm_channel1, i);
    Serial.print("i=");
    Serial.println(i);
    delay(500);
  }

  for (int i = 255; i >= 0; --i)
  {
    ledcWrite(pwm_channel1, i);
    Serial.print("i=");
    Serial.println(i);
    delay(500);
  }
}