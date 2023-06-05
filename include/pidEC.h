#include <PID_v1_bc.h>
#include <Arduino.h>

PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);

// Khai báo các hằng số đặc tả bộ điều khiển PID
float Kp = 2.0;
float Ki = 5.0;
float Kd = 1.0;

// Khai báo biến để lưu giá trị đầu vào, giá trị đặt, và giá trị đầu ra của PID
float setpoint = 0.0;
float input = 0.0;
float output = 0.0;

// Khai báo biến để lưu trạng thái relay
int relay_state = LOW;

void appInit();
void appRun();

void appInit() {
  // Khai báo cổng Serial với tốc độ truyền là 9600 baud
  Serial.begin(9600);

  // Khai báo chân kết nối với relay là đầu ra
  pinMode(RELAY_PIN, OUTPUT);

  // Thiết lập các thông số của PID
  pid.SetTunings((double)Kp, (double)Ki, (double)Kd);
  pid.SetSampleTime(1000);
  pid.SetOutputLimits(0, 1);
  pid.SetMode(AUTOMATIC);
}

void appRun() {
  Serial.println("Enter setpoint:");
  while (!Serial.available()) {}
  setpoint = Serial.parseFloat();
  Serial.print("Setpoint is ");
  Serial.println(setpoint);

  pid.Compute();

  if (output >= 0.5 && relay_state == LOW) {
    relay_state = HIGH;
    digitalWrite(RELAY_PIN, relay_state);
    Serial.println("Relay ON");
  } else if (output < 0.5 && relay_state == HIGH) {
    relay_state = LOW;
    digitalWrite(RELAY_PIN, relay_state);
    Serial.println("Relay OFF");
  }

  Serial.print("EC = ");
  Serial.print(input);
  Serial.print(", Setpoint = ");
  Serial.print(setpoint);
  Serial.print(", Output = ");
  Serial.println(output);

  delay(1000);
}