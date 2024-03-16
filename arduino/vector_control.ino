#include <Servo.h>

Servo servo;  // Servo for steering
Servo esc;    // ESC for throttle control

int steerPos = 90;     // Neutral position for steering
int throttleVal = 1500;   // Neutral position for throttle

void setup() {
  servo.attach(11);  // Steering servo on pin 11
  esc.attach(10);    // ESC for throttle on pin 10
  Serial.begin(115200);
  servo.write(steerPos);
  esc.writeMicroseconds(throttleVal);
}

void loop() {
  if (Serial.available() >= 2) {
    uint8_t steeringInput = Serial.read();
    uint8_t throttleInput = Serial.read();

    steerPos = map(int(steeringInput), 0, 200, 50, 130); // 50 to 130 is max
    throttleVal = map(int(throttleInput), 0, 200, 1000, 2000); // 1000 to 2000 is max
    
  }
  servo.write(steerPos);
  esc.writeMicroseconds(throttleVal);
}
