#include <ServoHack.h>

ServoHack servo;
const int SERVO_PIN = 10;
const int POT_PIN = A6;
const int SPEED = 23; // degrees per second
const int EEPROM_ADDR = 1000;

void failSafeCallback() {
  Serial.println("Fail-safe triggered: Servo stuck!");
  servo.detach(); // Detach servo to prevent damage
  // Optionally, you can add more actions here, like sending a signal or logging or beeping etc.
}

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN, POT_PIN);
  servo.setFailSafe(true, failSafeCallback);
}

void loop() {
  // Simulate obstruction by physically holding servo

  // Move to 0 degrees
  servo.write(0, SPEED);
  if (servo.isAttached()) {
    Serial.print("Servo Angle: ");
    Serial.println(servo.readFeedback());
  } else {
    Serial.println("Servo detached due to fail-safe.");
  }
  delay(1000);

  // Move to 180 degrees
  servo.write(180, SPEED);
  if (servo.isAttached()) {
    Serial.print("Servo Angle: ");
    Serial.println(servo.readFeedback());
  }
  delay(1000);

}