#include <ServoHack.h>

ServoHack servo;
const int SERVO_PIN = 10;
const int POT_PIN = A6;
const int SPEED = 23; // degrees per second
const int EEPROM_ADDR = 1000;

void printAngle() {
    Serial.print("Feedback Angle: ");Serial.print(servo.readFeedback());
    Serial.print(": Library Angle: "); Serial.println(servo.read());
}

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN, POT_PIN);
  if (servo.isCalibrated(EEPROM_ADDR)) {
    Serial.println("Already calibrated!");
    servo.loadCalibrationSettings(EEPROM_ADDR); // Load previous calibration
  } else {
    Serial.println("Calibrating servo...");
    servo.calibrate(EEPROM_ADDR); // Calibrate and save to EEPROM, finds min and max pulse width and pot values
  }
  Serial.print("Pulse Min: ");
  Serial.println(servo.pulse.min);
  Serial.print("Pulse Max: ");
  Serial.println(servo.pulse.max);
  Serial.print("Pot Min: ");
  Serial.println(servo.pot.min);
  Serial.print("Pot Max: ");
  Serial.println(servo.pot.max);
}

void loop() {

    Serial.println("Moving to 0 degrees");
    servo.write(0, SPEED); // Move to 0 degrees
    printAngle();
    delay(1000); // Wait for 1 second

    Serial.println("Moving to 180 degrees");
    servo.write(180, SPEED); // Move to 180 degrees
    printAngle();
    delay(1000); // Wait for 1 second

}