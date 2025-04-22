#include <ServoHack.h>

ServoHack servo1, servo2;
const uint8_t SERVO1_PIN = 10;
const uint8_t POT1_PIN = A6;
const int servo1EEPROM = 400;
const int servo1Speed = 60; // Speed in degrees per second

const uint8_t SERVO2_PIN = 11;
const uint8_t POT2_PIN = A7;
const int servo2EEPROM = 410;
const int servo2Speed = 10; // Speed in degrees per second

void printInfo(ServoHack &servo) {
    Serial.print("Feedback Angle: ");Serial.print(servo.readFeedback());
    Serial.print(": Library Angle: "); Serial.println(servo.read());
}


void setup() {
  Serial.begin(9600);
  servo1.attach(SERVO1_PIN, POT1_PIN, 0, 180, 500, 2500);
  servo1.setAngleRange(0, 180); // Set angle range for servo1
  servo1.setHome(90); // Set home position for servo1
  servo1.setTolerance(5); // Set tolerance for servo1
  servo1.setAddress(servo1EEPROM); // Set EEPROM address for servo1  
  
  servo2.attach(SERVO2_PIN, POT2_PIN, 0, 180, 500, 2500);
  servo2.setAngleRange(0, 90); // Set angle range for servo2
  servo2.setHome(45); // Set home position for servo2
  servo2.setTolerance(5); // Set tolerance for servo2
  servo2.setAddress(servo2EEPROM); // Set EEPROM address for servo2


  if (!servo1.isCalibrated(servo1EEPROM)) {
    Serial.println("Servo 1 not calibrated. calibrating ...");
    servo1.calibrate(servo1EEPROM); // Calibrate servo1
    Serial.println("Servo 1 calibrated.");
  } else {
    servo1.loadCalibrationSettings(servo1EEPROM);
    Serial.println("Servo 1 loaded calibration settings.");
  }

  if (!servo2.isCalibrated(servo2EEPROM)) {
    Serial.println("Servo 2 not calibrated. calibrating ...");
    servo2.calibrate(servo2EEPROM); // Calibrate servo2
    Serial.println("Servo 2 calibrated.");
  } else {
    servo2.loadCalibrationSettings(servo2EEPROM);
    Serial.println("Servo 2 loaded calibration settings.");
  }
  
}

void loop() {
    // Move both to opposite angles
    servo1.write(0, servo1Speed);
    Serial.print("Servo 1: \t");
    printInfo(servo1);
    servo2.write(90, servo2Speed);
    Serial.print("Servo 2: \t");
    printInfo(servo2);
    delay(1000);

    // Return both to home
    servo1.goHome();
    Serial.print("Servo 1: \t");
    printInfo(servo1);
    servo2.goHome(servo2Speed);
    Serial.print("Servo 2: \t");
    printInfo(servo2);
    delay(1000);

}