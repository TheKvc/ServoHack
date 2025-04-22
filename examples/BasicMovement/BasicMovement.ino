#include <ServoHack.h>

ServoHack servo;
const int SERVO_PIN = 10;
const int POT_PIN = A0;
const int SPEED = 23; // degrees per second

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN, POT_PIN);
  servo.write(90, SPEED); // Move to 90 degrees
}

void loop() {
    int angle = servo.readFeedback();
    Serial.print("Feedback Angle: ");
    Serial.println(angle);
    if (servo.isAtAngle(90)) {
        Serial.println("Servo is at 90 degrees!");
    }
    
    delay(1000); // Wait for 1 second
    Serial.println("Moving to 0 degrees");
    servo.write(0, SPEED); // Move to 0 degrees
    Serial.println("Moving to 180 degrees");
    servo.write(180, SPEED); // Move to 180 degrees

    delay(500);
}