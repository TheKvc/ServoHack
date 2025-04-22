#include <ServoHack.h>

ServoHack servo;
const int SERVO_PIN = 10;
const int POT_PIN = A6;

void printInfo() {
    Serial.print("Feedback Angle: ");Serial.print(servo.readFeedback());
    Serial.print(": Library Angle: "); Serial.print(servo.read());
    Serial.print(": Current Pulse: "); Serial.println(servo.readMicroseconds());
}


void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN, POT_PIN, 0, 180, 500, 2500); // Attach servo with angle and pulse range
}

void loop() {
    servo.writeMicroseconds(500, 20); // Move to min pulse smoothly with speed of 60 degrees per second
    printInfo();
    delay(1000);

    servo.writeMicroseconds(2500, 60); // Move to max pulse smoothly with speed of 120 degrees per second
    delay(1000);
    printInfo();
}