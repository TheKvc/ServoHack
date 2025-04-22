#include <ServoHack.h>

ServoHack servo;
const uint8_t SERVO_PIN = 10;        // PWM pin for servo
const uint8_t POT_PIN = A6;         // Potentiometer pin for feedback
const uint16_t EEPROM_ADDR = 0;     // Safe EEPROM address for calibration

void failSafeCallback() {
  Serial.println("Fail-safe triggered: Servo not at target!");
}

void setup() {
  Serial.begin(9600);
  
  // Attach servo with potentiometer
  servo.attach(SERVO_PIN, POT_PIN);
  
  // Set all properties
  servo.setAngleRange(10, 170);                  // Angle range: 10–170 degrees
  servo.setPulseRange(600, 2400);                // Pulse range: 600–2400 µs
  servo.setPotRange(100, 600);                   // Pot range: 100–600
  servo.setHome(100);                            // Home position: 100 degrees
  servo.setTolerance(3);                         // Tolerance: ±3 degrees
  servo.setFailSafe(true, failSafeCallback);     // Enable fail-safe with callback
  servo.setAddress(EEPROM_ADDR);                 // EEPROM address for saving
  
  // Save settings to EEPROM
  servo.saveCalibrationSettings(EEPROM_ADDR);
  
  // Verify settings
  Serial.println("Servo Properties Set:");
  Serial.print("Angle Range: ");
  Serial.print(servo.angle.min);
  Serial.print("–");
  Serial.println(servo.angle.max);
  Serial.print("Pulse Range: ");
  Serial.print(servo.pulse.min);
  Serial.print("–");
  Serial.println(servo.pulse.max);
  Serial.print("Pot Range: ");
  Serial.print(servo.pot.min);
  Serial.print("–");
  Serial.println(servo.pot.max);
  Serial.print("Home: ");
  Serial.println(servo.getHome());
  Serial.print("Tolerance: ");
  Serial.println(servo.getTolerance());
  Serial.print("Fail-safe: ");
  Serial.println(servo.getFailSafe() ? "Enabled" : "Disabled");
  
  // Test movement with settings
  Serial.println("Calibrating servo...");
  servo.calibrate();  // Calibrate to confirm settings
  Serial.println("Moving to 10 degrees... with speed 10 deg/s");
  servo.write(10, 10);  // Speed 10 degrees/s
  delay(2000);
  Serial.println("Moving to home (100 degrees)... with speed 60 deg/s");
  servo.goHome(60);  // Speed 60 degrees/s
  delay(2000);
}

void loop() {}