# ServoHack Library

ServoHack is an Arduino library for hacked/modified servo motors that extends the standard Servo library with advanced features for precise servo control. It includes potentiometer feedback, fail-safe operation, and EEPROM-based calibration. This library is designed for applications like robotics, automation, and testing rigs where reliable servo positioning is critical.

## How to Mod / Hack a  hobby servo? 

1. https://www.instructables.com/Using-SG90-or-MG90-Servo-Feedback-Modification-for/
2. https://www.instructables.com/Servo-Feedback-Hack-free/

## Features

- **Potentiometer Feedback**: Reads servo position via a potentiometer for accurate angle verification
- **Fail-Safe Mode**: Detects mechanical obstructions and triggers a callback function to prevent damage
- **EEPROM Calibration**: Stores pulse and potentiometer ranges for consistent operation across power cycles
- **Smooth Movement**: Supports speed-controlled movements with customizable step delays
- **Flexible Configuration**: Set angle ranges, pulse widths, potentiometer ranges, home position, and tolerance

## Installation

### Arduino IDE
1. Open Library Manager (Sketch > Include Library > Manage Libraries)
2. Search for "ServoHack"
3. Click Install

### PlatformIO
1. Add to your platformio.ini:
   ```ini
   lib_deps = 
     karanveer/ServoHack@^1.0.1
   ```
2. Or clone the repository into your lib folder:
   ```
   cd lib
   git clone https://github.com/TheKvc/ServoHack.git
   ```
3. Include in your code:
   ```cpp
   #include <ServoHack.h>
   ```
## Usage

```cpp
#include <ServoHack.h>

ServoHack servo;
const uint8_t SERVO_PIN = 9;
const uint8_t POT_PIN = A0;
const uint16_t EEPROM_ADDR = 0;

void failSafeCallback() {
  Serial.println("Servo stuck!");
  servo.detach();
}

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN, POT_PIN);
    servo.setPulseRange(500, 2500);
    servo.setAngleRange(0, 180);
    servo.setHome(90);
    servo.setTolerance(5);
    servo.setFailSafe(true, failSafeCallback);
    servo.calibrate(EEPROM_ADDR);
    servo.write(90, 20);
}

void loop() {
  // Move to 0 degrees at speed 20
  servo.write(0, 20);
  delay(2000);  // Wait for the servo to reach position
  
  // Move to 180 degrees at speed 20
  servo.write(180, 60);
  delay(2000);  // Wait for the servo to reach position
}
```

## Examples
- DualServoTest: Demonstrates dual-servo testing with feedback, fail-safe, and EEPROM storage.

## Dependencies
- Arduino Servo library
- Arduino EEPROM library

## License
MIT License

## Contributing
Submit issues or pull requests on GitHub.

## Author

TheKvc karanveerchouhan@gmail.com