#ifndef __SERVOHACK_H__
#define __SERVOHACK_H__

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

#define MAGIC_BYTE 0xAA
#define DEFAULT_TOLERANCE 5
#define DEFAULT_SPEED 23
#define DEFAULT_PULSE_MIN 500
#define DEFAULT_PULSE_MAX 2500
#define DEFAULT_ANGLE_MIN 0
#define DEFAULT_ANGLE_MAX 180
#define DEFAULT_POT_MIN 70
#define DEFAULT_POT_MAX 650
#define DEFAULT_EEPROM_ADDRESS 65535
#define DEFAULT_HOME 90
#define DEFAULT_PIN 255
#define DEFAULT_POT_PIN 255

#define POT_READINGS 30
#define POT_DISCARD_READINGS 5

struct Range {
    int16_t min;
    int16_t max;
};

class ServoHack {
public:
    Range angle;           // Min and max angle in degrees (uint16_t)
    Range pulse;           // Min and max pulse width in microseconds (uint16_t)
    Range pot;             // Min and max potentiometer values (uint16_t)

    // Constructors
    ServoHack();                             // Default constructor

    // Methods to initialize and detach servo
    void attach(uint8_t pin);              // Attach servo to pin
    void attach(uint8_t pin, uint8_t potPin); // Attach servo to pin and potentiometer
    void attach(uint8_t pin, uint8_t potPin, uint16_t angleMin, uint16_t angleMax); // Attach with angle range
    void attach(uint8_t pin, uint8_t potPin, uint16_t angleMin, uint16_t angleMax, uint16_t pulseMin, uint16_t pulseMax); // Attach with angle and pulse range
    void detach();                         // Detach servo

    // Calibration and EEPROM methods
    void calibrate();                      // Calibrate servo (no saving)
    void calibrate(uint16_t EEPROMAddress); // Calibrate and save to EEPROM at specified address
    void loadCalibrationSettings(uint16_t EEPROMAddress); // Load calibration from EEPROM
    void loadDefaultSettings(); // Load default settings
    void saveCalibrationSettings();        // Save current settings to variables
    void saveCalibrationSettings(uint16_t EEPROMAddress); // Save current settings to EEPROM at specified address

    // Methods to set servo properties
    void setHome(uint16_t homeAngle);     // Set home position
    void setTolerance(uint8_t tolerance); // Set position tolerance
    void setPulseRange(uint16_t pulseMin, uint16_t pulseMax); // Set pulse width
    void setAngleRange(uint16_t minAngle, uint16_t maxAngle); // Set angle range
    void setPotRange(uint16_t minPot, uint16_t maxPot); // Set potentiometer range
    void setAddress(uint16_t EEPROMAddress); // Set EEPROM address for calibration
    void setFailSafe(bool failSafe, void (*callback)()); // Set fail-safe mode with callback

    // Methods to get servo properties
    uint16_t read();                      // Get current angle
    uint16_t readPrev();                  // Get previous angle
    uint16_t readFeedback();              // Get angle from potentiometer feedback
    uint16_t readMicroseconds();          // Get pulse width in microseconds
    uint16_t getHome();                   // Get home position
    uint8_t getTolerance();               // Get position tolerance
    uint16_t getPot();                    // Get potentiometer value
    bool getFailSafe();                  // Check if fail-safe mode is enabled

    // Methods to check servo properties
    bool isAttached();                    // Check if servo is attached
    bool isAtHome();                      // Check if servo is at home position
    bool isAtAngle(uint16_t angle);       // Check if servo is at specified angle
    bool isCalibrated();                  // Check if servo is calibrated
    bool isCalibrated(uint16_t EEPROMAddress); // Check if servo is calibrated at specified address

    // Methods to move servo
    void write(uint16_t angle);           // Instant move to angle (like Servo::write)
    void write(uint16_t angle, uint8_t speed); // Blocking move to angle with speed
    void write(uint16_t angle, uint8_t speed, void (*callback)()); // Blocking move with speed and callback
    void writeMicroseconds(uint16_t pulse); // Instant move to pulse width (like Servo::writeMicroseconds)
    void writeMicroseconds(uint16_t pulse, uint8_t speed); // Blocking move to pulse with speed
    void writeMicroseconds(uint16_t pulse, uint8_t speed, void (*callback)()); // Blocking move with speed and callback
    void goHome();                        // Blocking move to home position with default speed
    void goHome(uint8_t speed);           // Blocking move to home position with specified speed

private:
    Servo servo;              // Underlying Servo object
    uint8_t pin;              // PWM pin for servo
    uint8_t potPin;           // Analog pin for potentiometer feedback
    uint16_t currentAngle;    // Current angle in degrees
    uint16_t prevAngle;       // Previous angle in degrees
    uint16_t home;            // Home position angle in degrees
    uint8_t tolerance;        // Position tolerance in degrees
    uint16_t EEPROMAddress;   // EEPROM base address for calibration values
    bool failSafe;         // Flag for fail-safe mode
    void (*failSafeCallback)();  // failsafe callback function ptr

    // Private struct for EEPROM storage (18 bytes)
    struct ServoSettings {
        Range pulse;      // 4 bytes: Min and max pulse width in microseconds
        Range pot;        // 4 bytes: Min and max potentiometer values
    } settings;           // Private member for EEPROM settings

    void checkfailSafe(uint16_t angle); // Check if servo is at target angle
    
};
#endif