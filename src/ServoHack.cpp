#include <ServoHack.h>

ServoHack::ServoHack() 
    : angle{DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_MAX}, 
      pulse{DEFAULT_PULSE_MIN, DEFAULT_PULSE_MAX}, 
      pot{DEFAULT_POT_MIN, DEFAULT_POT_MAX}, 
      pin(DEFAULT_PIN), potPin(DEFAULT_POT_PIN), currentAngle(DEFAULT_HOME), prevAngle(DEFAULT_HOME), 
      home(DEFAULT_HOME), tolerance(DEFAULT_TOLERANCE), EEPROMAddress(DEFAULT_EEPROM_ADDRESS), failSafe(false), failSafeCallback(nullptr),
      settings{{DEFAULT_PULSE_MIN, DEFAULT_PULSE_MAX}, {DEFAULT_POT_MIN, DEFAULT_POT_MAX}} {
}

void ServoHack::attach(uint8_t pin) {
    if (pin == DEFAULT_PIN) {return;}

    this->pin = pin;
    servo.write(home);
    servo.attach(pin, pulse.min, pulse.max);

    currentAngle = home;
    prevAngle = currentAngle;

    saveCalibrationSettings();
}

void ServoHack::attach(uint8_t pin, uint8_t potPin) {
    if (pin == DEFAULT_PIN || potPin == DEFAULT_POT_PIN) {return;}

    this->pin = pin;
    this->potPin = potPin;
    currentAngle = readFeedback();
    servo.write(currentAngle);  // writing the current predicted angle (from potPin) before attaching the servo to avoid any sudden movement
    servo.attach(pin, pulse.min, pulse.max);

    prevAngle = currentAngle;
    saveCalibrationSettings();
}

void ServoHack::attach(uint8_t pin, uint8_t potPin, uint16_t angleMin, uint16_t angleMax) {
    angle.min = angleMin;
    angle.max = angleMax;
    saveCalibrationSettings();
    attach(pin, potPin);
}

void ServoHack::attach(uint8_t pin, uint8_t potPin, uint16_t angleMin, uint16_t angleMax, uint16_t pulseMin, uint16_t pulseMax) {
    angle.min = angleMin;
    angle.max = angleMax;
    pulse.min = pulseMin;
    pulse.max = pulseMax;
    saveCalibrationSettings();
    attach(pin, potPin);
}

void ServoHack::detach() {
    servo.detach();
}

void ServoHack::calibrate() {
    if (pin == DEFAULT_PIN || potPin == DEFAULT_POT_PIN) {return;}
    if (!isAttached()) {return;}

    bool failSafeBakup = failSafe; // backup the failSafe state
    failSafe = false; // disable failSafe during calibration

    // calibraing values at the minimum angle ...
    write(angle.min, DEFAULT_SPEED); // moving to min angle to calibrate potentiometer
    write(angle.min); // writing the min angle to be sure
    delay(1000);
    pulse.min = readMicroseconds();
    pot.min = getPot();
    delay(1000);

    // calibraing values at the maximum angle ...
    write(angle.max, DEFAULT_SPEED); // moving to max angle to calibrate potentiometer
    write(angle.max); // writing the max angle to be sure
    delay(1000);
    pulse.max = readMicroseconds();
    pot.max = getPot();
    delay(1000);

    saveCalibrationSettings();
    goHome();
    failSafe = failSafeBakup; // restore the failSafe state
}

void ServoHack::calibrate(uint16_t EEPROMAddress) {
    this->EEPROMAddress = EEPROMAddress;
    calibrate();
    saveCalibrationSettings(EEPROMAddress);
    loadCalibrationSettings(EEPROMAddress);
}

void ServoHack::loadDefaultSettings() {
    angle = {DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_MAX};
    pulse = {DEFAULT_PULSE_MIN, DEFAULT_PULSE_MAX};
    pot = {DEFAULT_POT_MIN, DEFAULT_POT_MAX};
    pin = DEFAULT_PIN;
    potPin = DEFAULT_POT_PIN;
    home = DEFAULT_HOME;
    tolerance = DEFAULT_TOLERANCE;
    failSafe = false;
    failSafeCallback = nullptr;
    EEPROMAddress = DEFAULT_EEPROM_ADDRESS;
    saveCalibrationSettings();
}

void ServoHack::loadCalibrationSettings(uint16_t EEPROMAddress) {
    this->EEPROMAddress = EEPROMAddress;
    if (!isCalibrated()) return;

    EEPROM.get(EEPROMAddress + 1, settings);

    if (settings.pulse.min >= settings.pulse.max || settings.pot.min >= settings.pot.max) {
        // Use default settings when calibration data is invalid
        loadDefaultSettings();
    }

    pulse = settings.pulse;
    pot = settings.pot;

    if (isAttached()) {
        detach();
        attach(pin, potPin);
    }
}

void ServoHack::saveCalibrationSettings() {
    settings = ServoSettings{pulse, pot};
}

void ServoHack::saveCalibrationSettings(uint16_t EEPROMAddress) {
    this->EEPROMAddress = EEPROMAddress;
    if ((EEPROMAddress != DEFAULT_EEPROM_ADDRESS) && (EEPROMAddress < (EEPROM.length() - sizeof(settings)))) {
        EEPROM.update(EEPROMAddress, MAGIC_BYTE);
        EEPROM.put(EEPROMAddress + 1, settings);
    }
    saveCalibrationSettings();
}

void ServoHack::setAddress(uint16_t EEPROMAddress) {
    this->EEPROMAddress = EEPROMAddress;
}

void ServoHack::setHome(uint16_t homeAngle) {
    home = homeAngle;
}

void ServoHack::setTolerance(uint8_t tolerance) {
    this->tolerance = tolerance;
}

void ServoHack::setPulseRange(uint16_t pulseMin, uint16_t pulseMax) {
    pulse.min = pulseMin;
    pulse.max = pulseMax;
    settings.pulse = pulse;
    if (isAttached()) {
        detach();
        attach(pin, potPin);
    }
}

void ServoHack::setAngleRange(uint16_t minAngle, uint16_t maxAngle) {
    angle.min = minAngle;
    angle.max = maxAngle;
}

void ServoHack::setPotRange(uint16_t minPot, uint16_t maxPot) {
    pot.min = minPot;
    pot.max = maxPot;
    settings.pot = pot;
}

uint16_t ServoHack::read() {
    currentAngle = servo.read();
    return currentAngle;
}

uint16_t ServoHack::readPrev() {
    return prevAngle;
}

uint16_t ServoHack::readFeedback() {
    
    int16_t practicleAngle;
    if (potPin != DEFAULT_POT_PIN) {        // in case of no potPin, use the default servo library function
        practicleAngle = map(getPot(), pot.min, pot.max, angle.min, angle.max);
    } else {
        practicleAngle = read();
    }

    practicleAngle = constrain (practicleAngle, angle.min, angle.max); // constrain to min and max angle
    int16_t theoreticalAngle = read();
    uint16_t finalAngle = (abs(theoreticalAngle - practicleAngle) <= tolerance) ? theoreticalAngle : practicleAngle;
    return finalAngle;
}

uint16_t ServoHack::readMicroseconds() {
    return servo.readMicroseconds();
}

uint16_t ServoHack::getHome() {
    return home;
}

uint8_t ServoHack::getTolerance() {
    return tolerance;
}

uint16_t ServoHack::getPot() {
    if (potPin == DEFAULT_POT_PIN) {return 0;}
    
    const uint8_t usedReadings = POT_READINGS - (POT_DISCARD_READINGS * 2);
    static uint16_t readings[POT_READINGS];    

    for (int i = 0; i < POT_READINGS; i++) {
        readings[i] = analogRead(potPin);
        delay(1);
    }
    
    // sorting readings ...
    for (int i = 0; i < POT_READINGS - 1; i++) {
        for (int j = 0; j < POT_READINGS - 1 - i; j++) {
            if (readings[j] > readings[j + 1]) {
                uint16_t temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    
    uint32_t sum = 0;
    for (int i = POT_DISCARD_READINGS; i < POT_READINGS - POT_DISCARD_READINGS; i++) {
        sum += readings[i];
    }
    uint16_t potValue = sum / usedReadings;
    // potValue = constrain(potValue, pot.min, pot.max);

    return potValue;
}

bool ServoHack::isAttached() {
    return servo.attached();
}

bool ServoHack::isAtHome() {
    return (isAtAngle(home));
}

bool ServoHack::isAtAngle(uint16_t angle) {
    return (abs(readFeedback() - angle) <= tolerance);
}

bool ServoHack::getFailSafe() {
    return failSafe;
}

void ServoHack::setFailSafe(bool failSafe, void (*callback)()) {
    failSafeCallback = callback;
    this->failSafe = failSafe;
}

bool ServoHack::isCalibrated() {
    return (EEPROM.read(EEPROMAddress) == MAGIC_BYTE);
}

bool ServoHack::isCalibrated(uint16_t EEPROMAddress) {
    this->EEPROMAddress = EEPROMAddress;
    return isCalibrated();
}

void ServoHack::write(uint16_t angle) {
    prevAngle = currentAngle;
    currentAngle = constrain((int16_t)angle, this->angle.min, this->angle.max);  // avoiding out of range values
    servo.write(currentAngle); 
}

void ServoHack::write(uint16_t angle, uint8_t speed) {
    write(angle, speed, nullptr);
}

void ServoHack::write(uint16_t angle, uint8_t speed, void (*callback)()) {

    if (!isAttached()) return;

    int16_t startAngle = readFeedback();
    int16_t targetAngle = constrain((int16_t)angle, this->angle.min, this->angle.max);  // avoiding out of range values
    int16_t distance = abs(targetAngle - startAngle);
    if (distance == 0) return;

    uint16_t stepDelay = (1000 / speed);
    uint32_t lastStep = millis();

    prevAngle = currentAngle;
    for (int16_t stepAngle = startAngle; stepAngle != targetAngle; ) {
        if (millis() - lastStep >= stepDelay) {
            stepAngle += (targetAngle > startAngle) ? 1 : -1;
            servo.write(stepAngle);
            currentAngle = stepAngle;
            lastStep = millis();
        }
        
        // Check if the servo is at the stepAngle
        // if ((stepAngle % tolerance) == 0) {checkfailSafe(stepAngle);}
        if (callback) callback();

    }
    
    servo.write(targetAngle);
    currentAngle = targetAngle;

    // if failSafe is enabled, check if servo is actually at target angle or maybe struggling due to
    // mechanical obstruction, in which case it will not be able to reach the target angle.
    // e.g: software can say, goto 180 deg, but if servo is stuck at 90 deg, it will not be able to reach 180 deg
    // and it will eat up battery power and heat up the motor.
    // this feature is the main thing, for which this library is implemented.

    checkfailSafe(currentAngle);

}

void ServoHack::writeMicroseconds(uint16_t pulse) {
    prevAngle = currentAngle;
    servo.writeMicroseconds(pulse);
    currentAngle = read();
}

void ServoHack::writeMicroseconds(uint16_t pulse, uint8_t speed) {
    writeMicroseconds(pulse, speed, nullptr);
}

void ServoHack::writeMicroseconds(uint16_t pulse, uint8_t speed, void (*callback)()) {
    if (!isAttached()) return;

    uint16_t startPulse = readMicroseconds();
    uint16_t targetPulse = pulse;
    int16_t distance = abs(targetPulse - startPulse);
    if (distance == 0) return;

    // unsigned long stepDelay = (1000.0 / (float)(11.1112*speed)) * 5;
    if (angle.max == angle.min) return;  // avoid division by zero
    unsigned long stepDelay = (1000.0 / (float)((abs(this->pulse.max - this->pulse.min)/abs(angle.max - angle.min))*speed)) * 5;
    unsigned long lastStep = millis();
    int16_t tempAngle = 0; // temporary angle value

    prevAngle = currentAngle;
    for (uint16_t stepPulse = startPulse; abs(targetPulse - stepPulse) > 11; ) {
        tempAngle = read(); // read the angle from the servo
        if ((tempAngle <= angle.max) && (tempAngle >= angle.min)) {        
            if (millis() - lastStep >= stepDelay) {
                stepPulse += (targetPulse > startPulse) ? 5 : -5;  // almost half a degree of movement
                writeMicroseconds(stepPulse);
                lastStep = millis();
            }
        }
        // Check if the servo is at the stepPulse
        // if ((stepPulse % tolerance) == 0) {checkfailSafe(tempAngle);}
        if (callback) callback();
    }
    writeMicroseconds(targetPulse);
    currentAngle = read();

    // if failSafe is enabled, check if servo is actually at target angle or maybe struggling due to
    // mechanical obstruction, in which case it will not be able to reach the target angle.
    // e.g: software can say, goto 180 deg, but if servo is stuck at 90 deg, it will not be able to reach 180 deg
    // and it will eat up battery power and heat up the motor.
    // this feature is the main thing, for which this library is implemented.
    checkfailSafe(currentAngle);

}

void ServoHack::goHome() {
    write(home, DEFAULT_SPEED);
}

void ServoHack::goHome(uint8_t speed) {
    write(home, speed);
}

void ServoHack::checkfailSafe(uint16_t angle) {
    if (failSafe && !isAtAngle(angle)) {
        if (failSafeCallback) {
            failSafeCallback();
        } else {
            detach(); // to prevent any damage to the motor and the system by over current due to failure...
        }
    }
}