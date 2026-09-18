/**
 * Simple PWM-based Servo Control for Arduino Uno R4 WiFi
 *
 * Since the standard Servo library is not available in PlatformIO for Renesas,
 * this provides a simple software PWM implementation.
 */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <Arduino.h>

class ServoPWM {
private:
    int pin;
    int angle;
    bool attached;

    void writeMicroseconds(int us) {
        if (!attached) return;
        // Standard servo pulse: 500-2500us for 0-180 degrees
        // Period: 20ms (50Hz)
        digitalWrite(pin, HIGH);
        delayMicroseconds(us);
        digitalWrite(pin, LOW);
    }

public:
    ServoPWM() : pin(-1), angle(90), attached(false) {}

    void attach(int p) {
        pin = p;
        pinMode(pin, OUTPUT);
        attached = true;
    }

    void detach() {
        attached = false;
    }

    void write(int ang) {
        if (!attached) return;
        angle = constrain(ang, 0, 180);
        // Map angle (0-180) to pulse width (500-2500us)
        int us = map(angle, 0, 180, 500, 2500);
        writeMicroseconds(us);
    }

    int read() {
        return angle;
    }

    // Call this in loop() to maintain servo position
    void refresh() {
        if (!attached) return;
        int us = map(angle, 0, 180, 500, 2500);
        writeMicroseconds(us);
    }
};

#endif // SERVO_PWM_H
