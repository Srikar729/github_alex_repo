#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

enum class MotorStatus {
    ACK,
    ERROR,
    LIMIT,
    IDLE
};

struct MotorResponse {
    MotorStatus status;
    String message;
};

class MotorController {
public:
    MotorController(int in1, int in2, int speedPin, int reedForward, int reedBackward, int speed);
    void setup();
    MotorResponse loop();
    MotorResponse handleCommand(char command);

private:
    int motorIn1, motorIn2, motorSpeedPin;
    int reedPinForward, reedPinBackward;
    int motorSpeed;
    bool motorForward, motorRunning;

    void runMotor(int in1State, int in2State);
    void stopMotor();
};

#endif // MOTORCONTROLLER_H
