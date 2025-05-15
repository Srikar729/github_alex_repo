#include "MotorController.h"

MotorController::MotorController(int in1, int in2, int speedPin, int reedForward, int reedBackward, int speed) 
    : motorIn1(in1), motorIn2(in2), motorSpeedPin(speedPin),
      reedPinForward(reedForward), reedPinBackward(reedBackward),
      motorSpeed(speed), motorForward(false), motorRunning(false) {}

void MotorController::setup() {
    pinMode(motorIn1, OUTPUT);
    pinMode(motorIn2, OUTPUT);
    pinMode(motorSpeedPin, OUTPUT);
    pinMode(reedPinForward, INPUT_PULLUP);
    pinMode(reedPinBackward, INPUT_PULLUP);
}

MotorResponse MotorController::loop() {
    if (!motorRunning) return {MotorStatus::IDLE, ""};
    
    if (motorForward && digitalRead(reedPinForward) == LOW) {
        stopMotor();
        return {MotorStatus::LIMIT, "F"};
    } else if (!motorForward && digitalRead(reedPinBackward) == LOW) {
        stopMotor();
        return {MotorStatus::LIMIT, "B"};
    }
    return {MotorStatus::IDLE, ""};
}

MotorResponse MotorController::handleCommand(char command) {
    switch (command) {
        case 'F': case 'f':
            motorForward = true;
            motorRunning = true;
            runMotor(HIGH, LOW);
            return {MotorStatus::ACK, "F"};
        case 'B': case 'b':
            motorForward = false;
            motorRunning = true;
            runMotor(LOW, HIGH);
            return {MotorStatus::ACK, "B"};
        case 'S': case 's':
            stopMotor();
            return {MotorStatus::ACK, "S"};
        default:
            return {MotorStatus::ERROR, "Invalid"};
    }
}

void MotorController::runMotor(int in1State, int in2State) {
    digitalWrite(motorIn1, in1State);
    digitalWrite(motorIn2, in2State);
    analogWrite(motorSpeedPin, motorSpeed);
}

void MotorController::stopMotor() {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorSpeedPin, 0);
    motorRunning = false;
}
