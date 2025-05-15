#include "MotorController.h"

MotorController::MotorController(int motorStepPin, int motorDirectionPin, int motorEnablePin, int positiveLimitSwitchPin, int negativeLimitSwitchPin) {
    positiveLimitSwitch = positiveLimitSwitchPin;
    negativeLimitSwitch = negativeLimitSwitchPin;
    stepPin   = motorStepPin;
    dirPin    = motorDirectionPin;
    enablePin = motorEnablePin;
}

void MotorController::setup() {
    pinMode(stepPin,   OUTPUT);
    pinMode(dirPin,    OUTPUT);
    pinMode(enablePin, OUTPUT);

    pinMode(positiveLimitSwitch, INPUT_PULLUP);
    pinMode(negativeLimitSwitch, INPUT_PULLUP);
}

MotorResponse MotorController::loop() {
    if (!motorRunning) return {MotorStatus::IDLE};
    
    if (currentRequest.dir == Direction::POSITIVE && reachedPositiveLimit()) {
        stopMotor();
        return {MotorStatus::LIMIT, "+"};
    } else if (currentRequest.dir == Direction::NEGATIVE && reachedNegativeLimit()) {
        stopMotor();
        return {MotorStatus::LIMIT, "-"};
    }

    MotorResponse runMotorStatus = runMotor();
    return runMotorStatus;
}

MotorResponse MotorController::handleCommand(MotorRequest request) {

    currentRequest = request;
    if (request.dir == Direction::NEGATIVE && request.distance == 0) {
        stopMotor();
        return {MotorStatus::ERROR, "Invalid request"};
    }

    switch (request.dir) {
        case Direction::POSITIVE:
            motorRunning = true;
            MotorController::setMovementDirection(request.dir);
            totalRemainingSteps = request.distance * STEPS_PER_MM;
            // loop should start the movement
            return {MotorStatus::ACK, "+" + String(request.distance)};
        case Direction::NEGATIVE:
            motorRunning = true;
            totalRemainingSteps = request.distance * STEPS_PER_MM;
            MotorController::setMovementDirection(request.dir);
            // loop should start the movement
            return {MotorStatus::ACK, "-" + String(request.distance)};
    }
    return {MotorStatus::ERROR, "Unknown error"};
}

void MotorController::setMovementDirection(Direction direction) {
    digitalWrite(dirPin, direction == Direction::POSITIVE ? HIGH : LOW);
}

MotorResponse MotorController::runMotor() {
    if (totalRemainingSteps<0) {
        stopMotor();
        return {MotorStatus::DONE};
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(MOTOR_CYCLE_DELAY);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(MOTOR_CYCLE_DELAY);
    
    totalRemainingSteps--;
    return {MotorStatus::WORKING};
}

bool MotorController::reachedPositiveLimit() {
    return (digitalRead(positiveLimitSwitch) == LOW);
}

bool MotorController::reachedNegativeLimit() {
    return (digitalRead(negativeLimitSwitch) == LOW);
}

void MotorController::stopMotor() {
    motorRunning = false;
    totalRemainingSteps = 0;
    currentRequest = {};
}
