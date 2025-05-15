#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

// TODO: Should move from here
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 2
#define STEPS_PER_REV (STEPS_PER_REVOLUTION * MICROSTEPS)
#define DISTANCE_PER_REVOLUTION_MM 10 // Distance moved per revolution (e.g., 5 cm per revolution)
#define STEPS_PER_MM (STEPS_PER_REV / DISTANCE_PER_REVOLUTION_MM) // Steps per centimeter
#define MOTOR_CYCLE_DELAY 300

enum class MotorStatus {
    ACK,
    ERROR,
    LIMIT,
    DONE,
    WORKING,
    IDLE
};

enum class Direction {
    POSITIVE,
    NEGATIVE
};

struct MotorRequest {
    Direction dir;
    long distance;
};

struct MotorResponse {
    MotorStatus status;
    String message;
};

class MotorController {
public:
    MotorController(int motorStepPin, int motorDirectionPin, int motorEnablePin, int positiveLimitSwitchPin, int negativeLimitSwitchPin);
    void setup();
    MotorResponse loop();
    MotorResponse handleCommand(MotorRequest command);

private:
    bool motorRunning;
    int totalRemainingSteps;
    int stepPin, dirPin, enablePin;
    int positiveLimitSwitch, negativeLimitSwitch;

    MotorRequest currentRequest;

    MotorResponse runMotor();
    void setMovementDirection(Direction request);
    bool reachedNegativeLimit();
    bool reachedPositiveLimit();
    void stopMotor();
};

#endif // MOTORCONTROLLER_H
