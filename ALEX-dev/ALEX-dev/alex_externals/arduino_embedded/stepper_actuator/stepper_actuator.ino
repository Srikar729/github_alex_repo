#include "Constants.h"
#include "MotorController.h"

#define upperLimit 6
#define lowerLimit 7
#define stepPin 3
#define dirPin 4
#define enablePin 5

MotorController motorController(stepPin, dirPin, enablePin, upperLimit, lowerLimit);

// Handles response and prints formatted output
void handleResponse(const MotorResponse& response) {
    String statusStr;
    switch (response.status) {
        case MotorStatus::ACK: statusStr = "ACK"; break;
        case MotorStatus::ERROR: statusStr = "ER"; break;
        case MotorStatus::LIMIT: statusStr = "LIMIT"; break;
        case MotorStatus::DONE: statusStr = "DONE"; break;
        case MotorStatus::WORKING: return;  // Ignore WORKING responses
        case MotorStatus::IDLE: return;  // Ignore IDLE responses
    }
    Serial.println(SerialBand_StepperActuator + statusStr + ":" + response.message);
}

MotorRequest handleRequest(const String& data) {
    long distance = data.toInt();

    if (distance == 0 && data != "0") {
        // If conversion failed and the string isn't just "0", return an invalid MotorRequest
        return MotorRequest{Direction::NEGATIVE, 0};
    }

    Direction dir = (distance < 0) ? Direction::NEGATIVE : Direction::POSITIVE;
    return MotorRequest{dir, abs(distance)};
}

void setup() {
    Serial.begin(115200);
    motorController.setup();
    Serial.println("System initialized");
    Serial.println("Enter '+x' to move forward, '-x' to move backward, '+0' to stop.");
}

void loop() {
    MotorResponse response = motorController.loop();
    handleResponse(response);

    if (Serial.available() <= 0) return;

    String serial_data = Serial.readStringUntil(10);
    MotorRequest commandRequest = handleRequest(serial_data);

    MotorResponse commandResponse = motorController.handleCommand(commandRequest);
    handleResponse(commandResponse);
}
