#include "Constants.h"
#include "MotorController.h"

#define motorIn1 6
#define motorIn2 5
#define forwardLimitPin 2
#define backwardLimitPin 3
#define motorSpeedPin 9
#define motorSpeed 60

MotorController motorController(motorIn1, motorIn2, motorSpeedPin, forwardLimitPin, backwardLimitPin, motorSpeed);

// Handles response and prints formatted output
void handleResponse(const MotorResponse& response) {
    String statusStr;
    switch (response.status) {
        case MotorStatus::ACK: statusStr = "ACK"; break;
        case MotorStatus::ERROR: statusStr = "ERROR"; break;
        case MotorStatus::LIMIT: statusStr = "LIMIT"; break;
        case MotorStatus::IDLE: return;  // Ignore IDLE responses
    }
    Serial.println(SerialBand_DCActuator + statusStr + ":" + response.message);
}

void setup() {
    Serial.begin(115200);
    motorController.setup();
    Serial.println("System initialized. Enter 'F' to move forward, 'B' to move backward, 'S' to stop.");
}

void loop() {
    MotorResponse response = motorController.loop();
    handleResponse(response);

    if (Serial.available() <= 0) return;

    String serial_data = Serial.readStringUntil(10);
    char command = serial_data.charAt(0);

    MotorResponse commandResponse = motorController.handleCommand(command);
    handleResponse(commandResponse);
}
