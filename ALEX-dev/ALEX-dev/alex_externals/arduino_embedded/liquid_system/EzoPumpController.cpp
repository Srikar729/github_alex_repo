#include "EzoPumpController.h"

EzoPumpController::EzoPumpController(uint8_t rx, uint8_t tx)
  : pumpSerial(rx, tx), deviceString(""), deviceStringComplete(false) {}

void EzoPumpController::begin(long baudRate) {
  pumpSerial.begin(baudRate);  // Initialize pump serial communication only
  deviceString.reserve(30);    // Reserve memory for incoming data from the pump
}

void EzoPumpController::processCommand(const String &command) {
  pumpSerial.print(command);  // Send the command to the pump
  pumpSerial.print('\r');     // Add carriage return
}

String EzoPumpController::update() {
  handleDeviceResponse();      // Check for and handle responses from the pump
  if (deviceStringComplete) {  // If a complete response is available
    String response = deviceString; // Copy response to a new variable
    deviceString = "";          // Clear device buffer
    deviceStringComplete = false; // Reset completion flag
    return response;            // Return the complete response
  }
  return "";                    // Return empty string if no complete response
}

void EzoPumpController::handleDeviceResponse() {
  while (pumpSerial.available() > 0) {      // If data is available from the pump
    char inChar = (char)pumpSerial.read();  // Read next character
    deviceString += inChar;                 // Append character to deviceString
    if (inChar == '\r') {                   // Check for end of message
      deviceStringComplete = true;          // Mark response complete
    }
  }
}
