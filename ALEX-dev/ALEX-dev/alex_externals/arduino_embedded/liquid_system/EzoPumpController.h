#ifndef EZO_PUMP_CONTROLLER_H
#define EZO_PUMP_CONTROLLER_H

#include <SoftwareSerial.h>
#include <Arduino.h>

class EzoPumpController {
  private:
    SoftwareSerial pumpSerial;
    String deviceString;
    bool deviceStringComplete;

  public:
    // Constructor initializes the pump serial communication
    EzoPumpController(uint8_t rx, uint8_t tx);

    // Initialize the pump serial communication (without Serial.begin)
    void begin(long baudRate);

    // Process a command received from the PC and send it to the pump
    void processCommand(const String &command);

    // Update pump responses; returns a complete response if available
    String update();

  private:
    // Handle pump response processing
    void handleDeviceResponse();
};

#endif // EZO_PUMP_CONTROLLER_H
