#include <SoftwareSerial.h>

#include "Constants.h"
#include "EzoPumpController.h"
#include "UltrasonicSensor.h"

#define PUMP_RX_PIN 2
#define PUMP_TX_PIN 3
#define US_TRIG_PIN 4
#define US_ECHO_PIN 5

EzoPumpController pumpController(PUMP_RX_PIN, PUMP_TX_PIN);
UltrasonicSensor u_sensor(US_TRIG_PIN, US_ECHO_PIN);

unsigned long initial_time;

void update_usensor(){
    if (millis() - initial_time < 5000) return;

    initial_time = millis();
    int liquid_level_distance = u_sensor.getDistance();
    if (liquid_level_distance <= 0) return;

    String liquid_level_output = SerialBand_LiquidLevel + String(liquid_level_distance);
    Serial.println(liquid_level_output);
}

void update_pump(){
    String response = pumpController.update();
    if (response.length() <= 0) return;

    String liquid_dosing_output = SerialBand_LiquidDosing + response;
    Serial.println(liquid_dosing_output);
}

void setup() {
    Serial.begin(115200);
    pumpController.begin(9600);
    u_sensor.begin();

    initial_time = millis();
}

void serialEvent() {
    String inputString = Serial.readStringUntil(10);  // Read from Serial until <CR>
    pumpController.processCommand(inputString);       // Pass command to pump controller
}

void loop() {
    update_pump();
    update_usensor();
}
