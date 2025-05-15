#include "Pt100Sensor.h"
#include <Arduino.h>

PT100Sensor::PT100Sensor(int power, int CS, int SDI, int SDO, int CLK) : thermo(CS, SDI, SDO, CLK) {
    ON = power;
}

// Initialization method
void PT100Sensor::begin() {
    pinMode(ON, OUTPUT);
    thermo.begin(MAX31865_3WIRE);

    delay(500); // Short delay to allow sensor to stabilize after power-up
    digitalWrite(ON, HIGH);    // Turn on the sensor power
}

float PT100Sensor::get_temperature() {
    float temperature = thermo.temperature(RNOMINAL, RREF);
    return temperature;
}
