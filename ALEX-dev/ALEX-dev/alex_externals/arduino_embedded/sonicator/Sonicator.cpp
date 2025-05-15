#include "Sonicator.h"
#include <Arduino.h>

Sonicator::Sonicator(int heater, int sonicator, int heaterIndicator, int sonicatorIndicator) {
    heaterPin = heater;
    sonicatorPin = sonicator;
    heaterIndicatorPin = heaterIndicator;
    sonicatorIndicatorPin = sonicatorIndicator;
}

void Sonicator::begin() {
    pinMode(heaterPin, OUTPUT);
    pinMode(sonicatorPin, OUTPUT);
    pinMode(heaterIndicatorPin, OUTPUT);
    pinMode(sonicatorIndicatorPin, OUTPUT);
}

void Sonicator::set_sonication(bool should_on){
    int output = should_on ? HIGH : LOW;
    digitalWrite(sonicatorPin, output);
    digitalWrite(sonicatorIndicatorPin, output);
}

void Sonicator::set_heater(bool should_on){
    int output = should_on ? HIGH : LOW;
    digitalWrite(heaterIndicatorPin, output);
    digitalWrite(heaterPin, output);
}
