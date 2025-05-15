#include "Sonicator.h"
#include "Constants.h"
#include "Pt100Sensor.h"

#define HEATER_PIN 6
#define SONICATOR_PIN 7
#define HEATER_LED_PIN 4
#define SONICATOR_LED_PIN 5

Sonicator sonicator(HEATER_PIN, SONICATOR_PIN, HEATER_LED_PIN, SONICATOR_LED_PIN);

#define THERMO_ON 9
#define THERMO_CS 10
#define THERMO_DI 11
#define THERMO_DO 12
#define THERMO_CL 13

PT100Sensor sensor = PT100Sensor(THERMO_ON, THERMO_CS, THERMO_DI, THERMO_DO, THERMO_CL);

unsigned long initial_time;

struct ParsedInput {
    Command command = Unknown;
    bool value = false;
};

void update_temperature(){
    if (millis() - initial_time < 5000) return;

    initial_time = millis();
    float temperature = sensor.get_temperature();

    String temperature_output = SerialBand_SonicatorTemperature + String(temperature);
    Serial.println(temperature_output);
}

ParsedInput process_input(const String &inputString){
    ParsedInput result;
    if (inputString.startsWith("SH:")) {
        result.command = Heating;
        result.value = inputString.charAt(3) == '1';
    } else if (inputString.startsWith("SS:")) {
        result.command = Sonication;
        result.value = inputString.charAt(3) == '1';
    }
    return result;
}

void process_command(ParsedInput data){
    switch (data.command) {
        case Heating:
            sonicator.set_heater(data.value);
            break;
        case Sonication:
            sonicator.set_sonication(data.value);
            break;
        case Unknown:
            Serial.println(SerialBand_Error+"Unknown");
            break;
    }
}

void setup() {
    Serial.begin(115200);
    sonicator.begin();
    sensor.begin();

    initial_time = millis();
}

void loop() {
    update_temperature();

    if (Serial.available() <= 0) {
        return; // Exit if there's no serial input
    }

    String inputString = Serial.readStringUntil(10);
    ParsedInput processed_data = process_input(inputString);
    process_command(processed_data);
}
