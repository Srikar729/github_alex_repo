#include <Adafruit_MAX31865.h>

#ifndef Pt100SENSOR_H
#define Pt100SENSOR_H

#define RREF      430.0
#define RNOMINAL  100.0

class PT100Sensor {
  private:
    Adafruit_MAX31865 thermo;

    int ON;  // Power pin for the sensor

  public:
    PT100Sensor(int power, int CS, int SDI, int SDO, int CLK);
    void begin();
    float get_temperature();
    // void get_fault(); // TODO:  Need to implement this
};

#endif
