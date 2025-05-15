#ifndef SONICATOR_H
#define SONICATOR_H

enum Command {
    Heating,
    Sonication,
    Unknown
};

class Sonicator {
  private:
    int heaterPin;
    int sonicatorPin;
    int heaterIndicatorPin;
    int sonicatorIndicatorPin;
  
  public:
    Sonicator(int heater, int sonicator, int heater_indicator, int sonicator_indicator);
    void begin();
    void set_sonication(bool should_on);
    void set_heater(bool should_on);
};

#endif
