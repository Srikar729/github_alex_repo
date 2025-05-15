#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

class UltrasonicSensor {
  private:
    int trigPin;
    int echoPin;
    long duration;
  
  public:
    UltrasonicSensor(int trig, int echo);
    void begin();
    int getDistance();
};

#endif
