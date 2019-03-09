#ifndef Rampa_h
#define Rampa_h

#include "arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_MotorShield.h>
#define BNO055_SAMPLERATE_DELAY_MS 10

class Rampa {

public:
    Rampa();
    Adafruit_DCMotor *MotorAdeIzq;
    Adafruit_DCMotor *MotorAdeDer;
    Adafruit_DCMotor *MotorAtrasIzq;
    Adafruit_DCMotor *MotorAtrasDer;
    Adafruit_MotorShield AFMS;
    Adafruit_BNO055 bno;
    int detectaRampa();
    void setup();


};


#endif
