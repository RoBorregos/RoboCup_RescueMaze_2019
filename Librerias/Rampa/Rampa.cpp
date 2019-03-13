#include "Rampa.h"




Rampa::Rampa(){
bno = Adafruit_BNO055();
AFMS = Adafruit_MotorShield();
MotorAdeIzq = AFMS.getMotor(4);
MotorAdeDer = AFMS.getMotor(1);
MotorAtrasIzq = AFMS.getMotor(3);
MotorAtrasDer = AFMS.getMotor(2);
}

void Rampa::setup(){
    AFMS.begin();
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   delay(1000);
  bno.setExtCrystalUse(true);
}


int Rampa::detectaRampa(){
 uint8_t i;
 int valor = 0;
double med=0, newMed=0, punto=0, newPunto=0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print(" Y: ");
    med= euler.y();
    Serial.println(med);
    delay(BNO055_SAMPLERATE_DELAY_MS);

 if(med<-1){
    Serial.println("Entramos");
    do{
    MotorAtrasDer->setSpeed(150);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(150);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(150);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(150);
    MotorAdeDer->run(BACKWARD);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.y();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    }while(newMed<-2);
    Serial.println("Salimos");
    valor++;
    }
    else if(med>8){
    Serial.println("Entramos");
    MotorAtrasDer->setSpeed(235);
    MotorAtrasDer->run(BACKWARD);
    MotorAtrasIzq->setSpeed(255);
    MotorAtrasIzq->run(BACKWARD);
    MotorAdeIzq->setSpeed(255);
    MotorAdeIzq->run(FORWARD);
    MotorAdeDer->setSpeed(235);
    MotorAdeDer->run(FORWARD);
    delay(1000);
    MotorAtrasDer->setSpeed(235);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(255);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(255);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(235);
    MotorAdeDer->run(BACKWARD);
    delay(1200);
    do{
    MotorAtrasDer->setSpeed(235);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(255);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(255);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(235);
    MotorAdeDer->run(BACKWARD);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.y();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    }while(newMed>8);
    Serial.println("Salimos");

    valor++;
    }
    return valor;
}

