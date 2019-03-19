  #include "Rampa.h"




Rampa::Rampa(){
bno = Adafruit_BNO055();
}

void Rampa::setup(){

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

 if(med<-3){
    Serial.println("Entramos");
    do{
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 235);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 235);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 220);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 220);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.y();
    if(newMed>-1){
        newMed=-5;
    }
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    }while(newMed<-3);
    Serial.println("Salimos");
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 200);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 200);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(150);
    valor++;
    }
    else if(med>8){
    Serial.println("Entramos");

    do{

            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 130);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 130);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 160);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 160);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.y();
    if(newMed<4){
        newMed=10;
    }
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    }while(newMed>8);
    Serial.println("Salimos");
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 200);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 200);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(150);

    valor++;
    }

    return valor;
}
