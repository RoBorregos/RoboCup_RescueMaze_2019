  #include "Rampa.h"
double Kp3=9, Ki3=0, Kd3=0;
double Input3=0, Output3=0, Setpoint3=0;
double Kp4=9, Ki4=0, Kd4=0;
double Input4=0, Output4=0, Setpoint4=0;
PID myPID3(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID myPID4(&Input4, &Output4, &Setpoint3, Kp4, Ki4, Kd4, REVERSE);


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
  myPID3.SetMode(AUTOMATIC);
  myPID4.SetMode(AUTOMATIC);
}


int Rampa::detectaRampa(){
 uint8_t i;
 int valor = 0;
double med=0, newMed=0, punto=0, newPunto=0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print(" Y: ");
    med= euler.y();
    Setpoint3=euler.x();
    Serial.println(med);
    delay(BNO055_SAMPLERATE_DELAY_MS);

 if(med<-8){
    Serial.println("Entramos");
    do{
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    med=euler.x();

    delay(BNO055_SAMPLERATE_DELAY_MS);
newMed=euler.y();

    bool flag = false;
    if((med > Setpoint3 && med-180 < Setpoint3)||(med < Setpoint3 && med+180 < Setpoint3)){
       flag = true;
     }
     if(flag && med < Setpoint3)
       med += 360;
     else if (!flag && med > Setpoint3)
       med -= 360;
    Input3=med;
    Input4=med;

    myPID3.Compute();
    myPID4.Compute();
    if(Input3-Setpoint3>1 || Input3-Setpoint3 <-1){
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 230+(Output4*3));
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 230);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 170-(Output3*.25));
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 230);
            Serial.print(Output3);
            Serial.print(" ");
            Serial.print(Output4);
            Serial.print(" ");
            Serial.print(Setpoint3);
            Serial.print(" ");
            Serial.println(Input3);
    }
    else{
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 240);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 240);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 220);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 220);


    }

if(newMed>-1){
        newMed=-5;
}
 }while(newMed<-8);
    Serial.println("Salimos");
            digitalWrite(motorIzqAde1, LOW);
            analogWrite(motorIzqAde2, 200);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 200);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(400);
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
            analogWrite(motorDerAde2, 145);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 145);
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
            delay(400);

    valor++;
    }

    return valor;

}
