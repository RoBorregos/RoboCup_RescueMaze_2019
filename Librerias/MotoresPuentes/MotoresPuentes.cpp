 
#include "MotoresPuentes.h"

double Kp=20, Ki=0, Kd=0;
double Input=0, Output=0, Setpoint=0;
double Kp2=15, Ki2=0, Kd2=0;
double Input2=0, Output2=0, Setpoint2=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, Kp2, Ki2, Kd2, REVERSE);

LiquidCrystal_I2C lcd(0x3F,16,2);

double velocidadBaseIzqAde=180;
double velocidadBaseIzqAtras=180;
double velocidadBaseDerAde=200;
double velocidadBaseDerAtras=200;

//CAMBIAR EL DRIRECT Y REVERSE EN LAS VUELTAS YA QUE UNAS LAS DA LENTO DEBIDO A QUE TIENE UN REVERSE Y DIRECT AL REVES



MotoresPuentes::MotoresPuentes(){
bno = Adafruit_BNO055();
}

void MotoresPuentes::setup(){

    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   delay(1000);
  bno.setExtCrystalUse(true);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  pinMode(motorIzqAde1, OUTPUT);
  pinMode(motorIzqAde2, OUTPUT);
  pinMode(motorIzqAtras1, OUTPUT);
  pinMode(motorIzqAtras2, OUTPUT);
  pinMode(motorDerAde1, OUTPUT);
  pinMode(motorDerAde2, OUTPUT);
  pinMode(motorDerAtras1, OUTPUT);
  pinMode(motorDerAtras2, OUTPUT);
   lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
}
void MotoresPuentes::actualizaSetpoint(){
    double med=0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //Serial.print("X: ");
    med=euler.x();
    Setpoint=med;
    //Setpoint = Setpoint -90 >0 ? Setpoint-90 : 360+ (Setpoint-90);
    //Serial.println(med);
    delay(BNO055_SAMPLERATE_DELAY_MS);
}
void MotoresPuentes::moveAdelanteFast(){
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(800);
}
void MotoresPuentes::moveAdelante(){
    double med=0;
    double medAux=0;
    double velocidadIzqAde, velocidadIzqAtras, velocidadDerAde, velocidadDerAtras;
    velocidadIzqAde=velocidadBaseIzqAde;
    velocidadIzqAtras=velocidadBaseIzqAtras;
    velocidadDerAde=velocidadBaseDerAde;
    velocidadDerAtras=velocidadBaseDerAtras;
    double n2Output=0;
    double nOutput=0;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    med=euler.x();

    delay(BNO055_SAMPLERATE_DELAY_MS);


    bool flag = false;
    if((med > Setpoint && med-180 < Setpoint)||(med < Setpoint && med+180 < Setpoint)){
       flag = true;
     }
     if(flag && med < Setpoint)
       med += 360;
     else if (!flag && med > Setpoint)
       med -= 360;
    Input2=med;
    Input=med;

    myPID.Compute();
    myPID2.Compute();
    if(Input-Setpoint>1 || Input-Setpoint <-1){
            n2Output=200+Output2-Output;
            (n2Output>=255)? n2Output=255: n2Output=n2Output;
            nOutput=180+Output-Output2*2;
            (nOutput>=255)? nOutput=255: nOutput=nOutput;



            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, nOutput);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, nOutput);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, n2Output);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, n2Output);
            Serial.print("Angulo: ");
            Serial.print(Input);
            Serial.print(" Setpoint: ");
            Serial.print(Setpoint);
            Serial.print(" Output1: ");
            Serial.print(nOutput);
            Serial.print(" Output2: ");
            Serial.println(n2Output);

    }
    else{
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 180);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 180);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 200);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 200);


    }

}

void MotoresPuentes::acomodoD()
{
  uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 50);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 50);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 170);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 170);
/*
    MotorAtrasDer->setSpeed(170);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(50);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(50);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(170);
    MotorAdeDer->run(BACKWARD);
    */
}
void MotoresPuentes::acomodoI()
{
 uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 170);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 170);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 50);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 50);
/*
  MotorAtrasDer->setSpeed(50);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(170);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(170);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(50);
    MotorAdeDer->run(BACKWARD);
    */
}
void MotoresPuentes::detenerse(){
  uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde1, HIGH);
digitalWrite(motorIzqAde2, HIGH);
digitalWrite(motorIzqAtras1, HIGH);
digitalWrite(motorIzqAtras2, HIGH);
digitalWrite(motorDerAde1, HIGH);
digitalWrite(motorDerAde2, HIGH);
digitalWrite(motorDerAtras1, HIGH);
digitalWrite(motorDerAtras2, HIGH);

delay(10);

digitalWrite(motorIzqAde1, LOW);
digitalWrite(motorIzqAde2, LOW);
digitalWrite(motorIzqAtras1, LOW);
digitalWrite(motorIzqAtras2, LOW);
digitalWrite(motorDerAde1, LOW);
digitalWrite(motorDerAde2, LOW);
digitalWrite(motorDerAtras1, LOW);
digitalWrite(motorDerAtras2, LOW);
}
void MotoresPuentes::moveAtras(){
uint8_t i;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......

digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 230);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 230);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
/*
    MotorAtrasDer->setSpeed(210);
    MotorAtrasDer->run(BACKWARD);
    MotorAtrasIzq->setSpeed(235);
    MotorAtrasIzq->run(BACKWARD);
    MotorAdeIzq->setSpeed(235);
    MotorAdeIzq->run(FORWARD);
    MotorAdeDer->setSpeed(210);
    MotorAdeDer->run(FORWARD);
    */
}
void MotoresPuentes::moveAtrasLento(){
 uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 50);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 50);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 50);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 50);
/*
    MotorAtrasDer->setSpeed(50);
    MotorAtrasDer->run(BACKWARD);
    MotorAtrasIzq->setSpeed(50);
    MotorAtrasIzq->run(BACKWARD);
    MotorAdeIzq->setSpeed(50);
    MotorAdeIzq->run(FORWARD);
    MotorAdeDer->setSpeed(50);
    MotorAdeDer->run(FORWARD);
    */
}
void MotoresPuentes::moveIzq(){
uint8_t i;
double med=0, newMed=0, punto=0, newPunto=0, nOutput=0;
long previousMillis = 0;
long interval = 5000;
unsigned long currentMillis;
long objective=0;
long objective2=0;
int balance=0;
int balance2=0;
int caster=0;
int locker=0;


objective= millis();
objective+= interval;
objective2+=interval+5000;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    med=euler.x();
    Serial.println(med);
    delay(BNO055_SAMPLERATE_DELAY_MS);

if(med<90){

    punto=med-83;
    newPunto=360+punto;
    Setpoint2=newPunto;



    do{
    currentMillis = millis();
    if(currentMillis < objective){
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Input2=newMed;
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    myPID2.Compute();

    nOutput=Output2+200;
    (nOutput>=255)? nOutput=255: nOutput=nOutput;
    Serial.println(nOutput);

digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 200);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 200);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 255);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 255);
    }
    else if(currentMillis >= objective && currentMillis < objective2){

    if(balance<3){
            //MOVIMIENTO DERECHA
    digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, 255);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, 255);
    digitalWrite(motorDerAde2, LOW);
    analogWrite(motorDerAde1, 255);
    digitalWrite(motorDerAtras2, LOW);
    analogWrite(motorDerAtras1, 255);
    delay(250);
        //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(250);
            caster++;
            balance++;

    }
    else if(caster>=2 && caster<5){
            //MOVIMIENTO IZQUIERDA
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(250);
    //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
        delay(250);
caster++;
    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    }
    }
    else{
    //MOVIMIENTO IZQUIERDA
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(300);
    //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(800);
            break;
    }
    }while(newMed<=med+180 || newMed>newPunto);
  }
  else{
    punto=med-83;
    Setpoint=punto;
    do{
     currentMillis = millis();
    if(currentMillis < objective){
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Input2=newMed;
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    myPID2.Compute();

    nOutput=Output2+200;
    (nOutput>=255)? nOutput=255: nOutput=nOutput;
    Serial.println(nOutput);

digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 200);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 200);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 255);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 255);
    }
    else if(currentMillis >= objective && currentMillis < objective2){

    if(balance<3){
            //MOVIMIENTO DERECHA
    digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, 255);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, 255);
    digitalWrite(motorDerAde2, LOW);
    analogWrite(motorDerAde1, 255);
    digitalWrite(motorDerAtras2, LOW);
    analogWrite(motorDerAtras1, 255);
    delay(250);
        //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(250);
            caster++;
            balance++;

    }
    else if(caster>=2 && caster<5){
            //MOVIMIENTO IZQUIERDA
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(250);
    //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
        delay(250);
caster++;
    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    }
    }
    else{
    //MOVIMIENTO IZQUIERDA
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(300);
    //MOVIMIENTO HACIA ADELANTE
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(800);
            break;
    }
    }while(newMed>punto);
  }
  Serial.println("YA SALIO****");
  digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, LOW);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, LOW);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, LOW);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, LOW);
}

void MotoresPuentes::moveDer(){
uint8_t i;
double med=0, newMed=0, punto=0, newPunto=0, nOutput=0;
long previousMillis = 0;
long interval = 5000;
unsigned long currentMillis;
long objective=0;
long objective2=0;
int balance=0;
int balance2=0;
int caster=0;
int locker=0;

objective= millis();
objective+= interval;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    med=euler.x();
    Serial.println(med);
    delay(BNO055_SAMPLERATE_DELAY_MS);

    if(med>=270){

    punto=med+83;
    newPunto=(360-punto)*-1;
    Setpoint=newPunto;
    do{
    currentMillis = millis();
    if(currentMillis < objective){
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Input2=newMed;
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    myPID2.Compute();

    nOutput=Output2+150;
    (nOutput>=255)? nOutput=255: nOutput=nOutput;
    Serial.println(nOutput);

digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 220);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 220);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 240);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 240);
    }
    else if(currentMillis >= objective){
    if(locker<1){
    objective2= millis();
    objective+= interval;
    locker++;
    }
    if(currentMillis<objective2){
    if(balance<2){
    //***
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(200);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(150);
            caster++;
            balance++;

    }
    else if(caster>=2 && caster<4){
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    delay(150);
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
delay(150);
caster++;
    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    }
    }
    else if(currentMillis>=objective2){
     if(balance2<3){
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    delay(200);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 200);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 200);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 255);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 255);
            delay(150);
            balance2++;

    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    }
    }
    }
    }while(newMed>=med || newMed<newPunto);
  }
  else{
    punto=med+83;
    Setpoint=punto;
    do{
    currentMillis = millis();
    if(currentMillis < objective){
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Input2=newMed;
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    myPID2.Compute();

    nOutput=Output2+150;
    (nOutput>=255)? nOutput=255: nOutput=nOutput;
    Serial.println(nOutput);

digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 220);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 220);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 240);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 240);
    }
    else if(currentMillis >= objective){
    if(locker<1){
    objective2= millis();
    objective+= interval;
    locker++;
    }
    if(currentMillis<objective2){
    if(balance<2){
    //***
    digitalWrite(motorIzqAde1, LOW);
    analogWrite(motorIzqAde2, 255);
    digitalWrite(motorIzqAtras2, LOW);
    analogWrite(motorIzqAtras1, 255);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, 255);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, 255);
    delay(200);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
            delay(150);
            caster++;
            balance++;

    }
    else if(caster>=2 && caster<4){
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    delay(150);
            digitalWrite(motorIzqAde2, LOW);
            analogWrite(motorIzqAde1, 255);
            digitalWrite(motorIzqAtras1, LOW);
            analogWrite(motorIzqAtras2, 255);
            digitalWrite(motorDerAde1, LOW);
            analogWrite(motorDerAde2, 255);
            digitalWrite(motorDerAtras1, LOW);
            analogWrite(motorDerAtras2, 255);
delay(150);
caster++;
    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    }
    }
    else if(currentMillis>=objective2){
     if(balance2<3){
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    delay(200);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, 200);
digitalWrite(motorIzqAtras2, LOW);
analogWrite(motorIzqAtras1, 200);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 255);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 255);
            delay(150);
            balance2++;

    }
    else{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    newMed=euler.x();
    Serial.println(newMed);
    delay(BNO055_SAMPLERATE_DELAY_MS);
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 255);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 255);
digitalWrite(motorDerAde2, LOW);
analogWrite(motorDerAde1, 255);
digitalWrite(motorDerAtras2, LOW);
analogWrite(motorDerAtras1, 255);
    }
    }
    }
    }while(newMed<punto);
  }
  Serial.println("YA SALIO****");
   digitalWrite(motorIzqAde1, LOW);
analogWrite(motorIzqAde2, LOW);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, LOW);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, LOW);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, LOW);
}
void MotoresPuentes::moveDerAcomodo(){
    uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 80);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 80);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 80);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 80);
    /*
   MotorAtrasDer->setSpeed(80);
    MotorAtrasDer->run(FORWARD);
    MotorAtrasIzq->setSpeed(80);
    MotorAtrasIzq->run(BACKWARD);
    MotorAdeIzq->setSpeed(80);
    MotorAdeIzq->run(FORWARD);
    MotorAdeDer->setSpeed(80);
    MotorAdeDer->run(BACKWARD);
    */
}
void MotoresPuentes::moveIzqAcomodo(){
    uint8_t i=235;
//TODAVI NO SE SABEN LAS POSICIONES CORRECTAS.......
digitalWrite(motorIzqAde2, LOW);
analogWrite(motorIzqAde1, 80);
digitalWrite(motorIzqAtras1, LOW);
analogWrite(motorIzqAtras2, 80);
digitalWrite(motorDerAde1, LOW);
analogWrite(motorDerAde2, 80);
digitalWrite(motorDerAtras1, LOW);
analogWrite(motorDerAtras2, 80);
    /*
    MotorAtrasDer->setSpeed(80);
    MotorAtrasDer->run(BACKWARD);
    MotorAtrasIzq->setSpeed(80);
    MotorAtrasIzq->run(FORWARD);
    MotorAdeIzq->setSpeed(80);
    MotorAdeIzq->run(BACKWARD);
    MotorAdeDer->setSpeed(80);
    MotorAdeDer->run(FORWARD);
    */
}

void MotoresPuentes::acomodarseChoqueIzquierda1()
{
    digitalWrite(motorIzqAde2, 200);
    analogWrite(motorIzqAde1, LOW);
    digitalWrite(motorIzqAtras1, 200);
    analogWrite(motorIzqAtras2, LOW);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2, LOW);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, LOW);

    delay(350);
}

void MotoresPuentes::acomodarseChoqueIzquierda2()
{
    digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, LOW);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, LOW);
    digitalWrite(motorDerAde1, 200);
    analogWrite(motorDerAde2, LOW);
    digitalWrite(motorDerAtras1, 200);
    analogWrite(motorDerAtras2, LOW);

    delay(300);
}

void MotoresPuentes::acomodarseChoqueDerecha1()
{
    digitalWrite(motorIzqAde2, LOW);
    analogWrite(motorIzqAde1, LOW);
    digitalWrite(motorIzqAtras1, LOW);
    analogWrite(motorIzqAtras2, LOW);
    digitalWrite(motorDerAde1, 200);
    analogWrite(motorDerAde2 , LOW);
    digitalWrite(motorDerAtras1, 200);
    analogWrite(motorDerAtras2, LOW);

    delay(350);
}
void MotoresPuentes::acomodarseChoqueDerecha2()
{
    digitalWrite(motorIzqAde2, 200);
    analogWrite(motorIzqAde1, LOW);
    digitalWrite(motorIzqAtras1, 200);
    analogWrite(motorIzqAtras2, LOW);
    digitalWrite(motorDerAde1, LOW);
    analogWrite(motorDerAde2 , LOW);
    digitalWrite(motorDerAtras1, LOW);
    analogWrite(motorDerAtras2, LOW);

    delay(350);
}
