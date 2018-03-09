#include <PulsePosition.h>
#include <Servo.h>

#define pixCH1 1
#define pixCH3 2
#define leftmotorpin 3
#define rightmotorpin 4
#define rcppm 5

#define frontIR A0
#define leftIR A1
#define rightIR A2

#define IRthreshold 400

PulsePositionInput rcInput;

Servo leftmotor;
Servo rightmotor;

const bool uselidar = true;

int CH1reading = 1500;
int CH2reading = 1500;
int CH3reading = 1000;

int pixCH1reading = 1500;
int pixCH3reading = 1500;

int leftspeed = 1500;
int rightspeed = 1500;

long pixCH1start = 0;
long pixCH3start = 0;

String buff = "";
String last_received = "";
int isObstacle = false;
int lidarleft = 1500;
int lidarright = 1500;

int irfront = 250;
int irleft = 250;
int irright = 250;

void rcInputUpdate() {
  CH1reading = rcInput.read(1);
  CH2reading = rcInput.read(3);
  CH3reading = rcInput.read(7);
}

void interruptinit() {
  attachInterrupt(pixCH1, pixCH1rising, RISING);
  attachInterrupt(pixCH3, pixCH3rising, RISING);
}

void pixCH1rising(){
  pixCH1start = micros();
  detachInterrupt(digitalPinToInterrupt(pixCH1));
  attachInterrupt(digitalPinToInterrupt(pixCH1),pixCH1falling,FALLING);
}

void pixCH1falling(){
  pixCH1reading = micros() - pixCH1start;
  if(pixCH1reading < 900 || pixCH1reading > 2100){
    pixCH1reading = 1500;
  }
  detachInterrupt(digitalPinToInterrupt(pixCH1));
  attachInterrupt(digitalPinToInterrupt(pixCH1),pixCH1rising,RISING);
}

void pixCH3rising(){
  pixCH3start = micros();
  detachInterrupt(digitalPinToInterrupt(pixCH3));
  attachInterrupt(digitalPinToInterrupt(pixCH3),pixCH3falling,FALLING);
}

void pixCH3falling(){
  pixCH3reading = micros() - pixCH3start;
  if(pixCH3reading < 900 || pixCH3reading > 2100){
    pixCH3reading = 1500;
  }
  detachInterrupt(digitalPinToInterrupt(pixCH3));
  attachInterrupt(digitalPinToInterrupt(pixCH3),pixCH3rising,RISING);
}

bool manualswitch()
{
  return (CH3reading <= 1500);
}

void rx(){
  while(Serial.available()){
    buff += Serial.readString();
  }
  int lastnindex = buff.lastIndexOf(',');
  if (lastnindex > 0){
    last_received = buff.substring(0, lastnindex);
    buff = buff.substring(lastnindex+1);
    Serial.println(last_received);
    int spaceindex = last_received.indexOf(' ');
    isObstacle = (last_received.substring(0,spaceindex)).toInt();
    last_received = last_received.substring(spaceindex+1);
    spaceindex = last_received.indexOf(' ');
    lidarleft = (last_received.substring(0, spaceindex)).toInt();
    last_received = last_received.substring(spaceindex+1);
    lidarright = (last_received).toInt();
  }
}

void ircheck()
{
  irfront = analogRead(frontIR);
  irleft = analogRead(leftIR);
  irright = analogRead(rightIR);
  if(irfront > IRthreshold){
    leftmotor.writeMicroseconds(1000);
    rightmotor.writeMicroseconds(2000);
    delay(500);
  }else if(irleft > IRthreshold){
    leftmotor.writeMicroseconds(2000);
    rightmotor.writeMicroseconds(1400);
  }else if(irright > IRthreshold){
    leftmotor.writeMicroseconds(1400);
    rightmotor.writeMicroseconds(2000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rcInput.begin(rcppm);
  interruptinit();
  leftmotor.attach(leftmotorpin);
  rightmotor.attach(rightmotorpin);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  rcInputUpdate();
  rx();
  if(isObstacle)
  {
    digitalWrite(13, HIGH);
  }else{
    digitalWrite(13, LOW);
  }
  if(manualswitch())
  {
    int CH2speed = CH2reading - 1500;
    int CH1turn = CH1reading - 1500;
    int leftspeed = CH2speed + CH1turn + 1500;
    int rightspeed = CH2speed - CH1turn + 1500;
    if (leftspeed > 2000){
      leftspeed = 2000;
    }
    if (leftspeed < 1000){
      leftspeed = 1000;
    }
    leftmotor.writeMicroseconds(leftspeed);
    if (rightspeed > 2000){
      rightspeed = 2000;
    }
    if (rightspeed < 1000){
      rightspeed = 1000;
    }
    rightmotor.writeMicroseconds(rightspeed);
  }else{
    if(uselidar){
      if(isObstacle){
        leftmotor.writeMicroseconds(lidarleft);
        rightmotor.writeMicroseconds(lidarright);
      }else{
        leftmotor.writeMicroseconds(pixCH1reading);
        rightmotor.writeMicroseconds(pixCH3reading);
      }
    }else{
      if(isObstacle){
        ircheck();
      }else{
        leftmotor.writeMicroseconds(pixCH1reading);
        rightmotor.writeMicroseconds(pixCH3reading);
      }
    }  
  }
}
