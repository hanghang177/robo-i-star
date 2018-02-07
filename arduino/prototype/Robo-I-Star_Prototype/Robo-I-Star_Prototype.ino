#include <Servo.h>

const int MotorL =5, MotorR = 6, MDL = 4 ,MDR = 7;
//MDL,MDR are motor direction left and motor direction right, when LOW motors go backwards, when HIGH motors go forwards
//MotorL, MotorR, are motor left and right outputs (PWM), they control speed of motors

Servo FrontR, FrontL, BackR, BackL;
// Servo instants

const int CH1 = 2;
const int CH2 = 3;
// Channel 1 and 2 pins

long ch1start = 0;
long ch2start = 0;
// Interrupt timer for recording PWM width

int dir = 1500, fbspeed = 1500;
//direction and forwardbackward speed inputs are between 1000,2000 pwm



void setup() {
  // put your setup code here, to run once:
  pinMode(MotorL,OUTPUT);
  pinMode(MotorR,OUTPUT);
  pinMode(MDL,OUTPUT);
  pinMode(MDR,OUTPUT);
  
  FrontL.attach(10);
  FrontR.attach(11);
  BackL.attach(12);
  BackR.attach(13);
  
  attachInterrupt(digitalPinToInterrupt(CH1),CH1rising,RISING);
  attachInterrupt(digitalPinToInterrupt(CH2),CH2rising,RISING);
  
  Serial.begin(115200);//NEED TO FILL OUT THIS VALUE 
}

void loop() {
  // put your main code here, to run repeatedly:
  /*Can be better written using map() function*/
  if(fbspeed<1500){
    backward((1500-fbspeed)/2);//the motor pins take in speeds between 0-255 so our pwm input is scaled down to between 0-250
  }else{
    forward((fbspeed-1500)/2);
  }

  //this makes all the wheels turn at the same time
  FrontL.writeMicroseconds(dir);
  FrontR.writeMicroseconds(dir);
  BackR.writeMicroseconds(3000-dir); //if you comment out these BackR,BackL lines then just the front two wheels will turn
  BackL.writeMicroseconds(3000-dir);
}

void forward(int a){
  digitalWrite(MDL,HIGH);
  digitalWrite(MDR,HIGH);
  analogWrite(MotorL,a);
  analogWrite(MotorR,a);
}

void backward(int a){
  digitalWrite(MDL,LOW);
  digitalWrite(MDR,LOW);
  analogWrite(MotorL,a);
  analogWrite(MotorR,a);
}

void CH1rising(){
  ch1start = micros();
  detachInterrupt(digitalPinToInterrupt(CH1));
  attachInterrupt(digitalPinToInterrupt(CH1),CH1falling,FALLING);
}

void CH1falling(){
  dir = micros() - ch1start;
  if(dir < 900 || dir > 2100){
    dir = 1500;
  }
  detachInterrupt(digitalPinToInterrupt(CH1));
  attachInterrupt(digitalPinToInterrupt(CH1),CH1rising,RISING);
}

void CH2rising(){
  ch2start = micros();
  detachInterrupt(digitalPinToInterrupt(CH2));
  attachInterrupt(digitalPinToInterrupt(CH2),CH2falling,FALLING);
}

void CH2falling(){
  fbspeed = micros() - ch2start;
  if(fbspeed < 900 || fbspeed > 2100){
    fbspeed = 1500;
  }
  detachInterrupt(digitalPinToInterrupt(CH2));
  attachInterrupt(digitalPinToInterrupt(CH2),CH2rising,RISING);
}

