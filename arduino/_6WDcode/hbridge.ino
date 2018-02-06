void go(){
  int absleft = abs(leftspeed);
  int absright = abs(rightspeed);
  if(leftspeed > 0){
    analogWrite(LmotorB, 0);
    analogWrite(LmotorA, absleft);
  }else if(leftspeed <0){
    analogWrite(LmotorA, 0);
    analogWrite(LmotorB, absleft);
  }else{
    analogWrite(LmotorA, 0);
    analogWrite(LmotorB, 0);
  }
  if(rightspeed < 0){
    analogWrite(RmotorB, 0);
    analogWrite(RmotorA, absright);
  }else if(rightspeed > 0){
    analogWrite(RmotorA, 0);
    analogWrite(RmotorB, absright);
  }else{
    analogWrite(RmotorA, 0);
    analogWrite(RmotorB, 0);
  }
}

void goforward(){
  leftspeed = 200;
  rightspeed = 200;
}

void stop(){
  leftspeed = 0;
  rightspeed = 0;
}

void turnright(){
  leftspeed = 255;
  rightspeed = 100;
}

void turnleft(){
  leftspeed = 100;
  rightspeed = 255;
}

void overloadcheck(){
  int leftamp = analogRead(LmotorC);
  int rightamp = analogRead(RmotorC);
  if(leftamp > Maxamps){
    leftover = millis();
  }
  if(rightamp > Maxamps){
    rightover = millis();
  }
  if(millis() - leftover < Overloadtime){
    leftspeed = 0;
  }
  if(millis() - rightover < Overloadtime){
    rightspeed = 0;
  }
}
