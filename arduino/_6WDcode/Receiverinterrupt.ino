void interruptinit(){
  //attachInterrupt(digitalPinToInterrupt(CH1),CH1rising,RISING);
  attachInterrupt(digitalPinToInterrupt(CH2),CH2rising,RISING);
}

void CH1rising(){
  ch1start = micros();
  detachInterrupt(digitalPinToInterrupt(CH1));
  attachInterrupt(digitalPinToInterrupt(CH1),CH1falling,FALLING);
}

void CH1falling(){
  CH1reading = micros() - ch1start;
  if(CH1reading < 900 || CH1reading > 2100){
    CH1reading = 1500;
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
  CH2reading = micros() - ch2start;
  if(CH2reading < 900 || CH2reading > 2100){
    CH2reading = 1500;
  }
  detachInterrupt(digitalPinToInterrupt(CH2));
  attachInterrupt(digitalPinToInterrupt(CH2),CH2rising,RISING);
}

void CH1read(){
  CH1reading = pulseIn(CH1,HIGH);
  if(CH1reading > 2100 || CH1reading < 900){
    CH1reading = 1500;
  }
}

int CH1speed(){
  return (CH1reading - 1500);
}

int CH2speed(){
  return (CH2reading - 1500);
}

int leftRCspeed(){
  int CH2speed = CH2reading - 1500;
  int CH1turn = CH1reading - 1500;
  int lspeed = CH2speed + CH1turn;
  if(lspeed >500){
    lspeed = 500;
  }
  if(lspeed <-500){
    lspeed = -500;
  }
  return lspeed;
}

int rightRCspeed(){
  int CH2speed = CH2reading - 1500;
  int CH1turn = CH1reading - 1500;
  int rspeed = CH2speed - CH1turn;
  if(rspeed >500){
    rspeed = 500;
  }
  if(rspeed <-500){
    rspeed = -500;
  }
  return rspeed;
}

