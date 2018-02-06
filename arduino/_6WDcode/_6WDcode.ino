
#define LmotorA             3  // Left  motor H bridge, input A
#define LmotorB            11  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

#define Battery             A0  // Analog input 00
#define RmotorC             A6  // Analog input 06
#define LmotorC             A7  // Analog input 07
#define Charger            13  // Low=ON High=OFF

#define Maxamps            800     // set overload current for left motor 
#define Overloadtime       100     // time in mS before motor is re-enabled after overload occurs

#define CH2                2    //Channel 2 (Speed) on receiver
#define CH1                4    //Channel 1 (Turn) on receiver

int leftspeed = 0;              //Speed of left motor
int rightspeed = 0;             //Speed of right motor

long leftover = 0;              //The timeout counter after left motor overheats
long rightover = 0;             //The timeout counter after right motor overheats

int CH1reading = 1500;          //The PWM reading from CH1 on the receiver
int CH2reading = 1500;          //The PWM reading from CH2 on the receiver

long ch1start = 0;              //Used for counting PWM pulse width of CH1
long ch2start = 0;              //Used for counting PWM pulse width of CH2

void readRC(){
  //leftspeed = leftRCspeed()*255.0/500.0;    //Mapping the PWM -500 - 500 raeding to -255 to 255 reading
  //rightspeed = rightRCspeed()*255.0/500.0;  //Same, but for the right motor
  leftspeed = CH1speed();
  rightspeed = CH2speed();
  if(abs(leftspeed) < 100){
    leftspeed = 0;
  }else if (leftspeed >= 100){
    leftspeed = map(leftspeed, 100, 500, 300, 500);
  }else{
    leftspeed = map(leftspeed, -100, -500, -300, -500);
  }
  if(abs(rightspeed) < 100){
    rightspeed = 0;
  }else if (rightspeed >= 100){
    rightspeed = map(rightspeed, 100, 500, 300, 500);
  }else{
    rightspeed = map(rightspeed, -100, -500, -300, -500);
  }
  leftspeed = leftspeed*255.0/500.0;
  rightspeed = rightspeed*255.0/500.0;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           //Begin the debugging serial
  interruptinit();              //PWM interrupt read initialization
}

void loop() {
  // put your main code here, to run repeatedly:
  CH1read();                    //Read the CH1 reading through pulseIn because it's not as important as CH2 and there is only one available interrupt pin left
  readRC();                     //Read the receiver reading (CH2)
  overloadcheck();              //Check if the motor is overloading
  go();                         //Run the motors using the leftspeed and rightspeed
}
