#define BalDeg1 4
#define BalDeg2 160
#define mLeft  0
#define mRight 1
#define mSlow  750
#define mFast  750
#define MaxMoveLength 1000
#define ResetMoveLength 500
#define RailLength  1200
#define MoveLengthIncrement 10


//PIN Defination
const int pinA = 2;
const int pinB = 3; 
const int stepPin = 6; 
const int dirPin = 7; 
const int enPin = 8;

//Encoder Variables
volatile int lastEncoded = 0;
volatile int encoderValue = 0;
int eValue = 0;
int eValueA = 0;

long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

//Pendulum Variables
boolean PDir = false;
boolean SDir = false;

int MovPos = 0;
int MovLen = ResetMoveLength;
int AbsPos = 0;
int skipCnt = 0;

void setup(){ 
  Serial.begin (9600);

  //Motor Controller Pins
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,HIGH);

  //Encoder Pins
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP);

  //call updateEncoder() when any  Change (high/low) occurs on interrupt 0(pin 2), or interrupt 1(pin 3) 
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(pinB), updateEncoder, CHANGE);
 
   Serial.println("BEGIN");
} 

void updateEncoder(){
  int MSB = digitalRead(pinA); //MSB = most significant bit
  int LSB = digitalRead(pinB); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
    PDir = true;
    encoderValue ++;
    if(encoderValue > 1440){
       encoderValue = encoderValue - 1440; 
    }
  }else if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000){
    PDir = false;
    encoderValue --;
    if(encoderValue < -1440){
       encoderValue = encoderValue + 1440; 
    }
  }
  lastEncoded = encoded; //store this value for next time
}

inline static void MoveMotor(int Dir, int Delay){
  digitalWrite(dirPin, Dir);
  digitalWrite(enPin, LOW);
  
  digitalWrite(stepPin,HIGH); 
  delayMicroseconds(1); 
  
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(Delay); 

//  digitalWrite(enPin,HIGH);
  if(Dir){
    AbsPos++;
  }else{
    AbsPos--;
  }
}

inline static void MoveToCenter(){
  if(AbsPos < 0){
    SDir = true;
  }else{
    SDir = false;
  }
  while(AbsPos != 0){
    MoveMotor(SDir, 1000);
  }
  MovPos = 0;
  MovLen = ResetMoveLength;
}

void loop() { 
  eValue = encoderValue;
  eValueA = abs(eValue);
  if(abs(AbsPos) > RailLength){
    MoveToCenter();
  }else if((eValueA > (720 - BalDeg1)) && (eValueA < (720 + BalDeg1))){ //Pendulum is in Balanced Zone i.e. 1~2 degree variation from upright position. Don't Move Motor
    skipCnt = 500;
  }else if((eValueA > (720 - BalDeg2)) && (eValueA < (720 + BalDeg2))){ //Pendulum is in Balancing Zone i.e. 10 degree varation
    if(eValue < -720){
      MoveMotor(mLeft, mFast);
    }else if(eValue < -720 + BalDeg2){
      MoveMotor(mRight, mFast);
    }else if(eValue < 720){
      MoveMotor(mLeft, mFast);
    }else if(eValue < 720 + BalDeg2){
      MoveMotor(mRight, mFast);
    }
    skipCnt = 500;
  }else{                                                              //Pendulum not in Balancing Zone. Swing the pendulum to get in balancing Zone
    if(MovPos < MovLen){                                              //Keep moving Motor in present Swing direction
      MovPos++;
    }else{                      //Change direction and (Reset to High frequency by Resetting MoveLenght OR Decrease the frequency by increasing MoveLenght)
      SDir = !SDir;
      if(MovPos > MaxMoveLength){                                     //Restart to High frequency Movement of pendulum
        MoveToCenter();
      }else{
        delay(5);
        MovPos = 0;
        MovLen = MovLen + MoveLengthIncrement;   //Decrease the frequency by Increasing the Movement Lenght to get the pendulum upright
      }  
    }
    if(skipCnt == 0){
      MoveMotor(SDir, mSlow);
    }else{      //Wait for the pendulum to be stable
      skipCnt--;
      delay(1);
    }
  }
}
