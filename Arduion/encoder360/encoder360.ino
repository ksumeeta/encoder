/**************
 * VCC to 5V  *
 * GND to GND *
 * CLK to D3  *
 * CLK to D4  *
 **************/
const int pinA = 2;
const int pinB = 3; 

const int pinLend = 4;
const int pinRend = 5;

const int stepPin = 6; 
const int dirPin = 7; 
const int enPin = 8;

volatile int lastEncoded = 0;
volatile int encoderValue = 0;
int eValue = 0;
int eValueA = 0;

long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

boolean PDir = false;
boolean SDir = false;
boolean LeftEnd = false;
boolean RightEnd = false;

#define BalDeg1 8
#define BalDeg2 40
#define mLeft  1
#define mRight 0
#define mSlow  500
#define mFast  100
#define MaxMoveLength 1000
#define ResetMoveLength 200
#define MoveLengthIncrement 50

int MovPos = 0;
int MovLen = ResetMoveLength;

void setup(){ 
  Serial.begin (9600);

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,HIGH);
  
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinLend, INPUT); 
  pinMode(pinRend, INPUT);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(pinB), updateEncoder, CHANGE);
 
   Serial.println("BEGIN");

   //ToDo Move to Center
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

inline void MoveMotor(int Dir, int Delay){
  digitalWrite(dirPin, Dir);
  digitalWrite(enPin, LOW);
  
  digitalWrite(stepPin,HIGH); 
  delayMicroseconds(Delay); 
  
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(Delay); 

  digitalWrite(enPin,HIGH);
}

void loop() { 
  eValueA = abs(encoderValue);
  if((eValueA > (720 - BalDeg1)) && (eValueA < (720 + BalDeg1))){       //Pendulum is in Balanced Zone i.e. 1~2 degree variation from upright position. Don't Move Motor
    MovPos = 0;
    MovLen = ResetMoveLength;
  }else if((eValueA > (720 - BalDeg2)) && (eValueA < (720 + BalDeg2))){ //Pendulum is in Balancing Zone i.e. 10 degree varation
    if(PDir){
      //ToCheck Which Direction and What Delay
      //Todo Check Left IR Sensor
      MoveMotor(mLeft, mSlow);
    }else {
      //ToCheck Which Direction and What Delay
      //Todo Check Right IR Sensor
      MoveMotor(mRight, mSlow);
    }
    MovPos = 0;
    MovLen = ResetMoveLength;
  }else{                        //Pendulum not in Balancing Zone. Swing the pendulum to get in balancing Zone
    if(MovPos < MovLen){        //Keep moving Motor in present Swing direction
      MovPos++;
    }else{                      //Change direction and (Reset to High frequency by Resetting MoveLenght OR Decrease the frequency by increasing MoveLenght)
      SDir = !SDir;
      if(MovPos > MaxMoveLength){                 //Restart to High frequency Movement of pendulum
        //ToDo Move to Center
        MovPos = 0;
        MovLen = ResetMoveLength;
      }else{
        MovPos = 0;
        MovLen = MovLen + MoveLengthIncrement;   //Decrease the frequency by Increasing the Movement Lenght to get the pendulum upright
      }  
    }
    MoveMotor(SDir, mSlow);
  }
}
