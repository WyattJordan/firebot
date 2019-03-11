#include <Arduino.h>
#include <Wire.h>

const byte leftpinA = 2;//A pin -> the interrupt pin 2
const byte leftpinB = 4;//B pin -> the digital pin 4

const byte rightpinA = 3;//A pin -> the interrupt pin 3
const byte rightpinB = 5;//B pin -> the digital pin 5

const byte leftDirPin  = 6; 
const byte rightDirPin = 7;
const byte leftDrivePin  = 10;
const byte rightDrivePin = 11;

byte leftPinALast;
int leftDuration;//the number of the pulses
boolean leftDirection;//the rotation direction

byte rightPinALast;
int rightDuration;//the number of the pulses
boolean rightDirection;//the rotation direction

unsigned char lPWM, rPWM, lPWM2, rPWM2;
boolean lforward, rforward, lforward2, rforward2, zeroFlag;
long zeroStamp;
boolean sDebug, Error, writeBack, readMotors, secondMsg;

char sendbuff [40];
char  getBuff [40];
char  msg [40];

float time2;
int encCount, motCount, motCount2;
void setup() {
  // I2C
  Wire.begin(17); 
  Wire.onReceive(getData);
  Wire.onRequest(sendData);
  
  // encoders
  pinMode(LED_BUILTIN, OUTPUT);
  leftDirection = true; //default -> Forward
  rightDirection = true;//default -> Forward
  pinMode( leftpinB,INPUT);
  pinMode(rightpinB,INPUT);
  attachInterrupt(0, leftwheelSpeed, CHANGE);//int.0 (pin2)
  attachInterrupt(1, rightwheelSpeed,CHANGE);//int.1 (pin3)
  encCount = motCount = -1; // counts down to 0
  zeroFlag = false;
  zeroStamp = millis();
  
  // drive
  lPWM = rPWM = 0; // stopped
  lforward = rforward = true;
  pinMode(leftDrivePin, OUTPUT);
  pinMode(rightDrivePin, OUTPUT);
  pinMode(rightDirPin,OUTPUT);
  pinMode( leftDirPin,OUTPUT);
  analogWrite(leftDrivePin, lPWM);
  analogWrite(rightDrivePin, rPWM);
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, HIGH);  

  Error = false;
  sDebug = true;
  writeBack = true;
  readMotors = secondMsg = false;
  if(sDebug) Serial.begin(115200);
  
  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if(Error){
    sprintf(msg, "lPWM = %d rPWM = %d --- lPWM2 = %d rPWM2 = %d\n",
    lPWM,rPWM,lPWM2,rPWM2);
    Serial.print(msg);
    Error = false;
  }
  
  if(zeroFlag){
      digitalWrite(LED_BUILTIN, HIGH);
      zeroFlag = false;
  }
  if(millis()-zeroStamp > 500){      
      digitalWrite(LED_BUILTIN, LOW);
  }
}

void getData(int num){
  int i=0;
  while(Wire.available()){
    getBuff[i++] = Wire.read();
  }
  if(getBuff[0] == '1'){ 
    encCount = 4; // starts 4 byte enc interaction
  } 
  else{ 
    readMotors = true;
    secondMsg = (getBuff[0] == 'w' || getBuff[0] == 'x') ? true : false;
  }
    
  // leftmotor is a/b and right is c/d (first char is forward)
  if(readMotors){
    if(!secondMsg){
      lforward = getBuff[0] == 'a' ? true : false;
      lPWM = getBuff[1];
      rforward = getBuff[2] == 'c' ? true : false;
      rPWM = getBuff[3];
    }
    else{
      lforward2 = getBuff[0] == 'w' ? true : false;
      lPWM2 = getBuff[1];
      rforward2 = getBuff[2] == 'y' ? true : false;
      rPWM2 = getBuff[3];
      
      if(lPWM == lPWM2 && (lforward == lforward2)){
        if(writeBack) Wire.write(lPWM2); // send back for verification
        analogWrite(leftDrivePin, lPWM);
        digitalWrite(leftDirPin, lforward ? HIGH : LOW);
        if(sDebug) {Serial.println("changed left");}
      }
      
      if(rPWM == rPWM2 && (rforward == rforward2)){
        if(writeBack) Wire.write(rPWM2); // send back for verification
        analogWrite(rightDrivePin, rPWM);
        digitalWrite(rightDirPin, rforward ? HIGH : LOW);
        if(sDebug) {Serial.println("changed right");}
      }
    }
  }
} // end of getData

void sendData(){
  if(readMotors){
    readMotors = false;
    Wire.write(getBuff, 4);
  }
  if(encCount > 0){
    // incoming bytes should be ['1','2','3','4'], send back enc vals
    if(encCount == 4){
      Wire.write(highByte(leftDuration));
    }
    else if(encCount == 3){
      Wire.write(lowByte(leftDuration));
      leftDuration = 0;
    }
    else if(encCount == 2){
      Wire.write(highByte(rightDuration));    
    }
    else if(encCount == 1){
      Wire.write(lowByte(rightDuration));
      rightDuration = 0;
    }
    encCount--;
    time2 = millis();
  }
}

////////////////////////////////////////////////////////////////
///////////////////Encoder Interrupts///////////////////////////
////////////////////////////////////////////////////////////////
void leftwheelSpeed()
{
  int Lstate = digitalRead(leftpinA);
  if((leftPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(leftpinB);
    if(val == LOW && leftDirection)
    {
      leftDirection = false; //Reverse
    }
    else if(val == HIGH && !leftDirection)
    {
      leftDirection = true;  //Forward
    }
  }
  leftPinALast = Lstate;

  if(!leftDirection)  leftDuration++;
  else  leftDuration--;
}

void rightwheelSpeed()
{
  int Rstate = digitalRead(rightpinA);
  if((rightPinALast == LOW) && Rstate==HIGH)
  {
    int val = digitalRead(rightpinB);
    if(val == LOW && rightDirection)
    {
      rightDirection = false; //Reverse
    }
    else if(val == HIGH && !rightDirection)
    {
      rightDirection = true;  //Forward
    }
  }
  rightPinALast = Rstate;

  if(!rightDirection)  rightDuration++;
  else  rightDuration--;
}
