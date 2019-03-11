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
boolean sDebug, Error, writeBack, outputEverything;

char      msg [100]; 
char  getBuff [100]; 
unsigned int getBuffPos;
int code;

float time1, time2;
int encCount, motCount, motCount2, talkcount;
void setup() {
  
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

  Error = outputEverything = false;
  sDebug = true;
  writeBack = true;
  
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial port to connect
  
  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);

  getBuffPos = 0;
  code = 0;
  time1 = 0;
  talkcount = 0;
}

void loop() {
  if(Serial.available()>0){
    char c = Serial.read();
    getBuff[getBuffPos++] = c;
    if(getBuffPos == 2 && code == 0){ // reset code to 0 between msgs
      code = (getBuff[1] << 8) | getBuff[0];
      int test = -1;
      Serial.print("test is ");
      Serial.print(test);
      Serial.print("code is ");
      Serial.print(code);
      getBuffPos = 0;
      code = 0;
    }
  }
  else if(millis() - time1 > 1000){
    time1 = millis();
    Serial.print("talking");
    Serial.println(talkcount++);
  }
  if(Error){
    sprintf(msg, "lPWM = %d rPWM = %d --- lPWM2 = %d rPWM2 = %d\n",
    lPWM,rPWM,lPWM2,rPWM2);
    Serial.print(msg);
    Error = false;
  }
  
  if(outputEverything){
    sprintf(msg, "lPWM = %d rPWM = %d and lPWM2 = %d rPWM2 = %d\n",
    lPWM,rPWM,lPWM2,rPWM2);
    Serial.println(msg);
    sprintf(msg, "motCount = %d, motCount2 = %d", motCount, motCount2);
    Serial.println(msg);
    outputEverything = false;
  }
  
  if(zeroFlag){
      digitalWrite(LED_BUILTIN, HIGH);
      zeroFlag = false;
  }
  if(millis()-zeroStamp > 500){      
      digitalWrite(LED_BUILTIN, LOW);
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
