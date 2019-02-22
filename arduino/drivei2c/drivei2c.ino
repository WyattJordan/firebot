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
boolean sDebug;

char sendbuff [100];
char  getBuff [100];
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

  sDebug = true;
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
  if(0 &&sDebug){
        Serial.print("lPWM = ");
        Serial.print(lPWM);
        Serial.print(" rPWM = ");
        Serial.print(rPWM);
         Serial.print(" lPWM2 = ");
        Serial.print(lPWM2);
        Serial.print(" rPWM2 = ");
        Serial.println(rPWM2);
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
  getBuff[0] = Wire.read();
  if(getBuff[0] == '1'){ encCount = 4;} // starts 4 byte enc interaction
  else if(getBuff[0] == 'a' || getBuff[0] == 'b'){ motCount = 4;} // starts 3 byte motor interaction
  else if(getBuff[0] == 'w' || getBuff[0] == 'x'){ motCount2 = 4;}
  
  // leftmotor is a/b and right is c/d (first char is forrward)
  if(motCount == 4) lforward = getBuff[0] == 'a' ? true : false;
  if(motCount == 3) lPWM = getBuff[0];
  if(motCount == 2) rforward = getBuff[0] == 'c' ? true : false;
  if(motCount == 1) rPWM = getBuff[0];

  // leftmotor is w/x and right is y/z (first char is forrward)
  if(motCount2 == 4) lforward2 = getBuff[0] == 'w' ? true : false;
  if(motCount2 == 3) lPWM2 = getBuff[0];
  if(motCount2 == 2) rforward2 = getBuff[0] == 'y' ? true : false;
  if(motCount2 == 1) rPWM2 = getBuff[0];

  
  if(rPWM == 0 || lPWM == 0 || lPWM==50 || rPWM==52) {
    zeroFlag = true; zeroStamp = millis();}
} // end of getData

void sendData(){
  
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
  // first motor interaction incoming bytes should be ['a/b',lPWM,'c/d',rPWM]
  else if(motCount>0){
    if(motCount == 4){
        Wire.write(lforward ? 'a' : 'b');
    }
    if(motCount == 3){
      Wire.write(lPWM); // send back for verification
    }
    if(motCount == 2){
      Wire.write(rforward ? 'c' : 'd');
    }
    if(motCount == 1){
      Wire.write(rPWM); // send back for verification
    }
    motCount--;    
  }
  // second motor interaction to verify, only set PWMs if both agree
  // incoming bytes should be ['w/x',lPWM,'y/z',rPWM]
  else if(motCount2>0){
    if(motCount2 == 4){
        Wire.write(lforward2 ? 'w' : 'x');
    }
    if(motCount2 == 3){
      if(lPWM == lPWM2 && (lforward == lforward2)){
        Wire.write(lPWM2); // send back for verification
        analogWrite(leftDrivePin, lPWM);
        digitalWrite(leftDirPin, lforward ? HIGH : LOW);
      }
      else{
        Wire.write('q'); // the two messages don't match
      }
      
    }
    if(motCount2 == 2){
      Wire.write(rforward2 ? 'y' : 'z');
    }
    if(motCount2 == 1){
      if(rPWM == rPWM2 && (rforward == rforward2)){
        Wire.write(rPWM2); // send back for verification
        analogWrite(rightDrivePin, rPWM);
        digitalWrite(rightDirPin, rforward ? HIGH : LOW);
      }
      else{
        Wire.write('r');
        if(sDebug) {Serial.println("Failed to receive two correctly");}
      }
     
    }
    motCount2--;   
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
