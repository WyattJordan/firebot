#include <Arduino.h>
#include <Wire.h>

const byte leftpinA = 2;//A pin -> the interrupt pin 2
const byte leftpinB = 4;//B pin -> the digital pin 4
byte leftPinALast;
int leftDuration;//the number of the pulses
boolean leftDirection;//the rotation direction

const byte rightpinA = 3;//A pin -> the interrupt pin 3
const byte rightpinB = 5;//B pin -> the digital pin 5
byte rightPinALast;
int rightDuration;//the number of the pulses
boolean rightDirection;//the rotation direction

unsigned char lPWM, rPWM;
char sendbuff [100];
char  getBuff [100];
float time2;
int contactCount;
void setup() {
  // I2C
  Wire.begin(17); 
  Wire.onReceive(getData);
  Wire.onRequest(sendData);
  // encoders
  pinMode(LED_BUILTIN, OUTPUT);
  leftDirection = true;//default -> Forward
  rightDirection = true;//default -> Forward
  pinMode( leftpinB,INPUT);
  pinMode(rightpinB,INPUT);
  attachInterrupt(0, leftwheelSpeed, CHANGE);//int.0 (pin2)
  attachInterrupt(1, rightwheelSpeed,CHANGE);//int.1 (pin3)
  contactCount = -1; // counts down to 0
  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  /*if(millis() - time2 > 1000){
    digitalWrite(LED_BUILTIN, LOW);
  }*/
}

void getData(int num){
  getBuff[0] = Wire.read();
  if(getBuff[0] == "l"){ contactCount = 4;} // starts 4 byte interaction
  if(contactCount == 3) lPWM = getBuff[0];
  if(contactCount == 1) rPWM = getBuff[0];
}

void sendData(){
  if(contactCount > 0){
    // incoming bytes should be ['l',lPWM,'r',rPWM], send back enc vals
    if(contactCount == 4){
      Wire.write(highByte(leftDuration));
    }
    else if(contactCount == 3){
      Wire.write(lowByte(leftDuration));
      leftDuration = 0;
    }
    else if(contactCount == 2){
      Wire.write(highByte(righDuration));    
    }
    else if(contactCount == 1){
      Wire.write(lowByte(rightDuration));
      rightDuration = 0;
    }
    contactCount--;
    time2 = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

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
