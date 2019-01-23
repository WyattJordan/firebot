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

char sendbuff [100];
char  getBuff [100];
float time2;
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
  getBuff[1] = Wire.read();
}

void sendData(){
  leftDuration = 0;
  rightDuration = 0;
  time2 = millis();
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.write(getBuff[0]); //leftDuration
  Wire.write(getBuff[1]); //rightDuration
  
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
