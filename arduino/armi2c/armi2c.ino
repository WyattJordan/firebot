#include <Arduino.h>
#include <Wire.h>

//unsigned char D3, D6, D9, D10, D11;
int PWMpins[5]= {3,     6,   9,  10,  11};
int PWMdefs[5]= {127, 127, 127,  10, 245};
int armContactCount;
char sendbuff [100];
char  getBuff [100];
float time2;
void setup() {
  // I2C
  Wire.begin(16); 
  Wire.onReceive(getData);
  Wire.onRequest(sendData);
  
  // servos
  armContactCount = -1;
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  // Start at default positions (no Odroid signal!)
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<5; i++){
    //analogWrite(PWMpins[i], PWMdefs[i]);
    analogWrite(PWMpins[i], 50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if(millis() - time2 > 500){
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void getData(int num){
  time2 = millis();
  /*getBuff[0] = Wire.read();
  if(getBuff[0] == 'a') {armContactCount = 6;}
  if(armContactCount>0 && armContactCount<6){
    //analogWrite(PWMpins[5-armContactCount], getBuff[0]);
    armContactCount--;
  }*/
}

void sendData(){
  //if(getBuff[0] == 'a') {;}
  // list other cases for codes here
  
  time2 = millis();
  digitalWrite(LED_BUILTIN, HIGH);
}
