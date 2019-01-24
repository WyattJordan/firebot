#include <Arduino.h>
#include <Wire.h>

char sendbuff [100];
char  getBuff [100];
float time2;
void setup() {
  // I2C
  Wire.begin(16); 
  Wire.onReceive(getData);
  Wire.onRequest(sendData);
  // encoders
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<4; i++){
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
  time2 = millis();
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.write('m'); 
  Wire.write('n'); 
  
}

