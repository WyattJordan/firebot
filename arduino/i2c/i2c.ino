#include <Arduino.h>
#include <Wire.h>
int pos;
int s;
char buff [1000];

void setup(){
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  delay(200);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  
  Wire.begin(17); 
  Wire.onReceive(getData);
  Wire.onRequest(sendData);
  pos = 0;
  s = 0;
  delay(100);
  Serial.begin(9600);
  while(!Serial){;}
  //Serial.println("I'm ready\n");
}

void loop(){
  digitalWrite(3,HIGH);
  //Wire.write("hello odroid"); // send data back to master
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
}

void getData(int num){
  digitalWrite(5, HIGH);
  buff[pos++] = Wire.read();
  buff[pos++] = Wire.read();
}
 void sendData(){ // send data back
  digitalWrite(4,HIGH);
  Wire.write(buff[s++]);
  Wire.write(buff[s++]);
 } 
