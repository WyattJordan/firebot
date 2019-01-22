#include <Arduino.h>
#define TIMEOUT 3000

char usbbuffer [1000];
int position;
void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  delay(200);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  Serial.begin(230400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  digitalWrite(3, HIGH);
  position = 0;

  /*char inData[200];
  unsigned long time  out = millis() + TIMEOUT;
  uint8_t inIndex = 0;
  while ( ((int32_t)(millis() - timeout) < 0) && (inIndex < (sizeof(inData)/sizeof(inData[0])))) {
    delay(200);
    digitalWrite(5, HIGH);
    delay(200);
    digitalWrite(5, LOW);
      if (Serial.available() > 0) {
          inData[inIndex] = Serial.read();
          if ((inData[inIndex] == '\n') || (inData[inIndex]  == '\r')) {
              break;
          }
        Serial.write(inData[inIndex++]);
      }
  }*/
}
void loop() {
  /*while(1){
    delay(500);
    digitalWrite(3, HIGH);
    Serial.print('a');
    delay(500);
    Serial.print('b');
    digitalWrite(3, LOW);
    delay(500);
    Serial.print('c');
  }*/
  if(Serial.available() > 0){
    char b = Serial.read();
    usbbuffer[position] = b;
    position++;
  }
  if(position == 200){
    for(int i=0; i<position; i++){
      Serial.print(usbbuffer[i]);
    }
    //Serial.println();
    //Serial.println("end of transmission");
    digitalWrite(5, HIGH);
    delay(500);
    position = 0;
    digitalWrite(5, LOW);
  }

};
