#include <Arduino.h>
//#include <SPI.h>

char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup() {
  // put your setup code here, to run once:
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

  Serial.begin(9600);
  while (!Serial) {;}

  pinMode(10, INPUT); //pinMode(SS, INPUT);
  pinMode(11, INPUT); //pinMode(MOSI, INPUT);
  pinMode(13, INPUT); //pinMode(SCK, INPUT);
  pinMode(12, OUTPUT); //pinMode(MISO, OUTPUT);
  for(int i=0; i<100; i++) buf[i] = '!';
  DDRB  = 0x00;
  PORTB = 0x00;
  //SREG |= 0b10000000; // set global interrupts
  //SPCR = 0xC0; // slave mode, enable SPI interrupt, mode0
  //SPCR |= bit (SPE); // turn on slave SPI mode
  //SPCR = (1<<SPE); // use slave mode
 // SPI.setDataMode(SPI_MODE0); // clk normally low,

 // turn on SPI in slave mode
 SPCR |= _BV(SPE);
 
 // turn on interrupts
 SPCR |= _BV(SPIE);
 
  pos = 0;
  process_it = false;

  //SPI.attachInterrupt(); // THIS IS REQUIRED
}

ISR(SPI_STC_vect){
  // add a delay here? or a while loop checking the SPIF flag???
  digitalWrite(4, HIGH);
  while(!(SPSR & (1<<SPIF)));
  digitalWrite(5, HIGH);
  char c = (char) SPDR;
  // add to buffer if room
  if (pos < (sizeof (buf) - 1))
    buf [pos++] = c;

    process_it = true;
}

void loop() {
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(3, LOW);
  digitalWrite(5, LOW);

if (process_it) {
      unsigned long tmp = millis();
      while(millis() - tmp < 500){;}
      for(int i=0; i<pos; i++){
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
      digitalWrite(4, LOW);

      buf [pos] = 0;
      pos = 0;
      process_it = false;
  }  // end of flag set

  if(Serial.available() > 0){
    char b = Serial.read();
    Serial.print("received a: ");
    Serial.println(b);
    Serial.print("SPDR is: ");
    Serial.println((char) SPDR);
    digitalWrite(5, HIGH);
    delay(50);
    digitalWrite(5, LOW);
  }
};
 
