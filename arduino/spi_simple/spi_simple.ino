#include "pins_arduino.h"
#include <SPI.h>
char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup (void)
{
  Serial.begin (9600);   // debugging
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(200);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
 
  pin Mode(10, INPUT); //pinMode(SS, INPUT);
  pinMode(11, INPUT); //pinMode(MOSI, INPUT);
  pinMode(13, INPUT); //pinMode(SCK, INPUT);
 // have to send on master in, *slave out*
 pinMode(MISO, OUTPUT);
 
 // turn on SPI in slave mode
 SPCR |= _BV(SPE);
 
 // turn on interrupts
 SPCR |= _BV(SPIE);
 
 pos = 0;
 process_it = false;
 Serial.print("SPCR = "); 
 Serial.println(SPCR);
 SPI.begin();
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  digitalWrite(5, HIGH);
byte c = SPDR;
 Serial.println(c);
 // add to buffer if room
 if (pos < sizeof buf)
   {
   buf [pos++] = c;
   
   // example: newline means time to process buffer
   if (c == '\n')
     process_it = true;
     
   }  // end of room available
}

// main loop - wait for flag set in interrupt routine
void loop (void)
{
    
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
 if (process_it)
   {
    digitalWrite(4,HIGH);
   buf [pos] = 0;  
   Serial.println (buf);
   pos = 0;
   process_it = false;
   }  // end of flag set
   
}  // end of loop
