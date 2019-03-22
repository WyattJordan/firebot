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

float time1, time2, motortimestamp, enctimestamp;
bool ledState, discon;
int setMotorsCount, encCount;

void setup() {
  
  // encoders
  pinMode(LED_BUILTIN, OUTPUT);
  leftDirection = true; //default -> Forward
  rightDirection = true;//default -> Forward
  leftDuration = 0;
  rightDuration = 0;
  pinMode( leftpinB,INPUT);
  pinMode(rightpinB,INPUT);
  attachInterrupt(0, leftwheelSpeed, CHANGE);//int.0 (pin2)
  attachInterrupt(1, rightwheelSpeed,CHANGE);//int.1 (pin3)
  zeroFlag = false;
  zeroStamp = millis();
  
  // drive
  lPWM = rPWM = 0; // stopped
  lforward = rforward = true;
  pinMode(leftDrivePin, OUTPUT);
  pinMode(rightDrivePin, OUTPUT);
  pinMode( leftDirPin,OUTPUT);
  pinMode(rightDirPin,OUTPUT);
  analogWrite(leftDrivePin, lPWM);
  analogWrite(rightDrivePin, rPWM);
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, HIGH);  

  Error = outputEverything = false;
  sDebug = true;
  writeBack = true;
  
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial port to connect
	blinkLED(3);

  getBuffPos = 0;
  code = 0;
  time1 = 0;
ledState = false;
	motortimestamp = millis();
	enctimestamp = millis();
	setMotorsCount = encCount = 0;
discon = false;
}

const int num_codes = 4;
int data_length[num_codes] = {0, 4, 0, 5};
// number of bytes coming in for a given code (index is -1*code)
// code =  0 -> 0 bytes returned
//	this is a placeholder, not an actual command
// code = -1 -> 4 bytes = {leftDir, lPWM, rightDir, rPWM}
//	set the motor directions and powers
// code = -2 -> 0 bytes = {}
//	send back the encoder values (4 bytes)
// code = -3 -> 5 bytes = {D3_, D6_, D9_, D10_, D11_ }
//	send the PWM values for the pins over I2C to secondary arduino

void loop() {
if(millis()-motortimestamp > 25 && !discon && setMotorsCount > 50){
	digitalWrite(LED_BUILTIN, HIGH);
	getBuffPos = 0;
	analogWrite(leftDrivePin,  0);
	analogWrite(rightDrivePin, 0);
	discon = true;
	
}
  if(Serial.available()>0){
    char c = Serial.read();
    getBuff[getBuffPos++] = c;
//	Serial.print("getBuffPos ="); Serial.println(getBuffPos);
	// a 2-byte int code is sent before any data
	if(getBuffPos == 2 ){
	     code = (getBuff[1] << 8) | getBuff[0];
	     if(!(code * -1 < num_codes) ) { // if a weird code is sent reset
			code = 0;
			getBuffPos = 0;
		}
	  }
	// big problem was this being an else if, if it's get encoders
	// there's no data to receive so it should run immediately
	if(getBuffPos >= 2 + data_length[-1*code]){
		if(code == 0) {;}
		else if(code == -1) {setMotors();}
		else if(code == -2) {sendEncoders();}
		else if(code == -3) {sendI2C();}
		//Serial.print("code was ");
		//Serial.println(code);
		getBuffPos = 0; // all the data for that msg type received
		code = 0;
	}

  }

  else if(millis() - time1 > 1000){ // code to run every second
/*    time1 = millis();
	ledState = !ledState;
	if(ledState) {
		digitalWrite(LED_BUILTIN,HIGH);
	}
	else{
		digitalWrite(LED_BUILTIN,LOW);
	}
*/
    //Serial.print("doing");
  }
  if(Error){
    sprintf(msg, "lPWM = %d rPWM = %d --- lPWM2 = %d rPWM2 = %d\n",
    lPWM,rPWM,lPWM2,rPWM2);
    //Serial.print(msg);
    Error = false;
  }
    
  if(zeroFlag){
      //digitalWrite(LED_BUILTIN, HIGH);
      zeroFlag = false;
  }
  if(millis()-zeroStamp > 500){      
      ///digitalWrite(LED_BUILTIN, LOW);
  }
} // end of main loop

/////////////////////////////////////////////////////////////////////
void outputBuff(int len, bool asIntegers){
	for(int i=0; i<len+2; i++){
		if(asIntegers) {Serial.print((unsigned char)getBuff[i]);}
		else {Serial.print(getBuff[i]);}
		Serial.print(" ");
	}
	Serial.println();
}

void setMotors(){
	//Serial.println("Ran set Motors with buff = ");
	//outputBuff(4,true); 
	digitalWrite(leftDirPin,  getBuff[2] == 'f' ? HIGH : LOW);
	digitalWrite(rightDirPin, getBuff[4] == 'f' ? HIGH : LOW);
	analogWrite(leftDrivePin,  getBuff[3]);
	analogWrite(rightDrivePin, getBuff[5]);
	motortimestamp = millis();
	setMotorsCount++;
	discon = false;
}

void sendEncoders(){
byte send[4] = {highByte(leftDuration), lowByte(leftDuration),
 highByte(rightDuration), lowByte(rightDuration)};
Serial.write(send,4);
	/*Serial.write(highByte(leftDuration));
	Serial.write(lowByte(leftDuration));
	Serial.write(highByte(rightDuration));
	Serial.write(lowByte(rightDuration)); 
*/
	leftDuration = 0;
	rightDuration = 0;
	enctimestamp = millis();
	encCount++;
}

void sendI2C(){
//	Serial.print("Ran sendI2C with buff = ");
//	outputBuff(5,true); 
}

void blinkLED(int n){
 for(int i=0; i<n; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
  }
    digitalWrite(LED_BUILTIN, LOW);

}
////////////////////////////////////////////////////////////////
///////////////////Encoder Interrupts///////////////////////////
////////////////////////////////////////////////////////////////
void leftwheelSpeed()
{
zeroFlag = true;
zeroStamp = millis();
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
zeroFlag = true;
zeroStamp = millis();
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

// swapped increment and decrement because they were backwards
  if(!rightDirection)  rightDuration--;
  else  rightDuration++;
}
