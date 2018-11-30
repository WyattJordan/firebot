const byte leftpinA = 2;//A pin -> the interrupt pin 2
const byte leftpinB = 4;//B pin -> the digital pin 4
byte leftPinALast;
int leftduration;//the number of the pulses
boolean leftDirection;//the rotation direction

const byte rightpinA = 3;//A pin -> the interrupt pin 3
const byte rightpinB = 5;//B pin -> the digital pin 5
byte rightPinALast;
int rightduration;//the number of the pulses
boolean rightDirection;//the rotation direction

const byte rightPWM = 5;
const byte leftPWM  = 6;
#define armServo1 9;
#define armServo2 10;
#define armServo3 11;
#define lineIR A0;
#define fireIR A1;

void setup()
{
  Serial.begin(57600);//Initialize the serial port
  pinMode( leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(armServo1, OUTPUT);
  pinMode(armServo2, OUTPUT);
  pinMode(armServo3, OUTPUT);
  EncoderInit();//Initialize the module
  analogWrite(leftPWM, 200);
}

void loop()
{
  Serial.print("Pulse:");
  Serial.println(duration);
  duration = 0;
  delay(100);
}

void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode( leftpinB,INPUT);
  pinMode(rightpinB,INPUT);
  attachInterrupt(digialPinToInterrupt( leftpinA), leftwheelSpeed, CHANGE);//int.0
  attachInterrupt(digialPinToInterrupt(rightpinA), rightwheelSpeed, CHANGE);//int.0
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
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  leftPinALast = Lstate;

  if(!leftDirection)  leftduration++;
  else  leftduration--;
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

  if(!rightDirection)  rightduration++;
  else  rightduration--;
}
