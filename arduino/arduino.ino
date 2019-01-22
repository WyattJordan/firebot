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

const byte rightPWM = 5;
const byte leftPWM  = 6;
#define armServo1 9
#define armServo2 10
#define armServo3 11
#define lineIR A0
#define fireIR A1

char buff[10];
int pos;
void setup()
{
  Serial.begin(9600);//Initialize the serial port
  pos = 0;
  Serial.begin(9600);//Initialize the serial port
  while(!Serial) {;}
  Serial.println("starting motor...");
  Serial.println("serial ready");
  pinMode( leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(armServo1, OUTPUT);
  pinMode(armServo2, OUTPUT);
  pinMode(armServo3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  EncoderInit();//Initialize the module
  analogWrite( leftPWM, 127);
  analogWrite(rightPWM, 127);
}

void loop()
{
  digitalWrite(LED_BUILTIN, LOW);
  while(Serial.available()){
   buff[pos++] = Serial.read();
  }
  
  switch (buff[0]){
    case 'm':
    digitalWrite( leftPWM, int(buff[1]));
    digitalWrite(rightPWM, int(buff[2]));
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    break;
    
    case 'l':
    Serial.println(leftDuration);
    leftDuration = 0;
    break;
    
    case 'r':
    Serial.println(rightDuration);
    rightDuration = 0;
    break;
    
    case 'a':
    digitalWrite(armServo1, int(buff[1]));
    digitalWrite(armServo2, int(buff[2]));
    digitalWrite(armServo3, int(buff[3]));
    break;
    
  }
  delay(20);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1);
}

void EncoderInit()
{
  leftDirection = true;//default -> Forward
  rightDirection = true;//default -> Forward
  pinMode( leftpinB,INPUT);
  pinMode(rightpinB,INPUT);
  attachInterrupt(0, leftwheelSpeed, CHANGE);//int.0
  //attachInterrupt(, rightwheelSpeed, CHANGE);//int.1
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
