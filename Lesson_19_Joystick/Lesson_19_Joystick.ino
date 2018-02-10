// http://rztronics.com/control-brushless-motor-using-arduino/
#include <Servo.h>//Using servo library to control ESC
Servo esc; //Creating a servo class with name as esc

#include <LiquidCrystal.h>   // call from Arduino LiquidCrysta library  
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);// select pin
int xpotPin = A0;  // select analog pin 0 as “input pin” for X signal
int ypotPin = A1;  // select analog pin 1 as “input” pin for Y signal
int bpotPin = 6;  // select analog pin 1 as “input” pin for Y signal

// parameters for reading the joystick:
int range = 24;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = range / 4;    // resting threshold
int center = range / 2;       // resting position value

// Ultrasonic Distance
int inputPin = 4; // define ultrasonic signal receiver pin ECHO to D4
int outputPin = 5; // define ultrasonic signal transmitter pin TRIG to D5

void setup()
{
  pinMode(xpotPin, INPUT); //
  pinMode(ypotPin, INPUT); //
  pinMode(bpotPin, INPUT_PULLUP ); //
  lcd.begin(16, 2);  // initialize LCD
  delay(1000); // delay 1000ms

  // ESC
  esc.attach(3); //Specify the esc signal pin,Here as D3
  esc.writeMicroseconds(1000); //initialize the signal to 1000
  
  Serial.begin(9600);
  // Ultrasonic Distance
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
}
void loop ()
{
  int xval, yval, bval;  // initialize variable

  // Ultrasonic Distance
  digitalWrite(outputPin, LOW);

  delayMicroseconds(2);

  digitalWrite(outputPin, HIGH); // Pulse for 10¦Ìs to trigger ultrasonic detection
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int distance = pulseIn(inputPin, HIGH); // Read receiver pulse time
  distance = distance / 58; // Transform pulse time to distance
  // Ultrasinic Distance END

  xval = readAxis(xpotPin);   // xval variable is the value read from signal pin 0
  yval = readAxis(ypotPin);   //yval variable is the value read from signal pin 1
  bval = digitalRead (bpotPin);   //bval variable is the value read from signal pin 2

  // ESC
  int escVal; //Creating a variable val

  escVal= map(xval, 0, 12,1000,2000); //mapping val to minimum and maximum(Change if needed) 
  esc.writeMicroseconds(escVal); //using val as the signal to esc

  // LCD output
  lcd.clear(); // clear screen
  lcd.setCursor(0, 0) ; // set cursor position at first line first position
  lcd.print("X=");      // display X= on the screen
  lcd.print(xval);
  lcd.setCursor(7, 0) ; // set cursor position at first line eighth position
  lcd.print("Y=");      // display Y= on the screen
  lcd.print(yval);
  lcd.setCursor(0, 2) ; // set cursor position at second line first position
  lcd.print("B=");      // display B= on the screen
  lcd.print(bval);
  // Ultrasonic Distance
  lcd.setCursor(7, 2) ; // set cursor position at second line eighth position
  lcd.print("D=");      // display distance= on the screen
  lcd.print(distance);


  lcd.setCursor(14, 2) ; // set cursor position at second line eighth position
  lcd.print("V3");
  
  Serial.print(xval);
  Serial.print(",");
  Serial.print(yval);
  Serial.print(",");
  
  Serial.print("B is : ");
  if (bval == HIGH){
    Serial.println ("not pressed");
  }
  else{
    Serial.println ("PRESSED");
  }
  
  delay(100);                     // delay 0.1second for fresh rate
  
}

// from mouse control
int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}

