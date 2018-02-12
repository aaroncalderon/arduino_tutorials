// WIP
// Need to review comments and clarify the actual purpose of the code
//
// http://rztronics.com/control-brushless-motor-using-arduino/
#include <Servo.h>//Using servo library to control ESC
Servo esc; //Creating a servo class with name as esc

// I am using this to display some of the values on a LCD display
#include <LiquidCrystal.h>   // call from Arduino LiquidCrysta library 
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);// select pin

int xpotPin = A0;  // select analog pin 0 as “input pin” for X signal
int ypotPin = A1;  // select analog pin 1 as “input” pin for Y signal
int bpotPin = 6;   // select analog pin 1 as “input” pin for Button signal

// parameters for reading the joystick:
int range = 200;               // output range of X or Y movement
int threshold = range / 50;    // resting threshold
int center = range / 2;        // resting position value

int rangeT = 200;               / output range throttle
int thresholdT = rangeT / 100; // resting threshold
int centerT = 0;               // resting position is `zero`. this gives you a value `0 - n`

// Ultrasonic Distance
int inputPin = 4; // define ultrasonic signal receiver pin ECHO to D4
int outputPin = 5; // define ultrasonic signal transmitter pin TRIG to D5

//
// process code without delay
// https://www.arduino.cc/en/tutorial/BlinkWithoutDelay
unsigned long previousMillis = 0;     // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to process code
// without delay

void setup()
{
  pinMode(xpotPin, INPUT); //
  pinMode(ypotPin, INPUT); //
  pinMode(bpotPin, INPUT_PULLUP ); //
  lcd.begin(16, 2);  // initialize LCD
  delay(1000); // delay 1000ms

  Serial.begin(9600);
  // Ultrasonic Distance
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  // ESC
  Serial.println("Wait for it... ");
  esc.attach(3); //Specify the esc signal pin,Here as D3
  // this sectin will most likelly change once I test this
  // on the RC Plane.
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  esc.writeMicroseconds(1000); //initialize the signal to 1000
}
void loop ()
{
  int xval, yval, bval, distance;  // initialize variable

  // Ultrasonic Distance
  digitalWrite(outputPin, LOW);

  delayMicroseconds(2);

  digitalWrite(outputPin, HIGH); // Pulse for 10¦Ìs to trigger ultrasonic detection
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  distance = pulseIn(inputPin, HIGH); // Read receiver pulse time
  distance = distance / 58; // Transform pulse time to distance
  // Ultrasinic Distance END

  xval = readAxis(xpotPin, rangeT, thresholdT, centerT );   // xval variable is the value read from signal pin 0
  yval = readAxis(ypotPin, range, threshold, center);   //yval variable is the value read from signal pin 1
  bval = digitalRead (bpotPin);   //bval variable is the value read from signal pin 2

  // ESC
  int escVal; //Creating a variable val

  escVal = map(xval, 0, range, 1000, 2000); //mapping val to minimum and maximum(Change if needed)
  esc.writeMicroseconds(escVal); //using val as the signal to esc

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;


    // LCD output
    lcd.clear(); // clear screen
    lcd.setCursor(0, 0) ; // set cursor position at first line first position
    lcd.print("X=");      // display X= on the screen
    lcd.print(xval);
    lcd.setCursor(6, 0) ; // set cursor position at first line eighth position
    lcd.print("Y=");      // display Y= on the screen
    lcd.print(yval);
    lcd.setCursor(0, 2) ; // set cursor position at second line first position
    lcd.print("B=");      // display B= on the screen
    lcd.print(bval);
    // Ultrasonic Distance
    lcd.setCursor(6, 2) ; // set cursor position at second line eighth position
    lcd.print("D=");      // display distance= on the screen
    lcd.print(distance);

    lcd.setCursor(12, 0);
    lcd.print(escVal);

    lcd.setCursor(12, 2) ; // set cursor position at second line eighth position
    lcd.print("V3");

    Serial.print(xval);
    Serial.print(",");
    Serial.print(yval);
    Serial.print(",");

    Serial.print("B is : ");
    if (bval == HIGH) {
      Serial.println ("not pressed");
    }
    else {
      Serial.println ("PRESSED");
    }
  }

}

// from mouse control
int readAxis(int thisAxis, int range,int threshold, int center) {
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

