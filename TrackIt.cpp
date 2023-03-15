#include <Servo.h>
#include <math.h>

Servo myservo; // create servo object to control a servo

int servoPin = 9; // digital pin for servo signal
int switchPin = 7;
int potPinX = A0; // analog input pin for X coordinate
int potPinY = A1; // analog input pin for Y coordinate

float xCal, yCal;
float x, y; // variables to store X and Y coordinates
float a, b, c;
float distance;
float angle;

int switchState = 0;
float servoX = 100;
float servoY = 200;

void setup() {
  myservo.attach(servoPin); // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); // initialize serial communication at 9600 bits per second
  pinMode(switchPin, INPUT); // initialize switchPin as input
}

void loop() {
  switchState = digitalRead(switchPin);
  if (switchState == HIGH) {

    // read the values from the potentiometers
  	xCal = analogRead(potPinX);
  	yCal = analogRead(potPinY);

    // calculate distance between tag and servo (a)
    b = sqrt(pow(xCal - servoX, 2) + pow(yCal - servoY, 2));

    Serial.println("Calibrating...");
    Serial.print("X: ");
    Serial.print(xCal);
    Serial.print(" Y: ");
    Serial.println(yCal);
    delay(1000); // wait for the servo to move to the new position
  } else {

    // read the values from the potentiometers
  	x = analogRead(potPinX);
  	y = analogRead(potPinY);

    // calculate distance between tag and servo (b)
    a = sqrt(pow(x - servoX, 2) + pow(y - servoY, 2));
    distance = a;
    // calculate distance between tag and original tag position (c)
    c = sqrt(pow(xCal - x, 2) + pow(yCal - y, 2));

    // calculate angle between original tag position and new position
    if (y >= x*((servoY - yCal) / (servoX - xCal))) {
      angle = 360 - (acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)) * 180 / (atan(1) * 4));
    } else {
      angle = acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)) * 180 / (atan(1) * 4);
    }
    // write angle to servo, subtracting 90 due to
    myservo.write(angle - 90); // set servo angle

    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Servo angle: ");
    Serial.println(angle);
  }

  delay(15); // wait for the servo to move to the new position
}
