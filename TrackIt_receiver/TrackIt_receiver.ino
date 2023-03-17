#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 5; // Pin for CE on radio
const int radioPinCSN = 6; // Pin for CSN on radio
const int servoPin = 4; // digital pin for servo signal
const int calPot = A7; // analog pin for calibration switch
const int potPinX = A0; // analog input pin for X coordinate
const int potPinY = A1; // analog input pin for Y coordinate

Servo myservo; // create servo object
RF24 radio(radioPinCE, radioPinCSN); // create radio object (CE, CSN)

float xCal, yCal;
float x, y;
float a, b, c;
float distance;
float angle;
float calPotState;
float servoX;
float servoY;

struct Data_Package {
  string hi;
  float lat;
  float lng;
  float alt;
  int satellites;
};

Data_Package gpsData;

void setup() {
  Serial.begin(9600); // initialize serial communication at 9600 bits per second

  // Set up servo
  myservo.attach(servoPin); // attaches the servo on pin 9 to the servo object
  pinMode(calPot, INPUT); // initialize calPot as input

  // Set up radio receiver
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  calPotState = analogRead(calPot);
  if (calPotState <= 20) {
    // Calibration State 1

    // read values from radio receiver
    if (radio.available()) {
      radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    }

    Serial.println(gpsData.lat);

    servoX = gpsData.lng;
    servoY = gpsData.lat;

    Serial.println("Calibration state 1...");
    Serial.println(gpsData.hi);
    Serial.print("X: ");
    Serial.print(servoX);
    Serial.print(" Y: ");
    Serial.println(servoY);
  } else if (calPotState <= 600) {
    // Calibration State 2

    // read values from radio receiver
    if (radio.available()) {
      radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    }
    xCal = gpsData.lng;
    yCal = gpsData.lat;

    // calculate distance between tag and servo (a)
    b = sqrt(pow(xCal - servoX, 2) + pow(yCal - servoY, 2));

    Serial.println("Calibration state 2...");
    Serial.print("X: ");
    Serial.print(xCal);
    Serial.print(" Y: ");
    Serial.println(yCal);
  } else {
    // Regular State

    // read values from radio receiver
    if (radio.available()) {
      radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    }
    xCal = gpsData.lng;
    yCal = gpsData.lat;

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

  delay(1000); // loop delay of 1 second
}
