#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 7; // Pin for CE on radio
const int radioPinCSN = 8; // Pin for CSN on radio
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
float oldangle;
float deltaangle;
float calPotState;
float servoX;
float servoY;

struct Data_Package {
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
  radio.begin(); // Start radio
  radio.openReadingPipe(0, address); // Set address to listen to
  radio.setPALevel(RF24_PA_MAX); // Set power level to max to maximize range
  radio.setDataRate(RF24_250KBPS); // Set data rate to min to maximize range
  radio.startListening(); // Start listening for transmission
}

void loop() {
  calPotState = analogRead(calPot);
  Serial.println(calPotState);
  if (calPotState <= 20) {
    // Calibration State 1
    myservo.write(85); // Centralizes the servo motor to the center of its range of possible movements. The user should then turn the tripod to face the center of the action as well
    if (radio.available()) {
      // read values from radio receiver
      radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    }

    servoX = gpsData.lng;
    servoY = gpsData.lat;
    

    Serial.println("---");
    Serial.println("Calibration state 1...");
    Serial.println("Please turn the tripod so that the camera faces roughly towards the center of the action");
    Serial.print("servoX: ");
    Serial.print(servoX, 16);
    Serial.print(" servoY: ");
    Serial.println(servoY, 16);
    Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
    delay(1000);
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
    
    Serial.println("---");
    Serial.println("Calibration state 2...");
    Serial.println("Please move reasonably far from the tripod to the right, going perpendicular to the direction of action");
    Serial.print("xCal: ");
    Serial.print(xCal, 16);
    Serial.print(" yCal: ");
    Serial.println(xCal, 16);
    Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
    // store the old tag angle (85 for the calibration axis since that is the direction of the camera towards the action) for motor speed calculation
    oldangle = 85;
  } else {
    // Regular State

    // read values from radio receiver
    if (radio.available()) {
      radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    }
    x = gpsData.lng;
    y = gpsData.lat;

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
    deltaangle = angle - oldangle;
    // write angle to servo by running the continuous motor clockwise or counterclockwise for a time determined with the motor's angular speed (0.02833 s/degree)
    if (deltaangle > 0) {
      myservo.write(0);
    } else if (deltaangle == 0) {
      myservo.write(90);
    } else {
      myservo.write(180);}
    delay(60 * deltaangle / 0.17);

    Serial.println("---");
    Serial.println("Tracking...");
    Serial.print("x: ");
    Serial.print(x, 16);
    Serial.print(" y: ");
    Serial.println(y, 16);
    Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
    delay(100);
  }
}
