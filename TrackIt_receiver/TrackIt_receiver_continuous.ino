#include <math.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

struct Data_Package {
  float lat;
  float lng;
  float alt;
  int satellites;
};

const byte address[6] = "00001"; // radio address
#define RADIO_CE_PIN 7 // Pin for CE on radio
#define RADIO_CSN_PIN 8 // Pin for CSN on radio
#define SERVO_PIN 4 // digital pin for servo signal
#define CALIBRATION_POT_PIN A7 // analog pin for calibration switch

float xCal, yCal;
float servoX, servoY;
float prevX, prevY;
float b;

Servo myservo; // create servo object
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN); // create radio object (CE, CSN)
Data_Package gpsData;

void setup() {
  Serial.begin(9600); // initialize serial communication at 9600 bits per second

  // Set up servo
  myservo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object
  pinMode(CALIBRATION_POT_PIN, INPUT); // initialize calPot as input

  // Set up radio receiver
  radio.begin(); // Start radio
  radio.openReadingPipe(0, address); // Set address to listen to
  radio.setPALevel(RF24_PA_MAX); // Set power level to max to maximize range
  radio.setDataRate(RF24_250KBPS); // Set data rate to min to maximize range
  radio.startListening(); // Start listening for transmission
}

void loop() {
  float calibrationPot;
  calibrationPot = analogRead(CALIBRATION_POT_PIN);
  Serial.println(calibrationPot);
  if (calibrationPot <= 20) {
    calibrationStateOne();    
  } else if (calibrationPot <= 600) {
    b = calibrationStateTwo();
    prevX = xCal;
    prevY = yCal;
  } else {
    // Regular State
    float x, y, angle;
    int steps;
    // Read values from radio receiver
    if (readRadio(gpsData)) {
      x = gpsData.lng;
      y = gpsData.lat;    
    } else {
      Serial.println("Radio unavailable.");
    }
    
    angle = calcAngle(prevX, prevY, x, y, b);
    steps = angleToSteps(angle);
    
    // write angle to servo, subtracting 85 due to centralization of the servo motor axis
    myservo.write(angle); // set servo angle

    prevX = x;
    prevY = y;

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

/** Read GPS data from radio and input it into gpsData Data_package object.
*
* @param gpsData Data_Package object where to store the GPS information pulled
* @return True if radio is available, false otherwise
*/
bool readRadio(Data_Package gpsData) {
  if (radio.available()) {
    // read values from radio receiver
    radio.read(&gpsData, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    return true;
  }
  return false;
}

/** Calculate pan angle at which camera must be given positions of camera, tag, and calibration.
*
* @param a Distance between servo and tag, in units of latitude and longitude
* @param b Distance between calibration point and servo, in units of latitude and longitude
* @param c Distance between calibration point and tag, in units of latitude and longitude
* @return Calculated angle
*/
float calcAngle(float prevX, float prevY, float newX, float newY, float b) {
  float a, c, angle;
  // calculate distance between tag and servo (b)
  a = sqrt(pow(newX - servoX, 2) + pow(newY - servoY, 2));
  // calculate distance between tag and original tag position (c)
  c = sqrt(pow(prevX - newX, 2) + pow(prevY - newY, 2));

  // calculate angle between original tag position and new position
  if (newY >= newX*((servoY - prevY) / (servoX - prevX))) {
    angle = -1 * (acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)) * 180 / (atan(1) * 4));
  } else {
    angle = acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)) * 180 / (atan(1) * 4);
  }

  return angle;
}

/** Calculate number of steps to take based on pan angle required.
*
* @param angle Angle required to turn to new tag position
* @return Calculated steps
*/
int angleToSteps(float angle) {

}

/** 
* Sets position of servo.
*/
// calculate distance between tag and servo (b)
void calibrationStateOne() {
  Serial.println("---");
  Serial.println("Calibration state 1...");
  Serial.println("Please turn the tripod so that the camera faces roughly towards the center of the action");

  myservo.write(85); // Centralizes the servo motor to the center of its range of possible movements. The user should then turn the tripod to face the center of the action as well
  if (readRadio(gpsData)) {
    servoX = gpsData.lng;
    servoY = gpsData.lat;    
  } else {
    Serial.println("Radio unavailable.");
  }
  
  Serial.print("servoX: ");
  Serial.print(servoX, 16);
  Serial.print(" servoY: ");
  Serial.println(servoY, 16);
  Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
  delay(1000);
}

/** 
* Sets calibration point of reference.
*/
float calibrationStateTwo() {
  float b;
  Serial.println("---");
  Serial.println("Calibration state 2...");
  Serial.println("Please move reasonably far from the tripod to the right, going perpendicular to the direction of action");

  // read values from radio receiver
  if (readRadio(gpsData)) {
    xCal = gpsData.lng;
    yCal = gpsData.lat;
  } else {
    Serial.println("Radio unavailable.");
  }

  // calculate distance between tag and servo (a)
  b = sqrt(pow(xCal - servoX, 2) + pow(yCal - servoY, 2));
  
  Serial.print("xCal: ");
  Serial.print(xCal, 16);
  Serial.print(" yCal: ");
  Serial.println(xCal, 16);
  Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
  
  return b;
}
