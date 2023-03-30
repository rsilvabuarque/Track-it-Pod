#include <math.h>
#include <string.h>
#include <SoftwareSerial.h>
#include <Stepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

struct Data_Package {
  float lat;
  float lng;
  float alt;
  int satellites;
};

const byte address[6] = "00001"; // radio address
#define RADIO_TX_PIN 8 // Pin for CE on radio
#define RADIO_RX_PIN 9 // Pin for CSN on radio
#define SERVO_PIN 4 // digital pin for servo signal
#define CALIBRATION_POT_PIN A7 // analog pin for calibration switch
// if motor changed, lookup max stepper rpm and steps per revolution
#define STEPS_PER_REVOLUTION 2048 // number of steps for one revolution for stepper motor (0.18 deg per step)
#define STEPPER_SPEED 15

float xCal, yCal;
float servoX, servoY;
float prevX, prevY;
float b;
float leftoverSteps; 

Stepper panStepper(STEPS_PER_REVOLUTION, 2, 4, 3, 5); // Create stepper object (IN1, IN2, IN3, IN4)
SoftwareSerial radio(RADIO_CE_PIN, RADIO_CSN_PIN); // Create radio object (CE, CSN)
Data_Package gpsData; // Create Data_Package for storing transmitting GPS data
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Set stepper to max RPM
  panStepper.setSpeed(STEPPER_SPEED);
  
  // Will change to 3-way button in the future
  pinMode(CALIBRATION_POT_PIN, INPUT); // initialize calPot as input

  // Set up radio receiver
  radio.begin(); // Start radio
  radio.openReadingPipe(0, address); // Set address to listen to
  radio.setPALevel(RF24_PA_MAX); // Set power level to max to maximize range
  radio.setDataRate(RF24_250KBPS); // Set data rate to min to maximize range
  radio.startListening(); // Start listening for transmission

  // initialize the LCD and turn on the blacklight
	lcd.begin(16, 2);
	lcd.backlight(); 
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
    gpsData = readRadio();
    x = gpsData.lng;
    y = gpsData.lat;
    
    angle = calcAngle(prevX, prevY, x, y, b);
    steps = angleToSteps(angle);
    
    panStepper.step(steps); // tell stepper to move desired number of steps

    prevX = x;
    prevY = y;

    Serial.println("---");
    Serial.println("Tracking...");
    Serial.print("x: ");
    Serial.print(x, 16);
    Serial.print(" y: ");
    Serial.println(y, 16);
    Serial.println((String)"Satellites: " + gpsData.satellites + ", Altitude: " + gpsData.alt);
    lcd.setCursor(0, 0);
    lcd.print("Track # Sats: ");
    lcd.print(gpsData.satellites);
    delay(100);
  }
}

/** Read GPS data from radio and input it into gpsData Data_package object.
*
* @param gpsData Data_Package object where to store the GPS information pulled
* @return True if radio is available, false otherwise
*/
Data_Package readRadio() {
  Data_Package dp;
  String encoded = "";
  bool start = false;
  char incomingByte;
  while (radio.available()) {
    incomingByte = radio.read(); // Read next byte from radio
    delay(5); // Delay 5 milliseconds
    // If the byte is the <, start adding chars to 'encoded' string. If not, ignore.
    if (incomingByte = '<') { 
      start = true;
    }
    if (start) {
      if (incomingByte != '>') {
        encoded += char(incomingByte);
      } else {
        start = false;
      }
    }
  }
  /*
  INSERT CODE HERE FOR PARSING INFO FROM RECEIVED BYTES
  */
  lcd.setCursor(0, 1);
  lcd.print("Radio read OK :)");
  return dp;
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
  float steps;
  int roundedSteps;
  steps = angle / (360 / STEPS_PER_REVOLUTION);
  roundedSteps = round(steps);
  leftoverSteps += steps - roundedSteps;
  if (leftoverSteps >= 1) {
    roundedSteps += 1;
    leftoverSteps -= 1;
  } else if (leftoverSteps <= -1) {
    roundedSteps -= 1;
    leftoverSteps += 1;
  }
  return roundedSteps;
}

/** 
* Sets position of servo.
*/
// calculate distance between tag and servo (b)
void calibrationStateOne() {
  Serial.println("---");
  Serial.println("Calibration state 1...");
  Serial.println("Please turn the tripod so that the camera faces roughly towards the center of the action");

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
  lcd.setCursor(0, 0);
  lcd.print("Cal1, # Sats: ");
  lcd.print(gpsData.satellites);
  
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
  lcd.setCursor(0, 0);
  lcd.print("Cal2, # Sats: ");
  lcd.print(gpsData.satellites);
  return b;
}


/** 
* Prints desired info on the analog display.
* @param message to be printed
*/
void lcdPrint (String message) {
  lcd.print(message);
}
