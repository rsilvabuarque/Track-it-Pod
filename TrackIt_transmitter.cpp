#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 6; // Pin for CE on radio
const int radioPinCSN = 7; // Pin for CSN on radio
const int RX_PIN 3
const int TX_PIN 4

RF24 radio(radioPinCE, radioPinCSN); // create radio object (CE, CSN)
SoftwareSerial gpsSerial(RX_PIN, TX_PIN); // Create a SoftwareSerial object to communicate with the GPS sensor
TinyGPSPlus gps; // Create an instance of the TinyGPS++ object

struct Data_Package {
  float lat;
  float lng;
  float alt;
};

Data_Package gpsData;

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600); 
  gpsSerial.begin(9600);

  // Set up radio receiver
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  // Read the GPS sensor data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsData.lat = gps.location.lat();
        gpsData.lng = gps.location.lng();
        gpsData.alt = 0;
      }
    }
  }
  // Transmit GPS sensor data
  radio.write(&gpsData, sizeof(Data_Package));
  delay(333);
}
