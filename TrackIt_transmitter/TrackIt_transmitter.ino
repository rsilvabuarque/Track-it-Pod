#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 6; // Pin for CE on radio
const int radioPinCSN = 7; // Pin for CSN on radio
#define RX_PIN 3
#define TX_PIN 4

RF24 radio(radioPinCE, radioPinCSN); // create radio object (CE, CSN)
SoftwareSerial gpsSerial(RX_PIN, TX_PIN); // Create a SoftwareSerial object to communicate with the GPS sensor
TinyGPSPlus gps; // Create an instance of the TinyGPS++ object

struct Data_Package {
  float lat;
  float lng;
  float alt;
  int satellites;
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
        Serial.print("GPS location valid!")
        gpsData.lat = gps.location.lat();
        gpsData.lng = gps.location.lng();
        gpsData.alt = gps.altitude.value();
        gpsData.satellites = gps.satellites.value();
      }
    }
  }
  // Debug
  Serial.println("---");
  Serial.print("Latitude: ");
  Serial.println(gpsData.lat, 16);
  Serial.print("Longitude: ");
  Serial.println(gpsData.lng, 16);
  Serial.print("Altitude (cm): ");
  Serial.println(gpsData.alt, 2);
  Serial.print("Satellites: ");
  Serial.println(gpsData.satellites);
  // Transmit GPS sensor data
  radio.write(&gpsData, sizeof(Data_Package));
  Serial.println("---");
  delay(333);
}
