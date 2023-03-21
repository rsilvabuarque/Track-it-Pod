#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

struct Data_Package {
  float lat;
  float lng;
  float alt;
  int satellites;
};

const byte address[6] = "00001"; // radio address
#define RADIO_CE_PIN 7 // Pin for CE on radio
#define RADIO_CSN_PIN 8 // Pin for CSN on radio
#define GPS_RX_PIN 4 // Pin for RX on GPS
#define GPS_TX_PIN 3 // Pin for TX on GPS
#define GPS_BAUD_RATE 9600 // Baud rate for GPS

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN); // Create radio object (CE, CSN)
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // Create a SoftwareSerial object to communicate with the GPS sensor
TinyGPSPlus gps; // Create an instance of the TinyGPS++ object
Data_Package gpsData; // Create Data_Package object for storing and sending gps data

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600); 
  gpsSerial.begin(GPS_BAUD_RATE);

  // Set up radio transmitter
  radio.begin(); // Start radio
  radio.openWritingPipe(address); // Set address to write to
  radio.setPALevel(RF24_PA_MAX); // Maximum power level to maximize range
  radio.setDataRate(RF24_250KBPS); // Minimum data range to maximize range
  radio.stopListening(); // Stop listening since this is a transmitter
  
  // Default values
  gpsData.lat = gpsData.lng = gpsData.alt = gpsData.satellites = 9999;
}

void loop() {
  // Read the GPS sensor data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsData.lat = gps.location.lat();
        gpsData.lng = gps.location.lng();
      } else {
        gpsData.lat = 9999;
        gpsData.lng = 9999;        
      }
      if (gps.altitude.isValid()) {
        gpsData.alt = gps.altitude.value();
      } else {
        gpsData.alt = 9999;
      }
      if (gps.satellites.isValid()) {
        gpsData.satellites = gps.satellites.value();
      } else {
        gpsData.satellites = 9999;
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
    }
  }
  // Transmit GPS sensor data
  radio.write(&gpsData, sizeof(Data_Package));
}
