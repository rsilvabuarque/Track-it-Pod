#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 7; // Pin for CE on radio
const int radioPinCSN = 8; // Pin for CSN on radio
#define RX_PIN 3 // Pin for RX on GPS
#define TX_PIN 4 // Pin for TX on GPS

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

  // Set up radio transmitter
  radio.begin(); // Start radio
  radio.openWritingPipe(address); // Set address to write to
  radio.setPALevel(RF24_PA_MAX); // Maximum power level to maximize range
  radio.setDataRate(RF24_250KBPS); // Minimum data range to maximize range
  radio.stopListening(); // Stop listening since this is a transmitter
}
void loop() {
  // Read the GPS sensor data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print("GPS location valid!");
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
  //radio.write(&gpsData, sizeof(Data_Package));
  String testStr = "messi";
  radio.write(&testStr, sizeof(testStr));
  Serial.println("---");
  delay(333);
}
