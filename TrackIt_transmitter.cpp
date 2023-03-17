#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "00001"; // radio address
const int radioPinCE = 6; // Pin for CE on radio
const int radioPinCSN = 7; // Pin for CSN on radio

RF24 radio(radioPinCE, radioPinCSN); // create radio object (CE, CSN)

struct Data_Package {
  float lat;
  float lon;
  float alt;
};

Data_Package gpsData;

void setup() {
  Serial.begin(9600); // initialize serial communication at 9600 bits per second

  // Set up radio receiver
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
}
