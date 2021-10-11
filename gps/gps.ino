#include <SoftwareSerial.h>

// Global Variables
int  GpsRxPin = 2;
int  GpsTxPin = 3;
int  GpsBaud  = 9600;   
bool GpsConfigured = false;

/*
 * Gps Platform Modes:
 * 0 = Portable
 * 2 = Stationary
 * 3 = Pedestrian
 * 4 = Automotive
 * 5 = Sea
 * 6 = Airborne with <1g Acceleration
 * 7 = Airborne with <2g Acceleration
 * 8 = Airborne with <4g Acceleration
 */
int DesiredGpsPlatformMode = 0;

// Gps Serial Interface
SoftwareSerial GpsSerial(GpsRxPin, GpsTxPin);  

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  // Used to communicate with the Serial Monitor
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  GpsSerial.begin(GpsBaud);

  // Configure the Gps 
  Serial.println("Configuring Gps");
  GpsSetConfiguration();
  Serial.println("Gps Configured!");
}

// Main Loop
void loop()
{
  char c;
  
  // Data available?
  while (GpsSerial.available() > 0)
  {
    // Read one byte at a time
    // from the gps serial port
    c = GpsSerial.read();
    GpsParseNmea(c);
  }
}
