#include <SoftwareSerial.h>

#define SerialMonitor;

// Global Variables
int  GpsRxPin = 2;
int  GpsTxPin = 3;
int  GpsBaud  = 9600;   
bool GpsConfigured = false;
int  GpsPlatformMode = 0;
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

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  // Used to communicate with the Serial Monitor
  #ifdef SerialMonitor
    Serial.begin(9600);
  #endif

  // Configure the Gps 
  GpsSetConfiguration();
  #ifdef SerialMonitor
    Serial.println("Gps Configured!");
  #endif
}

// Main Loop
void loop()
{
  GpsProcessData();
}
