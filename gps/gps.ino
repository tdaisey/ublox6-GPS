#include <SoftwareSerial.h>

#define SerialMonitor;
#define GpsLed;

// Global Variables
int  GpsRxPin = 2;
int  GpsTxPin = 3;
int  GpsBaud  = 9600;  
int  GpsPlatformMode = 0; // See README 
bool GpsConfigured = false; 

#ifdef GpsLed
  int GpsLedPin = 4;
#endif 
 
struct GpsData_t
{
  uint16_t Year;
  uint8_t  Month;
  uint8_t  Day;
  uint8_t  Hours; 
  uint8_t  Minutes; 
  uint8_t  Seconds;
  double   Latitude;  // Degrees
  double   Longitude; // Degrees
  double   Course;    // Degrees
  double   Speed;     // Mph
  double   Altitude;  // Feet
  uint32_t Satellites;
} GpsData;

void setup()
{
  #ifdef SerialMonitor
    Serial.begin(9600);
  #endif

  #ifdef GpsLed 
    pinMode(GpsLedPin, OUTPUT);
  #endif

  GpsSetConfiguration();
  #ifdef SerialMonitor
    Serial.println("Gps Configured!");
    Serial.println("");
  #endif
  #ifdef GpsLed
    GpsLedOn();
  #endif 
}

// Main Loop
void loop()
{
  GpsGetData();
}
