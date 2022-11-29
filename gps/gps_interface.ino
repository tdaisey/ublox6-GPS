#include "TinyGPS++.h"

#define NMEA_DATA_BUF_SIZE   300
#define NMEA_BUF_SIZE        200
#define UBX_DATA_BUF_SIZE    100
#define UBX_BUF_SIZE         20

// Gps Serial Interface
SoftwareSerial GpsSerial(GpsRxPin, GpsTxPin);
TinyGPSPlus    NmeaParser;

void GpsGetData()
{
  char c;
  
  // Data available?
  while (GpsSerial.available() > 0)
  {
    c = GpsSerial.read();
    //Serial.print(c);
    NmeaParser.encode(c);
  }

  // Copy valid/new data to GpsData struct

  if ( (NmeaParser.date.isValid()) && (NmeaParser.date.isUpdated()) )
  {
    GpsData.Year  = NmeaParser.date.year();
    GpsData.Month = NmeaParser.date.month();
    GpsData.Day   = NmeaParser.date.day();
    #ifdef SerialMonitor
      Serial.print("Date: ");
      Serial.print(GpsData.Year);
      Serial.print(" / ");
      Serial.print(GpsData.Month);
      Serial.print(" / ");
      Serial.println(GpsData.Day);
      Serial.println("");
    #endif 
  }
  
  if ( (NmeaParser.time.isValid()) && (NmeaParser.time.isUpdated()) )
  {
    GpsData.Hours   = NmeaParser.time.hour();
    GpsData.Minutes = NmeaParser.time.minute();
    GpsData.Seconds = NmeaParser.time.second();
    #ifdef SerialMonitor
      Serial.print("Time: ");
      Serial.print(GpsData.Hours);
      Serial.print(" : ");
      Serial.print(GpsData.Minutes);
      Serial.print(" : ");
      Serial.println(GpsData.Seconds);
      Serial.println("");
    #endif 
  }
  
  if ( (NmeaParser.location.isValid()) && (NmeaParser.location.isUpdated()) )
  {
    GpsData.Latitude  = NmeaParser.location.lat();
    GpsData.Longitude = NmeaParser.location.lng();
    #ifdef SerialMonitor
      Serial.print("Lat / Long: ");
      Serial.print(GpsData.Latitude);
      Serial.print(" / ");
      Serial.println(GpsData.Longitude);
      Serial.println("");
    #endif 
    #ifdef GpsLed
      GpsLedBlink();
    #endif
  } 

  if ( (NmeaParser.course.isValid()) && (NmeaParser.course.isUpdated()) )
  {
    GpsData.Course = NmeaParser.course.deg();
    #ifdef SerialMonitor
      Serial.print("Course: ");
      Serial.println(GpsData.Course);
      Serial.println("");
    #endif    
  } 

  if ( (NmeaParser.speed.isValid()) && (NmeaParser.speed.isUpdated()) )
  {
    GpsData.Speed = NmeaParser.speed.mph();
    #ifdef SerialMonitor
      Serial.print("Speed: ");
      Serial.println(GpsData.Speed);
      Serial.println("");
    #endif   
  }      
   
  if ( (NmeaParser.altitude.isValid()) && (NmeaParser.altitude.isUpdated()) )
  {
    GpsData.Altitude = NmeaParser.altitude.feet();
    #ifdef SerialMonitor
      Serial.print("Altitude: ");
      Serial.println(GpsData.Altitude);
      Serial.println("");
    #endif   
  }  

  if ( (NmeaParser.satellites.isValid()) && (NmeaParser.satellites.isUpdated()) )
  {
    GpsData.Satellites = NmeaParser.satellites.value();
    #ifdef SerialMonitor
      Serial.print("Satellites: ");
      Serial.println(GpsData.Satellites);
      Serial.println("");
    #endif     
  }
}

void GpsSetConfiguration()
{
  byte b;
  int LoopCnt = 0;

  GpsSerial.begin(GpsBaud);
  
  GpsConfigured = true;

  /*
  // Set and verify GPS configuration 
  while (!GpsConfigured)
  {
    while (GpsSerial.available() > 0)
    {
      b = GpsSerial.read(); 
      GpsParseUbx(b);
    }

    // Periodically send set navigation 
    // engine settings message to the Gps
    if (LoopCnt == 100)
    {
      GpsSetNavigationEngineSettings();
      LoopCnt = 0;
    }

    LoopCnt++;
    delay(10);
  }
  */
}

void GpsParseUbx(byte b)
{ 
  static byte DataBuf[UBX_DATA_BUF_SIZE];
  static int  DataBufLen = 0;
  static byte UbxBuf[UBX_BUF_SIZE];
  static int  UbxBufLen = 0;  
  static bool ParsingUbxMsg = false;  

  // If DataBuf is not full,
  // add the char to LineBuf
  if (DataBufLen < UBX_DATA_BUF_SIZE)
  {
    DataBuf[DataBufLen] = b;
    DataBufLen++;
  }
  else // DataBuf is full
  {
    // Reset DataBuf
    memset(DataBuf,0,sizeof(DataBuf));
    DataBufLen = 0;
    DataBuf[DataBufLen] = b;
    DataBufLen++;
  }

  // Already parsing ubx message?
  if (ParsingUbxMsg)
  {
    UbxBuf[UbxBufLen] = DataBuf[DataBufLen-1];
    UbxBufLen++;
    
    if (UbxBufLen > 9)
    {      
      // Check for Set Nav Engine Settings ACK Msg
      if ( (UbxBuf[0] == 0xB5) &&
           (UbxBuf[1] == 0x62) &&
           (UbxBuf[2] == 0x05) &&
           (UbxBuf[3] == 0x01) &&
           (UbxBuf[4] == 0x02) &&
           (UbxBuf[5] == 0x00) &&
           (UbxBuf[6] == 0x06) &&
           (UbxBuf[7] == 0x24) &&
           (UbxBuf[8] == 0x32) &&
           (UbxBuf[9] == 0x5B) )
           {
              // Success! 
              GpsConfigured = true;
           }
           
      // Reset UbxBuf
      UbxBufLen = 0;
      memset(UbxBuf,0,sizeof(UbxBuf));
      ParsingUbxMsg = false;
    }
  }
  
  else // Not currently parsing a message
  {
    // Need at least two bytes in  
    // DataBuf to check for messages 
    if (DataBufLen > 1)
    {
      // Check for start of a nmea message
      if ( (DataBuf[DataBufLen-2] == 0xB5) && (DataBuf[DataBufLen-1] == 0x62) )
      {
        ParsingUbxMsg = true;

        // Start saving to UbxBuf
        UbxBuf[UbxBufLen] = DataBuf[DataBufLen-2];
        UbxBufLen++;
        UbxBuf[UbxBufLen] = DataBuf[DataBufLen-1];
        UbxBufLen++;
      }
    }
  }  
} 

void GpsGetNavigationEngineSettings()
{
  unsigned char GetNavMsg[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x00, 0x00};

  GpsCalculateChecksum(GetNavMsg, sizeof(GetNavMsg));

  GpsSendMsg(GetNavMsg, sizeof(GetNavMsg));
}

void GpsSetNavigationEngineSettings()
{
  unsigned char SetNavMsg[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                               0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  SetNavMsg[8] = GpsPlatformMode;

  GpsCalculateChecksum(SetNavMsg, sizeof(SetNavMsg));

  GpsSendMsg(SetNavMsg, sizeof(SetNavMsg));
}

void GpsCalculateChecksum(unsigned char *Message, int Length)
{ 
  unsigned char CK_A, CK_B;
 
  CK_A = 0;
  CK_B = 0;

  for (int i=2; i<(Length-2); i++)
  {
    CK_A = CK_A + Message[i];
    CK_B = CK_B + CK_A;
  }
  
  Message[Length-2] = CK_A;
  Message[Length-1] = CK_B;
}

void GpsSendMsg(unsigned char *Message, int Length)
{  
  for (int i=0; i<Length; i++)
  {
    GpsSerial.write(Message[i]);
  }
}
