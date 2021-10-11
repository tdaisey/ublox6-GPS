#include <SoftwareSerial.h>

#define DATA_BUF_SIZE   400
#define NMEA_BUF_SIZE   200
#define UBX_BUF_SIZE    100
#define PARSE_NMEA      1
#define PARSE_UBX       2

// Global Variables
int  GpsRxPin = 2;
int  GpsTxPin = 3;
int  GpsBaud  = 9600;   
int  ParseProtocol = PARSE_UBX;
bool GpsConfigured = false;

/*
 * Dynamic Platform model:
 * 0 = Portable
 * 2 = Stationary
 * 3 = Pedestrian
 * 4 = Automotive
 * 5 = Sea
 * 6 = Airborne with <1g Acceleration
 * 7 = Airborne with <2g Acceleration
 * 8 = Airborne with <4g Acceleration
 */
int DesiredPlatformMode = 7;

// Gps Serial Interface
SoftwareSerial GpsSerial(GpsRxPin, GpsTxPin);  

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  // Used to communicate with the Serial Monitor
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  GpsSerial.begin(GpsBaud);
}

// Main Loop
void loop()
{
  char c;
  byte b;
  static int LoopCnt = 0;
  
  // Data available?
  while (GpsSerial.available() > 0)
  {
    // Read one byte at a time
    // from the gps serial port
    if (ParseProtocol == PARSE_NMEA)
    {
      c = GpsSerial.read();
      GpsParseNmea(c);
      Serial.print(c);
    }
    else if (ParseProtocol == PARSE_UBX)
    {
      b = GpsSerial.read(); 
      GpsParseUbx(b);
      Serial.print("[");
      Serial.print(b, HEX);
      Serial.print("]");
    }
  }

  // Gps not configured?
  if (!GpsConfigured)
  {
    // Periodically request the 
    // Nav Engine Settings Msg
    if (LoopCnt == 100)
    {
      Serial.println("");
      Serial.println("");
      Serial.println("Setting Navigation Engine");
      GpsSetNavigationEngineSettings();
      LoopCnt = 0;
    }
  }

  LoopCnt++;

  delay(10);
}

void GpsParseNmea(char c)
{  
  static char DataBuf[DATA_BUF_SIZE];
  static int  DataBufLen = 0;
  static char NmeaBuf[NMEA_BUF_SIZE];
  static int  NmeaBufLen = 0;
  static bool ParsingNmeaMsg = false;

  // If DataBuf is not full,
  // add the char to LineBuf
  if (DataBufLen < DATA_BUF_SIZE)
  {
    DataBuf[DataBufLen] = c;
    DataBufLen++;
  }
  else // DataBuf is full
  {
    // Reset DataBuf
    Serial.println("");
    Serial.println("Data buffer full, resetting");
    Serial.println("");
    memset(DataBuf,0,sizeof(DataBuf));
    DataBufLen = 0;
    DataBuf[DataBufLen] = c;
    DataBufLen++;
  }

  // Already parsing nmea message?
  if (ParsingNmeaMsg)
  {
    // Save data to nmea buffer, 
    // don't overflow it
    if (NmeaBufLen < NMEA_BUF_SIZE)
    {
      NmeaBuf[NmeaBufLen] = DataBuf[DataBufLen-1];
      NmeaBufLen++;
    }
    else // NmeaBuf is full
    {
      // Reset NmeaBuf 
      ParsingNmeaMsg = false;
      memset(NmeaBuf,0,sizeof(NmeaBuf));
      NmeaBufLen = 0;
    }
    
    // Check for nmea msg end 
    if ( (DataBuf[DataBufLen-2] == '\r') && (DataBuf[DataBufLen-1] == '\n') )
    {
      //Serial.println(" -Msg end- ");
      GpsProcessNmeaMsg(NmeaBuf, NmeaBufLen);
      
      // Reset DataBuf
      memset(DataBuf,0,sizeof(DataBuf));
      DataBufLen = 0;

      // Reset NmeaBuf
      ParsingNmeaMsg = false;
      memset(NmeaBuf,0,sizeof(NmeaBuf));
      NmeaBufLen = 0;
    }       
  }

  else // Not currently parsing a message
  {
    // Need at least two bytes in  
    // DataBuf to check for messages 
    if (DataBufLen > 1)
    {
      // Check for start of a nmea message
      if ( (DataBuf[DataBufLen-2] == '$') && (DataBuf[DataBufLen-1] == 'G') )
      {
        //Serial.print(" -Msg start- ");
        ParsingNmeaMsg = true;
        
        // Start saving to nmea buffer
        NmeaBuf[0] = '$';
        NmeaBufLen++;
        NmeaBuf[1] = 'G';
        NmeaBufLen++;
      }
    }
  }
}

void GpsParseUbx(byte b)
{ 
  static byte DataBuf[DATA_BUF_SIZE];
  static int  DataBufLen = 0;
  static byte UbxBuf[UBX_BUF_SIZE];
  static int  UbxBufLen = 0;  
  static bool ParsingUbxMsg = false;  

  // If DataBuf is not full,
  // add the char to LineBuf
  if (DataBufLen < DATA_BUF_SIZE)
  {
    DataBuf[DataBufLen] = b;
    DataBufLen++;
  }
  else // DataBuf is full
  {
    // Reset DataBuf
    Serial.println("");
    Serial.println("Data buffer full, resetting");
    Serial.println("");
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
              Serial.println("");
              Serial.println("!!! GPS CONFIGURED !!!");
              Serial.println("");
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

void GpsProcessNmeaMsg(char* NmeaMsg, int NnmeaMsgLen)
{ 
  //Serial.println("Nmea Message: ");
  for(int x=0; x<NnmeaMsgLen; x++)
  {
    Serial.print(NmeaMsg[x]);
  }
  Serial.println("");
  Serial.println("");
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

  SetNavMsg[8] = DesiredPlatformMode;

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
