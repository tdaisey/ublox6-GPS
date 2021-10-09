#include <SoftwareSerial.h>

#define DATA_BUF_SIZE   400
#define NMEA_BUF_SIZE   200

// Global Variables
int GpsRxPin = 2;
int GpsTxPin = 3;
int GpsBaud  = 9600;   //Default baud of NEO-6M is 9600

SoftwareSerial GpsSerial(GpsRxPin, GpsTxPin);  // GPS Serial Interface

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  // Used to communicate with the Serial Monitor
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  GpsSerial.begin(GpsBaud);
}

void loop()
{
  char c;
  
  // Data available?
  while (GpsSerial.available() > 0)
  {
    // Read one byte at a time
    // from the gps serial port
    c = GpsSerial.read();
    //Serial.print(c);
    ProcessGpsData(c);
  }
}


void ProcessGpsData(char c)
{
  static char DataBuf[DATA_BUF_SIZE];
  static int  DataBufLen = 0;
  static char NmeaBuf[NMEA_BUF_SIZE];
  static int  NmeaBufLen = 0;
  static bool ParsingNmea = false;
  
  Serial.print(c);

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
    Serial.println("Data buffer full, resetting");
    memset(DataBuf,0,sizeof(DataBuf));
    DataBufLen = 0;
  }

  // Need at least two bytes  
  // in DataBuf to do anything 
  if (DataBufLen > 1)
  {
    // Already parsing a message?
    if (ParsingNmea)
    {
      // Save data to nmea buffer 
      // TBD Don't overflow the nmea buffer
      NmeaBuf[NmeaBufLen] = DataBuf[DataBufLen-1];
      NmeaBufLen++;
      
      // Check for nmea msg end 
      if ( (DataBuf[DataBufLen-2] == '\r') && (DataBuf[DataBufLen-1] == '\n') )
      {
        //Serial.println(" -Msg end- ");
        GpsProcessNmeaMsg(NmeaBuf, NmeaBufLen);
        ParsingNmea = false;

        // Reset DataBuf
        memset(DataBuf,0,sizeof(DataBuf));
        DataBufLen = 0;

        // Reset NmeaBuf
        memset(NmeaBuf,0,sizeof(NmeaBuf));
        NmeaBufLen = 0;
      }       
    }
    else // Not currently parsing a message
    {
      // Check for start of a nmea message
      if ( (DataBuf[DataBufLen-2] == '$') && (DataBuf[DataBufLen-1] == 'G') )
      {
        //Serial.print(" -Msg start- ");
        ParsingNmea = true;
        
        // Start saving to nmea buffer
        NmeaBuf[0] = '$';
        NmeaBufLen++;
        NmeaBuf[1] = 'G';
        NmeaBufLen++;
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


void GpsRequestNavEngineSettings()
{
  unsigned char GetNavMsg[] = {0xB5, 0x62, 0x06, 0x24,0x00, 0x00, 0x00};

  GpsCalculateChecksum(GetNavMsg, sizeof(GetNavMsg));

  GpsSendMsg(GetNavMsg, sizeof(GetNavMsg));
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
