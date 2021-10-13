#ifdef GpsLed
  void GpsLedOff() 
  {
      digitalWrite(GpsLedPin, LOW);
  }
  
  void GpsLedOn() 
  {
    digitalWrite(GpsLedPin, HIGH);
  }
  
  void GpsLedBlink() 
  {
    digitalWrite(GpsLedPin, LOW);
    delay(100);
    digitalWrite(GpsLedPin, HIGH);
  }
#endif
