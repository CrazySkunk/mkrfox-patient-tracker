 
#include<string.h>
 
#define DEBUG true
 
int PWR_KEY = 9;
int RST_KEY = 6;
int LOW_PWR_KEY = 5;
 
String msg = String("");
int SmsContentFlag = 0;
String mob;
String loct;
void setup()
{
  pinMode(PWR_KEY, OUTPUT);
  pinMode(RST_KEY, OUTPUT);
  pinMode(LOW_PWR_KEY, OUTPUT);
  digitalWrite(RST_KEY, LOW);
  digitalWrite(LOW_PWR_KEY, HIGH);
  digitalWrite(PWR_KEY, HIGH);
 
  //      String msg = String("");
  //    int SmsContentFlag = 0;
 
  SerialUSB.begin(115200);
  Serial1.begin(115200);
  //modulePowerOn();
  delay(2000);
  SerialUSB.println("Checking Module...");
  int i = 1;
  String res;
  while (i) {
    Serial1.println("AT");
    while (Serial1.available() > 0) {
      if (Serial1.find("OK"))
        i = 0;
    }
    delay(500);
  }
  SerialUSB.println("Module Connected");
  //GprsTextModeSMS();
  SerialUSB.print("Text Mode: ");
  i = 1;
  while (i) {
    Serial1.println("AT+CMGF=1\r");
    while (Serial1.available() > 0) {
      if (Serial1.find("OK"))
        i = 0;
    }
    delay(500);
  }
  SerialUSB.println("ON");
  Serial1.println("AT+GPS=1");
  SerialUSB.println("GPS Intializing...");
  delay(5000);
  SerialUSB.println("GPS Initialized");
  SerialUSB.flush();
  i = 1;
  String str;
  while (i) {
    Serial1.println("AT+LOCATION=2");
    delay(100);
    while (Serial1.available() <= 0);
    if (Serial1.find("AT+LOCATION=2")) {
      str = Serial1.readString();
      //SerialUSB.println(str);
      i = 0;
    }
    delay(500);
  }
  loct = str.substring(4, 23);
  SerialUSB.print("Location: ");
  SerialUSB.println(loct);
  //while(1);
  SerialUSB.println("Send sms to get the location");
}
 
 
void loop()
{
  char SerialInByte;
 
  if (Serial1.available())
  {
    //char SerialInByte;
    SerialInByte = (unsigned char)Serial1.read();
    if ( SerialInByte == 13 ) //0x0D
    {
      ProcessGprsMsg();
    }
    if ( SerialInByte == 10 )
    {
    }
    else
    {
      //EN: store the current character in the message string buffer
      msg += String(SerialInByte);
    }
  }
  if (SmsContentFlag == 1) {
    int i;
    SerialUSB.println("Flag Cleared");
    SerialUSB.print("Message: ");
    SerialUSB.println(msg);
    if (msg.indexOf("Location")) {
      SerialUSB.println("Message Received");
      i = msg.indexOf("+91");
      //SerialUSB.print("index: ");
      //SerialUSB.println(i);
      //SerialUSB.println(msg[i+1]);
      i = i + 3;
      for (int j = 0; j < 10; j++, i++) {
        mob += msg[i];
      }
      SerialUSB.print("Mobile: ");
      SerialUSB.println(mob);
      String cmd = "AT+CMGS=\"";
      cmd += mob;
      cmd += "\"";
      cmd += "\r";
      Serial1.println(cmd);
      delay(100);
 
      Serial1.print("Your Vehicle Current Location ");
      Serial1.print(loct);
      Serial1.print("\n");
      Serial1.print("Check Map: \n");
      Serial1.print(" https://www.google.com/maps/@");
      Serial1.println(loct);
      Serial1.println((char)26);
      delay(1000);
    }
    ClearGprsMsg();
    SmsContentFlag = 0;
    msg.remove(0);
    mob.remove(0);
  }
  delay(100);
}
 
 
// EN: Request Text Mode for SMS messaging
void GprsTextModeSMS()
{
  Serial1.println( "AT+CMGF=1" );
}
 
void GprsReadSmsStore( String SmsStorePos )
{
  // Serial.print( "GprsReadSmsStore for storePos " );
  // Serial.println( SmsStorePos );
  Serial1.print( "AT+CMGR=" );
  Serial1.println( SmsStorePos );
}
 
// EN: Clear the GPRS shield message buffer
void ClearGprsMsg()
{
  msg = "";
}
 
// EN: interpret the GPRS shield message and act appropiately
void ProcessGprsMsg()
{
  SerialUSB.println("");
  //  Serial.print( "GPRS Message: [" );
  SerialUSB.print( msg );
  //    Serial.println( "]" );
  if ( msg.indexOf( "Call Ready" ) >= 0 )
  {
    SerialUSB.println( "*** GPRS Shield registered on Mobile Network ***" );
    GprsTextModeSMS();
  }
 
  //EN: unsolicited message received when getting a SMS message
  if ( msg.indexOf( "+CIEV" ) >= 0 )
  {
    SerialUSB.println( "*** SMS Received ***" );
  }
 
  //EN: SMS store readed through UART (result of GprsReadSmsStore request)
  if ( msg.indexOf( "+CMT:" ) >= 0 )
  {
    // EN: Next message will contains the BODY of SMS
    SmsContentFlag = 1;
    // EN: Following lines are essentiel to not clear the flag!
    //ClearGprsMsg();
    return;
  }
 
  // EN: +CMGR message just before indicate that the following GRPS Shield message
  //     (this message) will contains the SMS body
  if ( SmsContentFlag == 1 )
  {
    SerialUSB.println( "*** SMS MESSAGE CONTENT ***" );
    SerialUSB.println( msg );
    SerialUSB.println( "*** END OF SMS MESSAGE ***" );
    //ProcessSms( msg );
  }
 
  /*ClearGprsMsg();
    //EN: Always clear the flag
    SmsContentFlag = 0; */
}
