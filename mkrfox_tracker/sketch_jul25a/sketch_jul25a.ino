#include "TinyGPS++.h"
#include <ArduinoLowPower.h>
#include <SigFox.h>
#include <string.h>
#include <stdio.h>

TinyGPSPlus gps;
#define RX_PIN 8
#define TX_PIN 9
#define GPS_PIN 6
#define DEBUG true
#define DEBUG true
#define bpPin A5
#define WAITING_TIME 15
#define bodyTempPin A6
int buzzerPin = 5,greenLedPin = 6,blueLedPin = 7,redLedPin = 8;
float internalTemp,bodyTemperature,bloodPressure;

String Data_SMS;
char Received_SMS; 
short LOCATE_OK=-1; 

struct gpscoordinates{
  float a_latitude;
  float a_longitude;
  float a_altitude;
};

float latitude = 0.0f;
float longitude = 0.0f;
float altitud = 0.0f;

bool debug = false;

void sendSigfoxData(String data){
  if(debug){
    Serial.print("Sending: ");Serial.println(data);
    if(data.length()>12){
      Serial.println("Message too long,only first 12 bytes will be sent");
      data.trim();
    }
  }
  //start sigfox module
  SigFox.begin();
  delay(100);
  SigFox.status();
  delay(1);
  if(debug) SigFox.debug();
  delay(100);
  //write packet
  SigFox.beginPacket();
  SigFox.print(data);

  if(debug){
    int retreival = SigFox.endPacket(true);
    if(retreival > 0){
      Serial.println("No transmission");
    }else{
      Serial.println("Transmission ok");
    }
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
    if(SigFox.parsePacket()){
      Serial.println("Response from server: ");
      while(SigFox.available()){
        Serial.print("0x");
        Serial.println(SigFox.read(),HEX);
      }
    }else{
      Serial.println("Could not get any response from the server");
      Serial.println("Check the SigFox coverage in your area");
      Serial.println("If you are indoor, check the 20dB coverage or move near a window");
    }
  }else{
    SigFox.endPacket();
  }
  SigFox.end();
}

String sendCommand(String command,int timeout,boolean debug){
  String response = "";
  Serial1.println(command);
  long int mTime = millis();
  while((mTime +timeout)>millis()){
    while(Serial1.available()){
      char c = Serial1.read();
      response +=c;
    }
  }
  if(debug){
    SerialUSB.print(response);
  }
  return response;
}

bool moduleState(){
  int i = 0;
  bool state = false;
  for(i = 0;i < 10;i++){
    String msg = String("");
    msg = sendCommand("AT",1000,DEBUG);
    if(msg.indexOf("OK")>=0){
      SerialUSB.println("A9G module turned on");
      state = true;
      return state;
    }
    delay(500);
  }
  return state;
}

void serialCommunication(){
  while (Serial1.available() > 0) {
    SerialUSB.write(Serial1.read());
    yield();
  }
  while (SerialUSB.available() > 0) {
    Serial1.write(SerialUSB.read());
    yield();
  }
}
void receiveMode(){
  String state = sendCommand("AT",1000,DEBUG);
  //check state of module
  if(state.indexOf("OK")){
    serialCommunication();
  }
  String mode = sendCommand("AT+CMGF=1",1000,DEBUG);
  if(mode.indexOf("OK")){
    serialCommunication();
  }
  //listen mode
  String listen = sendCommand("AT+CNMI=2,2,0,0,0",1000,DEBUG);
  serialCommunication();
}
void Send_Data(){
  Serial.println("Sending Data...");     //Displays on the serial monitor...Optional
  sendCommand("AT+CMGF=1\r",1000,DEBUG);          //Set the module to SMS mode
  delay(100);
  sendCommand("AT+CMGS=\"+***********\"\r",2000,DEBUG);  //Your phone number don't forget to include your country code example +212xxxxxxxxx"
  delay(500);  
  sendCommand(Data_SMS,1000,DEBUG);  //This string is sent as SMS
  delay(500);
  sendCommand("26",1000,DEBUG);//Required to tell the module that it can send the SMS
  delay(500);
  sendCommand("",500,DEBUG);
  Serial.println("Data Sent.");
  delay(500);
}

float roundOff(float val){
  float value = (int)(val * 10 +.5);
  return (float)value/10;
}
void readBodyTemp(){
  bodyTemperature = analogRead(bodyTempPin);
  bodyTemperature = (500 * bodyTemperature) /1024;
  Serial.print("Body temperature: ");Serial.print(bodyTemperature);
  String bodyTemp = "Temp: "+(String)roundOff(bodyTemperature);
  sendSigfoxData((String)bodyTemperature);
}
void readBloodPressure(){
 bloodPressure = analogRead(bpPin);
 Serial.println("Blood pressure: "+(String)bloodPressure);
 String bp = "BP: "+(String)roundOff(bloodPressure);
 sendSigfoxData(bp);
}
void setup() {
  Serial.println("Booting......");
  Serial.println("Welcome");
  pinMode(greenLedPin,OUTPUT);
  pinMode(blueLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  if(debug){
      Serial.begin(115200);
      while(!Serial){
        //wait for serial to connect
      }
      Serial.println("Serial connected");
  }
  Serial1.begin(115200);
  SerialUSB.begin(115200);
  while(!Serial){}

  bool modulestate = moduleState();
  if(modulestate==false){
    Serial.print("Module is off or not receiving power");
  }else{
    Serial.println("Module is on");
  }

  //turn on GPS
  sendCommand("AT+GPS=1",1000,DEBUG);
  //read gps after 10 seconds
  sendCommand("AT+GPSRD=10",1000,DEBUG);
  /**
   * 1.Get base addresss
   * 2.Get gps address
   */
   sendCommand("AT+LOCATION=2",1000,DEBUG);

  if(debug){
    Serial.println("GPS Connected");
  }
  
  pinMode(GPS_PIN,OUTPUT);

  if(!SigFox.begin()){
    Serial.println("Sheild error or not present");
    return;
  }
    if(debug){
      SigFox.debug();String version = SigFox.SigVersion();
      String ID = SigFox.ID();
      String PAC = SigFox.PAC();
      //Display module information
      Serial.println("First configuration");
      Serial.println("SigFox FW version "+version);
      Serial.println("ID = "+ID);
      Serial.println("PAC = "+PAC);
      Serial.println();
      delay(100);
    }else{
      SigFox.end();
    }
     receiveMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  String loc = sendCommand("AT+LOCATION=2",1000,DEBUG);
  serialCommunication();
  while (loc > 0)
   gps.encode(Serial1.read());
 if (gps.location.isUpdated()){
  latitude = roundOff(gps.location.lat());
  longitude = roundOff(gps.location.lng());
  altitud = roundOff(gps.altitude.meters());
   Serial.print(F("  Lat="));
   Serial.print(latitude);
   Serial.print(F(" Long="));
   Serial.println(longitude);
   Serial.println(F(" Altitude="));
   Serial.println(altitud);
 }
 readBodyTemp();
 readBloodPressure();
 String RSMS;             //We add this new variable String type, and we put it in loop so everytime gets initialized
                           //This is where we put the Received SMS, yes above there's Recevied_SMS variable, we use a trick below
                           //To concatenate the "char Recevied_SMS" to "String RSMS" which makes the "RSMS" contains the SMS received but as a String
                           //The recevied SMS cannot be stored directly as String
  

while(Serial1.available()>0){       //When SIM800L sends something to the Arduino... problably the SMS received... if something else it's not a problem
        
        Received_SMS=Serial1.read();  //"char Received_SMS" is now containing the full SMS received
        Serial.print(Received_SMS);   //Show it on the serial monitor (optional)     
        RSMS.concat(Received_SMS);    //concatenate "char received_SMS" to RSMS which is "empty"
        LOCATE_OK=RSMS.indexOf("LOCATE");   //And this is why we changed from char to String, it's to be able to use this function "indexOf"
                                      //"indexOf function looks for the substring "x" within the String (here RSMS) and gives us its index or position
                                      //For example if found at the beginning it will give "0" after 1 character it will be "1"
                                      //If it's not found it will give "-1", so the variables are integers
        
    }
    
  if(LOCATE_OK!=-1){                         //If "DHT" word is found within the SMS, it means that DHT_OK have other value than -1 so we can proceed
    Serial.println("LOCATE");          //Shows on the serial monitor "found DHT" (optional)
    Serial.print("Temperature = "); Serial.print(bodyTemperature); Serial.print("*C"); 
                                          //Show it on the serial monitor also optional

    Data_SMS = "Body\nTemp = "+String(bodyTemperature,2)+" C"+" \nBlood Pressure ="+String(bloodPressure,2)+" %";       //Prepare the SMS to send, it contains some strings like "DHT" "Temperature"...
                                                                                         //And then the values read
    
    Send_Data();                     //This function set the sending SMS mode, prepare the phone number to which we gonna send, and send "Data_SMS" String
    receiveMode();                   //Come back to Receving SMS mode and wait for other SMS
    
    LOCATE_OK=-1;                   //If the DHT is found the variable should be reset to -1 otherwise it will be kept as !=-1 and will send SMS over and over
                                    //Maybe not required... I did a lot of tests and maybe at the beginning the RSMS string kept concating and MLX word was kept there
                                    //And at this point I'm too lazy to reupload the code without it and test...
  }

}
