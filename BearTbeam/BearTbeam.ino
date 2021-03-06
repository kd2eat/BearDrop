/*
 * Beamer v0.1 - Mike Hojnowski / KD2EAT
 * 
 * This program drives a TTGO T-BEAM ESP32 board with an Si5351 riser board to provide
 * High Altitude Balloon APRS tracking, as well as a host of other useful functions.
 * 
 */
#include "beamer.h"   
#include "lorastructs.h"
#include "ESP32_Servo.h"

//#define DEBUG_UBLOX          // Normally commented out.  Used for debugging NAV setting for Flight Mode
//#define DEBUG_UBLOX_WAIT     // Normally commented out.  Used for debugging NAV setting for Flight Mode


//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hexdump.h"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
//#define RST     14   // GPIO14 -- SX1278's RESET
#define RST     23   // GPIO23 -- SX1278's RESET - TTGO T-Beam v1.1 is pin 23
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  915E6

//OLED pins
#define OLED_SDA PIN_OLED_SDA
#define OLED_SCL PIN_OLED_SCL 
#define OLED_RST -1       // Not defined on the TTGO T-Beam
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//package data
int BearDropped = 0; //initialize bear state
int dropSwitchState = 0;
int overrideSwitchState = 0;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

struct TelemetryData TelemetryData;
int16_t LastRSSI = 0;
float LastSNR = 0;
uint16_t  LastCommandReceived = 0;

TinyGPSPlus GPS;                            
HardwareSerial GPSSerial(1); 

bool  GpsEnabled = false;
long Bytes = 0;
long OldBytes = 0;
int MaxAltitude = 0;      // Largest Altitude value we've seen this flight
uint32_t LoRaMsgCount = 0;     // Keeping count of the number of messages sent
uint16_t  BadPacketsReceived = 0; 
boolean OneHzLed = LOW;
boolean TimeLed = LOW;
boolean BytesLedVal = LOW;

#ifdef CAMERA_SWITCHER
Servo myservo;
int ServoPos = DOWNCAMERA;    // Initialize with down facing camera
unsigned long ToggleCameraTime = 0;
unsigned long CameraForcedDownExpiration;
bool CameraForcedDown = false;
bool CameraForceDownCompleted = false;
#endif // CAMERA_SWITCHER

uint16_t temp_alt_counter = 0;
uint32_t burn_start_time = 0;
bool started_burn = false;

/**************************************************************************************************************/
bool
GoodFix()
{
    return( GPS.location.isValid() && 
            GPS.date.isValid() &&
            GPS.time.isValid() &&
            GPS.speed.isValid() &&
            GPS.course.isValid() &&
            GPS.altitude.isValid()&&
            GPS.satellites.isValid() &&
            GPS.hdop.isValid() &&
            (GPS.hdop.value() < 250.0)        // Based on scatter plot of sample data, anything under seems reasonable
            );
}

/**************************************************************************************************************/

void burnWire() { 
  Serial.println("In burnwire");
  if (BearDropped == 1) return;
  Serial.println("Turning on nichrome");
  digitalWrite(CUTDOWN, HIGH); //turn on nicrome cutter
  
  if (started_burn == false) { //if first time running burnWire()
    started_burn = true;
    burn_start_time = millis(); //get time when burnWire() first ran
  }
  if ((millis() - burn_start_time) >= 4000) { //if current time - time when burnWire() first ran >= 2000 ms
    Serial.println("Turning off nichrome");
    digitalWrite(CUTDOWN, LOW); //stop nicrome burning
    BearDropped = 1;   // Update global "Beardropped" variable.
    started_burn = false; //enable program to run cutdown again if error
  }
}

/**************************************************************************************************************/

void DropBear() {
  if (overrideSwitchState == 1) {  //override switch flipped 
    return; // exit function immediately 
  }
  //overrideSwitchState = 0 from here
  if ((GPS.altitude.meters() >=  DROPALTITUDE) && GoodFix()) { //check if drop altitude threshold is reached
    
    temp_alt_counter++;   //increment up counter, want several good readings in a row
  }
  if ((temp_alt_counter >= 5)  ||  (dropSwitchState == 1) || started_burn) {   //we have received five altitudes higher than the threshold
    burnWire();                                                            // OR manual switch is flipped ir we're already burning
    return;
  }
  return;
}

/**************************************************************************************************************/

void FixLoraChecksum(uint8_t *Message, int Length)
{
  int i;
  uint8_t CK_A, CK_B;

  CK_A = 0;
  CK_B = 0;

  for (i=0; i<(Length-2); i++)
  {
    CK_A = CK_A + Message[i];
    CK_B = CK_B + CK_A;
  }

  Message[Length-2] = CK_A;
  Message[Length-1] = CK_B;
}

/**************************************************************************************************************/
int LoraChecksumGood(uint8_t *Message, int Length)
{
  int i;
  uint8_t CK_A, CK_B;

  CK_A = 0;
  CK_B = 0;

  for (i=0; i<(Length-2); i++)
  {
    CK_A = CK_A + Message[i];
    CK_B = CK_B + CK_A;
  }

  if ((Message[Length-2] == CK_A) && (Message[Length-1] == CK_B)) {
    return 1;
  } else {
    return 0;
  }
}

/**************************************************************************************************************/
void
GpsSerialEnable()
{
  Serial.println("Enabling GPS");
  GPSSerial.begin(9600, SERIAL_8N1, 34, 12);      // Rev1 board pinout - Use "12, 15" or Rev 0.x boards.
  GpsEnabled = true;
}
/**************************************************************************************************************/
// Disable the serial port.  Bugs in the HAL layer cause delays up to 300uS while it's up, which can break AFSK.
void
GpsSerialDisable()
{
  Serial.println("Disabling GPS");
  GPSSerial.end();
  GpsEnabled = false;
}
/**************************************************************************************************************/
void FixUBXChecksum(uint8_t *Message, int Length)
{ 
  int i;
  uint8_t CK_A, CK_B;
  
  CK_A = 0;
  CK_B = 0;

  for (i=2; i<(Length-2); i++)
  {
    CK_A = CK_A + Message[i];
    CK_B = CK_B + CK_A;
  }
  
  if (Message[Length-2] != CK_A) {
     Serial.println("Checksum was incorrect.  Fixing CK_A");
     Message[Length-2] = CK_A;
  }
    if (Message[Length-1] != CK_B) {
     Serial.println("Checksum was incorrect.  Fixing CK_B");
     Message[Length-1] = CK_B;
  }
}
/**************************************************************************************************************/

void
GpsInit()
{
#ifdef DEBUG_UBLOX
  char  line[256];
  int i = 0;
  int linecount = 0;
#endif // DEBUG_UBLOX

  // Magic string of characters to set Airborne mode on the UBLOX GPS.
  // We only set the CFG-NAV5 "dynModel" section so that we can set the "Airborne" value to "<1g" (6).
  // u-blox 6 Receiver Reference. pp2 && pp119.  All other values in this block were the "default" values
  // when the GPS was booted cold.

   uint8_t SetCfgNav5A[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x01, // Note two 0x01's.  Don't know if the bitmask is little-endian or not.
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, // Set dyn mode to 6 - airborne <1g.  That's plenty for balloons.
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, // Note, other code has 0x00 where the T-Beam defaults to 0x3c.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x56, 0x76
    };

  // Magic string of characters to set Pedestrian mode on the UBLOX GPS.
  // We only set the CFG-NAV5 "dynModel" section so that we can set the "Portable" value (0).
  // u-blox 6 Receiver Reference. pp2 && pp119.  All other values in this block were the "default" values
  // when the GPS was booted cold.

   uint8_t SetCfgNav5P[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x01, // Note two 0x01's.  Don't know if the bitmask is little-endian or not.
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, // Set dyn mode to 0 - Portable
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, // Note, other code has 0x00 where the T-Beam defaults to 0x3c.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x56, 0x76
    };
/*
  const unsigned char SetCfgNav5A[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, // Note two 0x01's.  Don't know if the bitmask is little-endian or not.
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, // Set dyn mode to 6 - airborne <1g.  That's plenty for balloons.
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, // Note, other code has 0x00 where the T-Beam defaults to 0x3c.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x16, 0xdc
    };
*/
     uint8_t CheckNav[] =
    {
        0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84
    };
  
 
  // Use code borrowed from FlexTrack to recalculate checksums, so we can fiddle with the strings more easily
  FixUBXChecksum(&SetCfgNav5A[0], sizeof(SetCfgNav5A));
  FixUBXChecksum(&CheckNav[0], sizeof(CheckNav));
   
  GpsSerialEnable();

#ifdef DEBUG_UBLOX
  delay(300);

  // Confirm the setting before we start
  Serial.println("Sending first query");
  if (GPSSerial.write( (uint8_t *)&CheckNav, sizeof(CheckNav)) != sizeof(CheckNav)) {
    Serial.println("ERROR: Sending first query.");
  }
  GPSSerial.flush();
#endif //DEBUG_UBLOX


  delay(300);
  Serial.println("Sending GPS init string (Airborne mode)");
  // Set dynamic mode (Airborne).  Required to allow flights over 12000 meters.
  if (GPSSerial.write( (uint8_t *)&SetCfgNav5A, sizeof(SetCfgNav5A)) != sizeof(SetCfgNav5A)) {
    Serial.println("ERROR: Init string");
  }    
  GPSSerial.flush();


#ifdef DEBUG_UBLOX
  delay(300);
  Serial.println("Sending second query");

  if (GPSSerial.write( (uint8_t *)&CheckNav, sizeof(CheckNav)) != sizeof(CheckNav)) {
    Serial.println("ERROR: Sending second query.");
  }
  GPSSerial.flush();
#ifdef DEBUG_UBLOX_WAIT
  while (1) {
    if (GPSSerial.available()) {
      char c = GPSSerial.read();
      if ( (c == '\r') || (c == '\n')) {
        line[i] = '\0';
        DumpHex(&line, i);
        i = 0;
        if (++linecount > 80) {
          while (1);  // Grind to a halt.  We should have all the data
        }
      } else {
        line[i++] = c;
      }
    } 
  }
#endif // DEBUG_UBLOX_WAIT
#endif // DEBUG_UBLOX 
}

/**************************************************************************************************************/

void
OledInit() {
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.display();
}
/*************************************************************************************************************/
void
OledUpdate() {
  char line0[22];
  char line1[22];
  char line2[22];
  char line3[22];
  char line4[22];
  char line5[22];
  char line6[22];
  char line7[22];
  struct TelemetryData OledData = TelemetryData;  
  float TempLat = OledData.Lat / 1E6;
  float TempLng = OledData.Lng / 1E6;

  sprintf(line0,"%02d-%02d-%04d %02d:%02d:%02d", OledData.Month, OledData.Day, OledData.Year,
        OledData.Hour, OledData.Minute, OledData.Second );
  sprintf(line1, "%3.5f  %3.5f", TempLat, TempLng);
  sprintf(line2, "A %5d (%5d) S %2d", OledData.Altitude, OledData.MaxAltitude, OledData.Satellites);
  sprintf(line3, "Msgs: %5d Bad: %3d" , OledData.MsgCount, OledData.BadPacketsReceived);
  sprintf(line4, "tR: %3d tS: %2.1f", OledData.TrackerRSSI, OledData.TrackerSNR);
  sprintf(line5, "LastCmd: %d", OledData.LastCommandReceived);
  sprintf(line6, "Bytes: %d", Bytes);
  sprintf(line7, "rR: %3d rS: %2.1f", LastRSSI, LastSNR);

  display.clearDisplay();
  display.setCursor(0,0*8); display.print(line0);
  display.setCursor(0,1*8); display.print(line1);
  display.setCursor(0,2*8); display.print(line2);
  display.setCursor(0,3*8); display.print(line3);
  display.setCursor(0,4*8); display.print(line4);
  display.setCursor(0,5*8); display.print(line5);
  display.setCursor(0,6*8); display.print(line6);
  display.setCursor(0,7*8); display.print(line7);
  display.invertDisplay( (OledData.Satellites < 5) );    // Invert display if we have no or poor lock
  display.display();
}
/*************************************************************************************************************/
void
OledDie(char *msg) {

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0*16); display.print(msg);
  display.invertDisplay(true);
  display.display();
  while (1) ;   // Hang here
}
/**************************************************************************************************************/

void LoRaInit() {
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    OledDie("LoRa FAIL");   // Won't return
  }
  LoRa.setSpreadingFactor(8);    // Max 976bps, which is plenty for our needs
  Serial.println("LoRa Initializing OK!");
}
/**************************************************************************************************************/
void LoRaSend() {
  TelemetryData.DestinationAddr = LORA_GROUND_STATION;
  TelemetryData.SourceAddr = LORA_TRACKER;
  TelemetryData.TrackerRSSI = LastRSSI;
  TelemetryData.TrackerSNR = LastSNR;
  TelemetryData.MsgCount = LoRaMsgCount;
  TelemetryData.Hour = GPS.time.hour();
  TelemetryData.Minute = GPS.time.minute();
  TelemetryData.Second = GPS.time.second();
  TelemetryData.Year = GPS.date.year();
  TelemetryData.Month = GPS.date.month();
  TelemetryData.Day = GPS.date.day();
  TelemetryData.Satellites = GPS.satellites.value();
  TelemetryData.Altitude = GPS.altitude.meters();
  TelemetryData.MaxAltitude = MaxAltitude;
  TelemetryData.Lat = GPS.location.lat() * 1E6;
  TelemetryData.Lng = GPS.location.lng() * 1E6;
  TelemetryData.Hdop = GPS.hdop.value() * 1E6;
  TelemetryData.GoodFix = (uint8_t) GoodFix();
  TelemetryData.LastCommandReceived = LastCommandReceived;
  TelemetryData.BearDropped = BearDropped;    // Set telemetry value based on the global variable
  TelemetryData.OverrideReceived = overrideSwitchState; 
  FixLoraChecksum((uint8_t *) &TelemetryData, sizeof(TelemetryData));  

  
  LoRa.beginPacket();                   // start packet
  LoRa.write((uint8_t *)&TelemetryData, sizeof(TelemetryData));
  LoRa.endPacket();                     // finish packet and send it
  LoRaMsgCount++;                           // increment message ID 

}
/**************************************************************************************************************/
#ifdef CAMERA_SWITCHER
void
SwitchCameraView() {
  if (!CameraForcedDown) {
    if (ServoPos == DOWNCAMERA) {
      Serial.println("Pointing camera to the side.");
      ServoPos = SIDECAMERA;
      ToggleCameraTime = millis() + SIDE_CAMERA_TIME*1000;
    } else {
      Serial.println("Pointing camera down.");
      ServoPos = DOWNCAMERA;
      ToggleCameraTime = millis() + DOWN_CAMERA_TIME*1000;
    }
    myservo.write(ServoPos); 
  } else {
    // If the camera was forced down, just kick the timer down the road another second,
    // but don't mess with the camera.
    ToggleCameraTime = millis() + 1000;   
  }
}
#endif // CAMERA_SWITCHER
/**************************************************************************************************************/
void LoRaReceive(int PacketSize) {
  struct TeleCommand  Packet;
  char  *c = (char *) &Packet;
  
  if (PacketSize <= 0) return;      // Nothing to see here, move along
  Serial.print("Received packet "); Serial.print(PacketSize); Serial.println(" bytes long.");
  if (PacketSize != sizeof(TeleCommand)) {
    Serial.println("Skipping malformed packet");
    BadPacketsReceived++;
    for (int i = 0 ; i < PacketSize; i++) {
      char ignored = LoRa.read();     // Gobble up malformed packet
    }
    return;     // Don't do any further processing on bad packets
  } else {
    for (int i = 0 ; i < PacketSize; i++) {
      *c++ = LoRa.read();   // Load packet data into the structure
    }
  }
  if (!LoraChecksumGood((uint8_t *) &Packet, sizeof(Packet))) {
      Serial.println("Bad Packet Checksum.  Skipping.");
      return;
  }
  LastRSSI = LoRa.packetRssi();
  LastSNR = LoRa.packetSnr();

  if ((Packet.DestinationAddr != LORA_TRACKER) && (Packet.DestinationAddr != LORA_BROADCAST)) {
    Serial.println("Skipping packet that's not for me.");
    return;   // Packet is not for us
  } 
  LastCommandReceived = Packet.CommandCount;
  overrideSwitchState = Packet.overrideSwitchState;    // Set our global variable based on the value from the ground station
  dropSwitchState = Packet.dropSwitchState; 
  
  // If we got this far, we got a decent command.  Toggle the camera position.
#ifdef CAMERA_SWITCHER
  SwitchCameraView();
#endif // CAMERA_SWITCHER
  OledUpdate();
}

/**************************************************************************************************************/

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Moat 1.1");
  delay(2000);
  pinMode(CUTDOWN, OUTPUT);
  digitalWrite(CUTDOWN, LOW); //initialize CUTDOWN nicrome to LOW (don't drop bear)
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, OneHzLed);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_TIME, OUTPUT);    // This LED will be set high when the time is non-zero
  digitalWrite(LED_TIME, LOW);
  pinMode(LED_BYTESIN, OUTPUT); // We'll flash this LED if we are getting data from the GPS
  digitalWrite(LED_BYTESIN, LOW);

  
#ifdef OLED
  OledInit();
#endif //OLED
  GpsInit();

#ifdef LORA
  LoRaInit();
#endif // LORA
#ifdef CAMERA_SWITCHER
   myservo.attach(PIN_SERVO);
   myservo.write(ServoPos);
   ToggleCameraTime = millis() + DOWN_CAMERA_TIME*1000; 
#endif // CAMERA_SWITCHER
}
/**************************************************************************************************************/
unsigned long  NextSecond = 0;    // Start with a bogus LastSecond value so that we pass through the loop the first time
unsigned long NextTimeToSend = 0;

void loop() {
  
#ifdef LORA
  LoRaReceive(LoRa.parsePacket());
#endif // LORA
  if (GpsEnabled && GPSSerial.available()) {
    char c = GPSSerial.read();
    //Serial.print(c);
    GPS.encode(c);
    Bytes++;
  }
#ifdef CAMERA_SWITCHER
  // We force over to the downward facing camera around the time of the payload release, and stay there.
  if ((GPS.altitude.meters() > DOWN_CAMERA_FORCE_ALTITUDE) &&
      (!CameraForcedDown) && (!CameraForceDownCompleted) ) {
        ServoPos = DOWNCAMERA;
        myservo.write(ServoPos); 
        CameraForcedDown = true;
        CameraForcedDownExpiration = millis() + (DOWN_CAMERA_FORCE_MAX_MINUTES * 60 * 1000);
  }
  // If the camera is forced down, and we've exceeded the max "release" altitude, go ahead and unforce it.
  if ((GPS.altitude.meters() > DOWN_CAMERA_UNFORCE_ALTITUDE) &&
      (CameraForcedDown) && (!CameraForceDownCompleted) ) {
        CameraForceDownCompleted = true;
        CameraForcedDown = false;
  }
  // Override camera force down "completed" if we've done it for too long.  Maybe the GPS didn't work, or maybe
  // we burst before the "unforce" altitude
  if ((CameraForcedDown) && (millis() > CameraForcedDownExpiration)) {
    CameraForceDownCompleted = true;
    CameraForcedDown = false;
  }

  // Check to see if it's time to change the camera position
  if (millis() > ToggleCameraTime) {
    SwitchCameraView();
  }
#endif // CAMERA_SWITCHER 

  if ((millis() >= NextSecond) ) {
    OneHzLed = !OneHzLed;  digitalWrite(LED_GREEN, OneHzLed);     // Blink LED
    if (Bytes != OldBytes) {
      BytesLedVal = !BytesLedVal; digitalWrite(LED_BYTESIN, BytesLedVal);     // Blink LED
      OldBytes = Bytes;
    }
    if ((GPS.time.hour() + GPS.time.minute() + GPS.time.second()) == 0) {   // If we have time, set LED high
      digitalWrite(LED_TIME, LOW);
      //Serial.println("TIME = LOW");
    } else {
      if (GPS.satellites.value() > 4)  {    // If we have good sats - leave light solid
          //Serial.println("TIME = HIGH");
          digitalWrite(LED_TIME, HIGH);      // Time and sats - turn blue on solid
      } else {
         TimeLed = !TimeLed;
         //Serial.println("TIME = Blink");
         digitalWrite(LED_TIME, TimeLed);      // Time but no sats - blink  blue
      }
    }
    DropBear();
    NextSecond = millis() + 1000;
    
    if (NextSecond > 5000 && GPS.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
    }

#ifdef OLED
     OledUpdate();
#endif // OLED
  }   // Once per second loop 
  
#ifdef LORA  
  // Special once per "second" loop for LoRa.  If the GPS isn't ready at the top of the second, we keep retrying each
  // time through the main loop, since we're consuming more characters from the GPS above.  We will give it the totally abritray 
  // amount of "50 milliseconds" (about 60 characters at 9600 baud) to get a decent fix.  If we don't have one by then, we beacon anyway
  // and the status in the telemetry data will indicate that it's a bad fix.
  
  if (millis() > NextTimeToSend) {  //  
      if (GoodFix() || (millis() > NextTimeToSend + 50)) {   // Either it's good, or we've waited too long
        if (GoodFix() && (GPS.altitude.meters() > MaxAltitude)) {
          MaxAltitude = GPS.altitude.meters();
        }
        LoRaSend();
        NextTimeToSend += 1000;

        // Write some debugs to the console
        Serial.print(GPS.location.lat(), 5);  Serial.print("\t : ");    Serial.println(GPS.location.lng(), 4);
        Serial.print("Alt  : ");    Serial.print(GPS.altitude.meters());    Serial.print("\tSats: ");    Serial.print(GPS.satellites.value());
          Serial.print("\t");    Serial.print("Time      : ");   Serial.print(GPS.time.hour());    Serial.print(":");
          Serial.print(GPS.time.minute()); Serial.print(":"); Serial.println(GPS.time.second());
        Serial.print("Bytes: "); Serial.println(Bytes); Serial.println("");
      }
  }
#endif // LORA
}  // Main loop

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    if (GpsEnabled && GPSSerial.available()) {
      GPS.encode(GPSSerial.read());
      Bytes++;
    }
  } while (millis() - start < ms);
}
