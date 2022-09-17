// Ground station code to receive LoRa telemetry and send Project Horus "Payload Summary" packets via UDP
// Build code for board: "TTGO LoRa32-OLED V1"

#include "config.h"
#include "lorastructs.h"

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>


//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DI0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 915E6 


//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
WiFiMulti wifiMulti;
WiFiUDP udp;
uint8_t CurrentWiFiStatus = WL_DISCONNECTED;    // Start out disconnected

struct TelemetryData TelemetryData;
int16_t LastRSSI = 0;
float LastSNR = 0;
uint32_t  PacketsReceived = 0;    // Keep track of packets we receive
uint32_t  LastPacketForUDP = 0;   // Track the last packet we used for UDP telemetry
uint16_t TeleCommandCount = 0;   // Number of commands we've sent to the payload
uint16_t BadPacketCountGnd = 0;   // Bad packets the ground station has received
uint32_t  LastTbeamPacket = 0;   // We update this timestamp every time we receive a packet from the T-Beam.
bool  OledStale = false;

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
LEDupdate() {

   if (TelemetryData.BearDropped) { 
    digitalWrite(LED_DROP, HIGH);
   } else {
    digitalWrite(LED_DROP, LOW);
   }
   if (TelemetryData.OverrideReceived) {
    digitalWrite(LED_SAFETY, HIGH);
   } else {
    digitalWrite(LED_SAFETY, LOW);
   }
   if (TelemetryData.ParachuteDropped) {
    digitalWrite(LED_PARACHUTE, HIGH);
   } else {
    digitalWrite(LED_PARACHUTE, LOW);
   }
   if (TelemetryData.BuzzerReceived) {
    digitalWrite(LED_BUZZER, HIGH);
   } else {
    digitalWrite(LED_BUZZER, LOW);
   }
   if (CurrentWiFiStatus != WL_CONNECTED ){ //no wifi connection?
    digitalWrite(LED_WIFI_STATUS, HIGH);
   } else {
    digitalWrite(LED_WIFI_STATUS, LOW);
   }

   // haven't received a packet in a while?
  if ( millis() - LastTbeamPacket >= 3000 ){ // current time - last time packet was received
    digitalWrite(LED_WARNING, HIGH); //turn on warning LED
    OledStale = true;
  } else {
    digitalWrite(LED_WARNING, LOW);
    OledStale = false;
  }
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
  sprintf(line5, "IP: %d\.%d\.%d\.%d", WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],WiFi.localIP()[3] );
  sprintf(line6, "S/A: %4d/%4d", TeleCommandCount, OledData.LastCommandReceived);
  sprintf(line7, "BadM: %2d  Temp: %2.1f", BadPacketCountGnd, (float) OledData.TempTenths/10);

  display.clearDisplay();
  display.setCursor(0,0*8); display.print(line0);
  display.setCursor(0,1*8); display.print(line1);
  display.setCursor(0,2*8); display.print(line2);
  display.setCursor(0,3*8); display.print(line3);
  display.setCursor(0,4*8); display.print(line4);
  display.setCursor(0,5*8); display.print(line5);
  display.setCursor(0,6*8); display.print(line6);
  display.setCursor(0,7*8); display.print(line7);
  display.invertDisplay( ((OledData.Satellites < 5) ) || OledStale);    // Invert display if we have no or poor lock
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
// XXX Todo: Set unique starting position
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
/*************************************************************************************************************/

void LoRaSend() {
  struct TeleCommand Temp;

#ifndef READONLY_STATION
  TeleCommandCount++; 
  Temp.DestinationAddr = LORA_TRACKER;
  Temp.SourceAddr = LORA_GROUND_STATION;
  Temp.CommandCount = TeleCommandCount;
  Temp.dropSwitchState = digitalRead(DROP_SWITCH);
  Temp.overrideSwitchState = digitalRead(SAFETY_SWITCH);
  Temp.parachuteSwitchState = digitalRead(SWITCH_PARACHUTE);
  Temp.buzzerSwitchState = !digitalRead(SWITCH_BUZZER);
  FixLoraChecksum((uint8_t *) &Temp, sizeof(Temp));  
   
  LoRa.beginPacket();                   // start packet
  LoRa.write((uint8_t *)&Temp, sizeof(Temp));
  LoRa.endPacket();                     // finish packet and send it

  Serial.print("Override switch set to: "); Serial.print(Temp.overrideSwitchState); 
  Serial.print(".  Buzzer switch set to: "); Serial.print(Temp.buzzerSwitchState);  
  Serial.print(".  Parachute switch set to: "); Serial.print(Temp.parachuteSwitchState);   
  Serial.print(".  Drop switch set to: "); Serial.println(Temp.dropSwitchState);

#else // READONLY_GROUNDSTATION
  //Serial.println("Read only ground station.  Bypassing LoRaSend().");
#endif // READONLY Ground Station
  // XXX - Todo: Log that we sent the packet via UDP
}

/**************************************************************************************************************/
void LoRaReceive(int PacketSize) {
  struct TelemetryData  Packet;
  char  *c = (char *) &Packet;
  
  if (PacketSize <= 0) return;      // Nothing to see here, move along
  // Serial.print("Received packet "); Serial.print(PacketSize); Serial.println(" bytes long.");
  if (PacketSize != sizeof(TelemetryData)) {
    Serial.println("Skipping malformed packet");
    BadPacketCountGnd++;
    for (int i = 0 ; i < PacketSize; i++) {
      char ignored = LoRa.read();     // Gobble up malformed packet
    }
    return;     // Don't do any further processing
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

  if ((Packet.DestinationAddr != LORA_GROUND_STATION) && (Packet.DestinationAddr != LORA_BROADCAST)) {
    Serial.println("Skipping packet that's not for me.");
    return;   // Packet is not for us
  } else { // This packet is for us
    TelemetryData = Packet;
    //OledUpdate();
    PacketsReceived++;
    if (Packet.LastCommandReceived == TeleCommandCount) {
      // Tracker acknowledges our packet
      //Serial.println("Tracker acknowledged packet.");
    }
    SendUDPlog();
    LastTbeamPacket = millis();
  }
}
/**************************************************************************************************************/
void WiFiInit() {
    wifiMulti.addAP(SSID1, PASSWORD1);
    wifiMulti.addAP(SSID2, PASSWORD2);
}
/**************************************************************************************************************/
// Send log data via UDP.  
// Note, we don't bother to set up a listener with udp.begin() since we're not planning to receive.
// udp.beginPacket() / endPacket() allocate one buffer and recycle it forever, so it shouldn't leak memory if we re-bind to
// a new IP address.
void
SendUDPlog() {
  char  templine[12];
  int DropSwitchState = digitalRead(DROP_SWITCH);
  int OverrideSwitchState = digitalRead(SAFETY_SWITCH);
  int BuzzerSwitchState = !digitalRead(SWITCH_BUZZER);
  int ParachuteSwitchState = digitalRead(SWITCH_PARACHUTE);
  // We've received more packet since the last time we sent telemetry.  Proceed.
  if (CurrentWiFiStatus != WL_CONNECTED) {
    Serial.println("UDP Log Send aborted.  WiFi not connected");
    return;   // If not connected to WiFi, just punt.  We'll try again next time.
  }
  
  IPAddress BroadcastAddr = WiFi.localIP();
  BroadcastAddr[3] = (uint8_t) 0xff;    // XXX  - make this smarter - setting last octet to broadcast only works on /24 net.
  //Serial.print("Sending a UDP Log packet to "); Serial.print(BroadcastAddr); Serial.print(":"); Serial.println(UDP_LOGGING_PORT);
  udp.beginPacket(BroadcastAddr, UDP_LOGGING_PORT);
  // XXX - needs work for payload summary
  udp.print("{");
  udp.print("\"log_type\": \"lora_receive\"");
  udp.print(", \"DestinationAddr\": \""); udp.print(TelemetryData.DestinationAddr); udp.print("\"");
  udp.print(", \"SourceAddr\": \""); udp.print(TelemetryData.SourceAddr); udp.print("\"");
  udp.print(", \"TrackerRSSI\": \""); udp.print(TelemetryData.TrackerRSSI); udp.print("\"");
  udp.print(", \"TrackerSNR\": \""); udp.print(TelemetryData.TrackerSNR); udp.print("\"");
  udp.print(", \"MsgCount\": \""); udp.print(TelemetryData.MsgCount); udp.print("\"");
  udp.print(", \"Hour\": \""); udp.print(TelemetryData.Hour); udp.print("\"");
  udp.print(", \"Minute\": \""); udp.print(TelemetryData.Minute); udp.print("\"");
  udp.print(", \"Second\": \""); udp.print(TelemetryData.Second); udp.print("\"");
  udp.print(", \"Month\": \""); udp.print(TelemetryData.Month); udp.print("\"");
  udp.print(", \"Day\": \""); udp.print(TelemetryData.Day); udp.print("\"");
  udp.print(", \"Year\": \""); udp.print(TelemetryData.Year); udp.print("\"");
  udp.print(", \"Satellites\": \""); udp.print(TelemetryData.Satellites); udp.print("\"");
  udp.print(", \"Altitude\": \""); udp.print(TelemetryData.Altitude); udp.print("\"");
  udp.print(", \"MaxAltitude\": \""); udp.print(TelemetryData.MaxAltitude); udp.print("\"");
  udp.print(", \"GoodFix\": \""); udp.print(TelemetryData.GoodFix); udp.print("\"");

  float temp = TelemetryData.Lat / 1E6;
  sprintf(templine,"%3.5f", temp);
  udp.print(", \"Lat\": \""); udp.print(templine); udp.print("\"");
  temp = TelemetryData.Lng / 1E6;
  sprintf(templine,"%3.5f", temp);
  udp.print(", \"Lng\": \""); udp.print(templine); udp.print("\"");

  
  temp = TelemetryData.Hdop / 1E6;
  sprintf(templine,"%3.5f", temp);
  udp.print(", \"Hdop\": \""); udp.print(templine); udp.print("\"");

  
  udp.print(", \"LastCommandReceived\": \""); udp.print(TelemetryData.LastCommandReceived); udp.print("\"");
  udp.print(", \"BadPacketsReceived\": \""); udp.print(TelemetryData.BadPacketsReceived); udp.print("\"");

  udp.print(", \"LastCommandSent\": \""); udp.print(TeleCommandCount); udp.print("\"");
  udp.print(", \"GroundBadPackets\": \""); udp.print(BadPacketCountGnd); udp.print("\"");
  udp.print(", \"GroundRSSI\": \""); udp.print(LastRSSI); udp.print("\"");

  udp.print(", \"GroundSNR\": \""); udp.print(LastSNR); udp.print("\"");
  
  udp.print(", \"BearDropped\": \""); udp.print(TelemetryData.BearDropped); udp.print("\"");
  udp.print(", \"OverrideReceived\": \""); udp.print(TelemetryData.OverrideReceived); udp.print("\"");
  udp.print(", \"DropSwitch\": \""); udp.print(DropSwitchState); udp.print("\"");
  udp.print(", \"LockoutSwitch\": \""); udp.print(OverrideSwitchState); udp.print("\"");
  
  udp.print(", \"ParachuteDropped\": \""); udp.print(TelemetryData.ParachuteDropped); udp.print("\"");
  udp.print(", \"BuzzerReceived\": \""); udp.print(TelemetryData.BuzzerReceived); udp.print("\"");
  udp.print(", \"ParachuteSwitch\": \""); udp.print(ParachuteSwitchState); udp.print("\"");
  udp.print(", \"BuzzerSwitch\": \""); udp.print(BuzzerSwitchState); udp.print("\"");
                
  temp = ((float) TelemetryData.Millivolts / 1000.0);
  sprintf(templine,"%2.1f", temp);
  udp.print(", \"Voltage\": \""); udp.print(templine); udp.print("\"");
  temp = ((float) TelemetryData.TempTenths / 10.0);
  sprintf(templine,"%2.1f", temp);
  udp.print(", \"Temperature\": \""); udp.print(templine); udp.print("\"");  udp.println("}");
  udp.endPacket();
}
/**************************************************************************************************************/// Send UDP packet in Project Horus "Payload Summary" format.  
// Note, we don't bother to set up a listener with udp.begin() since we're not planning to receive.
// udp.beginPacket() / endPacket() allocate one buffer and recycle it forever, so it shouldn't leak memory if we re-bind to
// a new IP address.
boolean
SendPayloadSummary() {
  char  templine[12];
  if (PacketsReceived != LastPacketForUDP) {
    // We've received more packets since the last time we sent telemetry.  Proceed.
    if (CurrentWiFiStatus != WL_CONNECTED) {
      Serial.println("UDP Send aborted.  WiFi not connected");
      return false;   // If not connected to WiFi, just punt.  We'll try again next time.
    }

    if (!TelemetryData.GoodFix ) {
      // We we don't have a good fix reported from the T-Beam, don't send a payload summary.
      return false;
    }
    IPAddress BroadcastAddr = WiFi.localIP();
    BroadcastAddr[3] = (uint8_t) 0xff;    // XXX  - make this smarter - setting last octet to broadcast only works on /24 net.
    Serial.print("Sending a PAYLOAD_SUMMARY packet to "); Serial.print(BroadcastAddr);
    Serial.print(":"); Serial.println(PAYLOAD_SUMMARY_PORT);
    udp.beginPacket(BroadcastAddr, PAYLOAD_SUMMARY_PORT);    
    udp.print("{");
    udp.print("\"comment\": \"HAB\"");
    udp.print(", \"frame\": "); udp.print(TelemetryData.MsgCount); 
    udp.print(", \"callsign\": \""); udp.print(LORA_2_PAYLOAD_CALLSIGN); udp.print("\"");
    udp.print(", \"freq\": \""); udp.print("144.390 Mhz"); udp.print("\"");
//    udp.print(", \"speed\": "); udp.print(0); 
//    udp.print(", \"temp\": "); udp.print(0); 
    udp.print(", \"altitude\": "); udp.print(TelemetryData.Altitude); 
    float temp = TelemetryData.Lat / 1E6;
    sprintf(templine,"%3.5f", temp);
    udp.print(", \"latitude\": "); udp.print(templine); 
    temp = TelemetryData.Lng / 1E6;
    sprintf(templine,"%3.5f", temp);
    udp.print(", \"longitude\": "); udp.print(templine); 
//    udp.print(", \"humidity\": "); udp.print(-1);  
//    udp.print(", \"batt\": "); udp.print(-1);
    udp.print(", \"station\": \""); udp.print(MY_CALLSIGN); udp.print("\"");         
    udp.print(", \"hour\": "); udp.print(TelemetryData.Hour); 
    udp.print(", \"minute\": "); udp.print(TelemetryData.Minute); 
    udp.print(", \"second\": "); udp.print(TelemetryData.Second); 
    udp.print(", \"month\": "); udp.print(TelemetryData.Month); 
    udp.print(", \"day\": "); udp.print(TelemetryData.Day); 
    udp.print(", \"year\": "); udp.print(TelemetryData.Year); 
    sprintf(templine,"%02d:%02d:%02d", TelemetryData.Hour, TelemetryData.Minute, TelemetryData.Second);
    udp.print(", \"time\": \""); udp.print(templine); udp.print("\"");
    udp.print(", \"model\": \""); udp.print("APRS"); udp.print("\"");
    udp.print(", \"type\": \""); udp.print("PAYLOAD_SUMMARY"); udp.print("\"");
//    udp.print(", \"heading\": "); udp.print(0); 
    udp.print(", \"satellites\": "); udp.print(TelemetryData.Satellites); 
    temp = ((float) TelemetryData.Millivolts / 1000.0);
    sprintf(templine,"%2.1f", temp);
    udp.print(", \"voltage\": "); udp.print(templine); 
    temp = ((float) TelemetryData.TempTenths / 10.0);
    sprintf(templine,"%2.1f", temp);
    udp.print(", \"temperature\": "); udp.print(templine); 
    udp.println("}");  
    udp.endPacket();
    LastPacketForUDP = PacketsReceived;     // Make note of the packet that we used for this telemetry.
    return true;
  }
  return false;    // (Skipped packet)
}
/**************************************************************************************************************/


void CheckWiFiAndUDPbind() {
  uint8_t status = wifiMulti.run();
  if ((status == WL_CONNECTED) && (CurrentWiFiStatus != WL_CONNECTED)) {
    // New connection established
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else { // Not connected.
    if (CurrentWiFiStatus != WL_CONNECTED) {
      // WiFi unbound for some reason
      Serial.print("Wifi Disconnected.  Status: "); Serial.println(status);
    } else {
      // WiFi Still connected.  Do nothing.
    } 
  }
  CurrentWiFiStatus = status;
}

/**************************************************************************************************************/

void setup() { 
  
  //initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Alpha test LoRa receiver");
  pinMode(LED_DROP, OUTPUT); digitalWrite(LED_DROP, LOW);
  pinMode(LED_WARNING, OUTPUT); digitalWrite(LED_WARNING, LOW);
  pinMode(LED_SAFETY, OUTPUT); digitalWrite(LED_SAFETY, LOW);
  pinMode(LED_WIFI_STATUS, OUTPUT); digitalWrite(LED_WIFI_STATUS, LOW);
  pinMode(LED_BUZZER, OUTPUT); digitalWrite(LED_BUZZER, LOW);
  pinMode(LED_PARACHUTE, OUTPUT); digitalWrite(LED_PARACHUTE, LOW);
  pinMode(DROP_SWITCH, INPUT);
  pinMode(SAFETY_SWITCH, INPUT);
  pinMode(SWITCH_PARACHUTE, INPUT);
  pinMode(SWITCH_BUZZER, INPUT);
      
  OledInit();  
  LoRaInit();     
  WiFiInit();
  OledUpdate();   // Paint initial screen
}

uint32_t  NextSecond = 0;       // Initialize couner for 1 second loop
uint32_t  NextFifteen = 0;   // Initialize timer for 5 second loop
uint32_t  NextPayloadSummary = 0;         // Initialize timer for Payload Summary loop

void loop() {
  LoRaReceive(LoRa.parsePacket());

  // One Second Loop
  if (millis() > NextSecond) {  
    NextSecond = millis() + 1000;
    LoRaSend();  //once a second send LoRa signal to Tbeam to confirm connection
    LEDupdate();
    OledUpdate();
  } // One Second Loop

  // 15 second loop
  if (millis() > NextFifteen) {   
        NextFifteen = millis() + 15000;
        CheckWiFiAndUDPbind();    // Failed WiFi bind takes about 9 seconds, and freezes us up, so we only try this once/minute
  }  // 15 second loop
  
  // Send payload summaries less frequently, on their own timer.
  if (millis() > NextPayloadSummary) {
    if (SendPayloadSummary()) {
       NextPayloadSummary = millis() + PAYLOAD_SUMMARY_INTERVAL;
    } else {
      // If we didn't send payload summary (bad data), retry in a second.
      NextPayloadSummary = millis() + 1000;
    }
  }
}
