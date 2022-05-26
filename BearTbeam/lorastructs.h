#define LORA_GROUND_STATION 0xbb
#define LORA_BROADCAST 0xff
#define LORA_TRACKER    0xaa

typedef struct TelemetryData { //Tbeam to Ground Station
  byte  DestinationAddr;
  byte  SourceAddr;
  int16_t  TrackerRSSI;
  uint32_t  MsgCount;
  int32_t Lat;
  int32_t Lng;
  int32_t Hdop;
  float  TrackerSNR;
  uint16_t  Altitude;
  uint16_t  MaxAltitude;
  uint16_t LastCommandReceived;
  uint16_t  BadPacketsReceived;
  uint16_t  Year;
  uint8_t   Month;
  uint8_t   Day;
  uint8_t   Hour;
  uint8_t   Minute;
  uint8_t   Second;
  uint8_t   Satellites;
  uint8_t  BearDropped;           //being sent back from the bear
  uint8_t GoodFix;
  uint8_t  OverrideReceived;   // don't need Tbeam to communicate this to ground but need vise versa?
  uint8_t Checksum[2];        // Calculated just before we send the packet
};


typedef struct TeleCommand {
  byte  DestinationAddr;
  byte  SourceAddr;
  uint16_t  CommandCount;
  uint8_t dropSwitchState;
  uint8_t  overrideSwitchState; 
  uint8_t Checksum[2];        // Calculated just before we send the packet

};
