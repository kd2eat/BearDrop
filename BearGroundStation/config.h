//#define SSID1 "RedRover"			// Change as needed
//#define PASSWORD1 NULL				// Change as needed

#define SSID1 "The Hojos"      // Change as needed
#define PASSWORD1 "quiltsail1"        // Change as needed

#define SSID2 "Thehojos mobile 2.4"		// Change as needed
#define PASSWORD2 "NotTheRealPassword"		// Change as needed

#define LORA_2_PAYLOAD_CALLSIGN "KD2EAT-11"
#define MY_CALLSIGN "KD2EAT-10"

#define READONLY_STATION

#define BROADCAST_MASK  (uint32_t)0x000000ff      // Be dumb and assume /24 networks for now
#define PAYLOAD_SUMMARY_PORT  55673
//#define PAYLOAD_SUMMARY_PORT  5000
#define UDP_LOGGING_PORT  5010
#define PAYLOAD_SUMMARY_INTERVAL  10000   // Interval for UDP packet sending (ms)

#define DROP_SWITCH 32
#define SAFETY_SWITCH 33
#define LED_DROP 12     // Blue
#define LED_WARNING 13  // Yellow
#define LED_SAFETY  17  // Red
#define LED_WIFI_STATUS 2
