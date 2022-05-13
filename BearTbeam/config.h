#include <stdint.h>

#define DROPALTITUDE 30500 //in meters

// Pin Definitions
#define LED_GREEN 0
#define LED_BLUE  4
#define LED_RED   15
#define LED_YELLOW 35
#define LED_TIME  LED_RED
#define LED_BYTESIN  LED_BLUE
#define PIN_SERVO 2
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22
#define CUTDOWN     13 //GPIO13 cutdown nicrome

//########################################################################################
//# Functions
#define OLED
#define LORA
#define CAMERA_SWITCHER               // Enable if using a camera switching module
//########################################################################################

#ifdef CAMERA_SWITCHER
#define DOWNCAMERA 1000    // PWM value to select the downward facing camera
#define SIDECAMERA 2000    // PWM value to select the side facing camera

#define DOWN_CAMERA_FORCE_ALTITUDE  25058   // Force the camera downward and hold at this altitude
#define DOWN_CAMERA_UNFORCE_ALTITUDE  27743 // Stop forcing the camera downward at this altitude
#define DOWN_CAMERA_FORCE_MAX_MINUTES 10    // Max time to force the camera down (in case of GPS issue or early burst)
#define DOWN_CAMERA_TIME  10    // Seconds to use downward camera when alternating
#define SIDE_CAMERA_TIME  20    // Seconds to use side camera when alternating
#endif // CAMERA_SWITCHER


#define BEAMER
