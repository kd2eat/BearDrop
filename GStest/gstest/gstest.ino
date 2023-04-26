#define SWITCH_BEAR 32
#define SWITCH_SAFETY 33
#define SWITCH_PARACHUTE 23
#define SWITCH_BUZZER  0
#define LED_BEAR 12     // Blue
#define LED_WARNING 13  // Yellow
#define LED_SAFETY  17  // Red
#define LED_WIFI_STATUS 2
#define LED_BUZZER  22
#define LED_PARACHUTE 25

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Ground Station Test Program");
  pinMode(LED_BEAR, OUTPUT); digitalWrite(LED_BEAR, LOW);
  pinMode(LED_PARACHUTE, OUTPUT); digitalWrite(LED_PARACHUTE, LOW);
  pinMode(LED_WARNING, OUTPUT); digitalWrite(LED_WARNING, LOW);
  pinMode(LED_SAFETY, OUTPUT); digitalWrite(LED_SAFETY, LOW);
  pinMode(LED_WIFI_STATUS, OUTPUT); digitalWrite(LED_WIFI_STATUS, LOW);
  pinMode(LED_BUZZER, OUTPUT); digitalWrite(LED_BUZZER, LOW);  
  pinMode(SWITCH_BEAR, INPUT);
  pinMode(SWITCH_SAFETY, INPUT);
  pinMode(SWITCH_BUZZER, INPUT);
  pinMode(SWITCH_PARACHUTE, INPUT);
}

void loop() {

  int32_t next = millis();
  bool  led = false;
  bool  temp;
  
  while (1) {
    temp = digitalRead(SWITCH_BEAR); if (temp) {Serial.println("Bear");}
    digitalWrite(LED_BEAR,digitalRead(SWITCH_BEAR));
    
    temp = digitalRead(SWITCH_PARACHUTE); if (temp) {Serial.println("Parachute");}
    digitalWrite(LED_PARACHUTE,digitalRead(SWITCH_PARACHUTE));

    temp = !digitalRead(SWITCH_BUZZER); if (temp) {Serial.println("Buzzer");}
    digitalWrite(LED_BUZZER,!(digitalRead(SWITCH_BUZZER)));
    
    temp = digitalRead(SWITCH_SAFETY); if (temp) {Serial.println("Safety");}
    digitalWrite(LED_SAFETY,digitalRead(SWITCH_SAFETY));

    //digitalWrite(LED_SAFETY,LOW);
    delay(100);
    if (millis() > next) {
       next += 1000;
       digitalWrite(LED_WARNING, led);
       led = !led;
       digitalWrite(LED_WIFI_STATUS, led);
    }
  }
  
}
