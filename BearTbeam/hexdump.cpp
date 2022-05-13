#include <arduino.h>

void DumpHex(const void* data, size_t size) {
  char ascii[17];
  size_t i, j;
  ascii[16] = '\0';
  for (i = 0; i < size; ++i) {
    //printf("%02X ", ((unsigned char*)data)[i]);
    if (((uint8_t *)data)[i] < 16) Serial.print("0");
    Serial.print(((char*)data)[i], HEX);
    Serial.print(" ");
    if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
      ascii[i % 16] = ((unsigned char*)data)[i];
    } else {
      ascii[i % 16] = '.';
    }
    if ((i+1) % 8 == 0 || i+1 == size) {
      //printf(" ");
      Serial.print(" ");
      if ((i+1) % 16 == 0) {
        //printf("|  %s \n", ascii);
        Serial.print("|  ");
        Serial.println(ascii);    // Don't see a need for the trailing space
      } else if (i+1 == size) {
        ascii[(i+1) % 16] = '\0';
        if ((i+1) % 16 <= 8) {
          //printf(" ");
          Serial.print(" ");
        }
        for (j = (i+1) % 16; j < 16; ++j) {
          //printf("   ");
          Serial.print("   ");
        }
        //printf("|  %s \n", ascii);
        Serial.print("|  ");
        Serial.println(ascii);    // Don't see a need for the trailing space
      }
    }
  }
}
