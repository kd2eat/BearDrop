# BearDrop

## Major components

### Payload tracker - "T-beam"
This is an ESP32 TTGO T-Beam microcontroller.  It has integrated GPS, LoRa, Bluetooth and WiFi.  This device gets a GPS lock, and beacons to the ground station once per second with current coordinates.  It also receives command packets from the ground station which can initiate cutdown of a drop of our stuffed Bear ppayload.  The payload drop can be initiated manually from the ground station, or automatically at a hard-coded altitude.  There is also a "lock" functionality which the ground station can inititate, which prevents the bear from being dropped.

### Ground station 
The Ground station is implemented on a ESP32 LoRa device with integrated OLED display.  It receives packets from the payload, and also sends a packet every few seconds to confirm connectivity.  Should contact be lost for more than a few seconds, an LED lights indicating that there is a connectivity issue.  It also has switches to force a drop of the payload as well as lock out the drop function.  Finally, it also joins a hard-coded WiFi network and sends UDP "Payload Summaries" for each packet received from the tracker.  A linux computer on the same network can receive these payload summmaries and forward them to various tracking systems so that we can monitor the progress of the flight.
