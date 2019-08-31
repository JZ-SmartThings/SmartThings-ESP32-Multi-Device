# SmartThings-ESP32-Multi-Device

This project consists of code for an ESP32 to interface with SmartThings using the SmartThings MQTT Bridge. While any ESP32 can be used, ones with built-in displays like Heltec Wifi Kit 32, Wemos ESP32 LoLin and even the Heltec Wifi Kit 8 (ESP8266) should be considered instead. The project includes Samsung SmartThings Device Type handlers for a stateful (on/off) switch, a stateless/momentary button and a contact sensor. The project also supports a variety of temperature & humidity sensors which can be seen on all 3 device types.
GitHub link: https://github.com/JZ-SmartThings/SmartThings-ESP32-Multi-Device/tree/master/

---ESP32 Instructions
The *.ino file is an Arduino IDE code sample which is also compatible with MS Code Project IO so the ino file can be renamed to cpp if desired. Verify the few options at the top of the script before flashing your device. The default settings are for a Heltec Wifi Kit 32 with a 128x64 OLED display.

---SmartThings MQTT Bridge
The bridge should be configured properly well before using this code. I had to make some changes to the MQTT SmartApp in order for the Temperature & Humidity to be written back to SmartThings. Please see the following commit in my fork in order to properly use Temperature Measurement & Relative Humidity capabilities. If this change is not made then neither temperature nor humidity will sync back to SmartThings.
https://github.com/JZ-SmartThings/smartthings-mqtt-bridge/commit/2d756afe98da2eb9073fddcca3910d4eb02b3944

---Displays
Displays like the OLED 128x64, 128x32 and the 2004 LCD (20 character 4 line) are all supported w/o code changes.

---Temperature & Humidity
Temperature & Humidity sensors like the BME280, DHT22, DHT21, DHT11 and the built-in ESP32 temperature sensor (only on older chips, deprecated in later chips) are all supported w/o code change.

---Contact Sensor
The contact sensor is read-only in SmartThings. I used a switch capability to sync with the contact state. So if desiring to use the contact sensor, please enable SWITCH sync for your device in the MQTT SmartApp otherwise it will not work.

* v1.0.20190831 - Initial version
</br>
<img src="https://raw.githubusercontent.com/JZ-SmartThings/SmartThings-ESP32-Multi-Device/_PICTURE.jpg">