# SmartThings-ESP32-Multi-Device

This project consists of code for an ESP32 to interface with SmartThings using the SmartThings MQTT Bridge. While any ESP32 can be used, ones with built-in displays like Heltec Wifi Kit 32, Wemos ESP32 LoLin and even the Heltec Wifi Kit 8 (ESP8266) should be considered instead. The project includes Samsung SmartThings Device Type handlers for a stateful (on/off) switch, a stateless/momentary button and a contact sensor. The project also supports a variety of temperature & humidity sensors which can be seen on all 3 device types. Two of each capabilities (switch/button/contact) can be ran on each ESP32 device.

GitHub link: https://github.com/JZ-SmartThings/SmartThings-ESP32-Multi-Device/tree/master/

ESP32 Instructions
__________________

The *.ino file is an Arduino IDE code sample which is also compatible with MS Code Platform IO so the ino file can be renamed to cpp if desired. Verify the few options at the top of the script before flashing your device. The default settings are for a Heltec Wifi Kit 32 with a 128x64 OLED display.

SmartThings MQTT Bridge
_______________________

The bridge should be configured properly well before using this code. I had to make some changes to the MQTT SmartApp in order for the Contact state, Temperature & Humidity to be written back to SmartThings. Please see the following version of the SmartApp in my fork in order to properly use Contact Sensor, Temperature Measurement & Relative Humidity capabilities. If this change is not made then none of three capabilities will sync back to SmartThings.
https://github.com/JZ-SmartThings/smartthings-mqtt-bridge/blob/master/smartapps/stj/mqtt-bridge.src/mqtt-bridge.groovy

Displays
________

Displays like the OLED 128x64, 128x32 and the 2004 LCD (20 character 4 line) are all supported w/o code changes.

Temperature & Humidity
______________________

Temperature & Humidity sensors like the BME280, DHT22, DHT21, DHT11 and the built-in ESP32 temperature sensor (only on older chips, deprecated in later chips) are all supported w/o code change.

Contact Sensor
______________

Make sure to use my fork and run the MQTT SmartApp from there otherwise the contact sensor will not work.

* v1.0.20190907 - Changed MQTT SmartApp in order to sync Contact state. No longer using on/off sync workaround. Small enhancements and fixes.
* v1.0.20190831 - Initial version
</br>
<img src="https://raw.githubusercontent.com/JZ-SmartThings/SmartThings-ESP32-Multi-Device/master/_PICTURE.jpg">