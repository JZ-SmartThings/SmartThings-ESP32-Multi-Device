const char* version_number = "1.0.20190921";

/**
 *  ESP32 Multi Device
 *  Source code can be found here: https://github.com/JZ-SmartThings/SmartThings/blob/master/Devices/ESP32%20Multi%20Device
 *  Copyright 2019 JZ
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */

// ------------------------------------------------------------------  WIFI & ETHERNET CONNECTION

#if defined(ESP32) || defined(ESP8266)
  const char* ssid = "SSID";
  const char* password = "password";
#elif defined(__AVR__)
  // You can use Ethernet.init(pin) to configure the CS (chip select) pin
  // 10=Most Arduino shields --- 5=MKR ETH shield --- 0=Teensy 2.0 --- 20=Teensy++ 2.0 --- 15=ESP8266 with Adafruit Featherwing Ethernet --- 33=ESP32 with Adafruit Featherwing Ethernet
  int ethPin = 10;

  //#include <UIPEthernet.h>
  #include <Ethernet.h>
  #include <Arduino.h>
  // Enter a MAC address and IP address for your controller below.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

  // The IP address will be dependent on your local network:
  byte ip[] = { 0, 0, 0, 0 }; // USE DHCP
  //byte ip[] = { 192, 168, 0, 177 }; // STATIC IP
#endif

unsigned long RebootFrequencyDays = 0; // ZERO DISABLES THE AUTO-REBOOT

// ------------------------------------------------------------------ SWITCH CONFIGURATION
bool Use5Vrelay = true;       // SEND GROUND TO THE PIN INSTEAD OF VCC AS USED WITH 5V RELAYS - FALSE SENDS VCC
int switch1 = 32;             // switch pin 1 ---Heltec Wifi Kit 32 (use 32,33,19,23) ---Heltec Wifi Kit 8 (use 3,13) ---UNO (use 5,6)
int switch2 = 33;             // switch pin 2 ---Heltec Wifi Kit 33 (use 32,33,19,23) ---Heltec Wifi Kit 8 (use 3,13) ---UNO (use 5,6)

int STATUS_LED = 25;          // OPTIONAL --- Heltec Wifi Kit 32 use 25

// ------------------------------------------------------------------ DESIGNATE CONTACT SENSOR PINS OR COMMENT OUT 2 LINES BELOW TO BYPASS CONTACT SENSORS
//#define CONTACTPIN1 19      // what pin is the 1st Contact Sensor on? ---Heltec Wifi Kit 32 (use 32,33,19,23) ---Heltec Wifi Kit 8 (use 3,13) ---UNO (use 5,6)
//#define CONTACTPIN2 23      // what pin is the 2nd Contact Sensor on? ---Heltec Wifi Kit 32 (use 32,33,19,23) ---Heltec Wifi Kit 8 (use 3,13) ---UNO (use 5,6)


// ------------------------------------------------------------------ MQTT TOPICS
const char* mqttServer = "192.168.0.251";
const char* mqttDeviceName = "ESP32HELTEC";
const char* mqttSwitch1Topic = "smartthings/ESP32HELTEC Stateful/switch";
const char* mqttSwitch2Topic = "smartthings/ESP32HELTEC Stateful 2/switch";
const char* mqttSwitch1StateTopic = "smartthings/ESP32HELTEC/state";
const char* mqttSwitch2StateTopic = "smartthings/ESP32HELTEC 2/state";
const char* mqttSwitch1MomentaryTopic = "smartthings/ESP32HELTEC Button/button";
const char* mqttSwitch2MomentaryTopic = "smartthings/ESP32HELTEC Button 2/button";
const char* mqttContact1Topic = "smartthings/ESP32HELTEC Contact/contact";
const char* mqttContact2Topic = "smartthings/ESP32HELTEC Contact 2/contact";
const char* mqttTemperatureTopic = "smartthings/ESP32HELTEC/temperature";
const char* mqttHumidityTopic = "smartthings/ESP32HELTEC/humidity";

// ---------------------------------------------------------- OLED & LCD SECTION --- UNCOMMENT ONE OF THE TWO OLEDs BELOW OR THE LCD 15 LINES DOWN
#define useOLED128X64       // Large OLED --- Heltec Wifi Kit 32 & Wemos ESP32 OLED
//#define useOLED128X32       // Small OLED --- Heltec Wifi Kit 8

#if defined(useOLED128X64) || defined(useOLED128X32)
  #include <U8g2lib.h>      //--- Full OLED list: https://github.com/olikraus/U8g2_Arduino/blob/master/examples/full_buffer/GraphicsTest/GraphicsTest.ino

  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);      // Heltec Wifi Kit 32 Clock=15 Data=4 Reset=16 --- Wemos LoLin Clock=4 Data=5 Reset=16
  //U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 4, /* data=*/ 5, /* reset=*/ 16);     // Heltec Wifi Kit 32 Clock=15 Data=4 Reset=16 --- Wemos LoLin Clock=4 Data=5 Reset=16

  //U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 5, /* data=*/ 4); //Heltec Wifi Kit 8
  //U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 5, /* data=*/ 4); //slower Heltec Wifi Kit 8 using software

  int sceneDuration = 5000; unsigned long sceneNextMillis = 0; int sceneNext = 1;
#endif

//#define useLCD2004       // 20 character 4 line LCD
#if defined(useLCD2004)
  #include <Wire.h> 
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27,20,4); // set the LCD address to 0x27 for a 20 chars and 4 line display

  int sceneDuration = 5000; unsigned long sceneNextMillis = 0; int sceneNext = 1;
  unsigned long lcdNextRefresh = millis();
#endif


// ------------------------------------------------------------------ CONFIGURE TEMPERATURE & HUMIDITY SENSORS
int temperatureOffset = 0;
int humidityOffset = 0;

// USE BME280 TEMP/HUMIDITY/PRESSURE SENSOR. PICK YOUR OPTIONS BELOW OR COMMENT OUT THE LINE BELOW TO BYPASS BME280 LOGIC
#define useBME280
#ifdef useBME280
  // DEFAULT BELOW USES I2C, PINS HERE ARE FOR SPI ONLY
  #define BME_SCK 13
  #define BME_MISO 12
  #define BME_MOSI 11
  #define BME_CS 10
  
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme; // I2C
  //Adafruit_BME280 bme(BME_CS); // hardware SPI
  //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

  #define SEALEVELPRESSURE_HPA (1013.25)
  unsigned long lastBME280read = 0;
#endif

// USE DHT TEMP/HUMIDITY SENSOR DESIGNATE WHICH PIN BELOW & PICK DHTTYPE BELOW AS WELL --- COMMENT OUT THE LINE BELOW TO BYPASS DHT LOGIC
//#define useDHT
#ifdef useDHT
  uint8_t DHTPIN = 27;    // what pin is the DHT on? --- Heltec Wifi Kit 32=pin 27  --- Wemos LoLin32=pin 13 --- Heltec Wifi Kit 8=pin 12 or D6
  // Uncomment whatever type of temperature sensor you're using!
  //#define DHTTYPE DHT11   // DHT 11
  #define DHTTYPE DHT22   // DHT 22  (AM2302)
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)

  #include <DHT.h>
  DHT dht(DHTPIN, DHTTYPE);
  unsigned long lastDHTread = 0;
#endif

// USE ESP32 INTERNAL TEMPERATURE SENSOR
//#define useESP32Temp
#ifdef useESP32Temp
  extern "C" // Extern C is used when we are using a funtion written in "C" language in a C++ code.
  {
    uint8_t temprature_sens_read(); // This function is written in C language
  }
  uint8_t temprature_sens_read();
  unsigned long lastESP32Tempread = 0;
#endif

float lastTemperaturePayload = -1; float lastHumidityPayload = -1; // only send MQTT on changes

// WIFI, OTA & HTTP
#include <PubSubClient.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <Update.h>
  WebServer server(80);
#endif
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #include <ESP8266HTTPUpdateServer.h>
  ESP8266WebServer server(80);
  ESP8266HTTPUpdateServer httpUpdater;
#endif
#ifdef __AVR__
  #include <SPI.h>
  #include <Server.h>
#endif

const char* updateIndex = "<p style='font-size: 150%;'>~~~mqttDeviceName~~~</p><form method='POST' action='/updatepost' enctype='multipart/form-data'><input type='file' name='update' style='font-size: 150%;'><br><br><input type='submit' value='Update' style='font-size: 150%;'></form>";
const char* updateDone = "<html><head><meta http-equiv=\"REFRESH\" content=\"10;URL=/\"></head><marquee direction=\"right\"><h1>Update went OK!</h1><h1>Rebooting...</h1></marquee></html>";
const char* rebootIndex = "<p style='font-size: 150%;'>~~~mqttDeviceName~~~</p><button style='font-size: 150%;' onClick=\"javascript: if (confirm(\'Are you sure you want to reboot?\')) parent.location='/rebootnow';\">Reboot</button>";
const char* rebootNow = "<html><head><meta http-equiv=\"REFRESH\" content=\"10;URL=/\"></head><marquee direction=\"right\"><h1>Rebooting...</h1></marquee></html>";

#ifdef CONTACTPIN1
  const char* lastContact1Payload;
#endif
#ifdef CONTACTPIN2
  const char* lastContact2Payload;
#endif

#if defined (ESP32) || defined (ESP8266)
  WiFiClient espClient;
  PubSubClient client(espClient);
#elif defined(__AVR__)
  EthernetClient ethClient;
  EthernetServer server(80);
  PubSubClient client(ethClient);
#endif

int mqtt_reconnect_count = 0;

// ------------------------------------------------------------------ FUNCTION DECLARATIONS
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
String uptime();
String jsonString();
String infoPageString();
float probeDHT(int whichSensor);
void outputDHT(unsigned long varMillis);
void outputBME280(unsigned long varMillis);
void outputESP32Temp(unsigned long varMillis);
void DisplayScene(unsigned long varMillis, int whichScene);
void setup_wifi(bool reset_wifi = false);


void setup() { // ---------------------------------------------------------------------------------------------------------- SETUP
  Serial.begin(115200);

  pinMode(switch1, OUTPUT);
  pinMode(switch2, OUTPUT);
  digitalWrite(switch1, Use5Vrelay == true ? HIGH : LOW);
  digitalWrite(switch2, Use5Vrelay == true ? HIGH : LOW);

  #ifdef CONTACTPIN1
    pinMode(CONTACTPIN1, INPUT_PULLUP);
  #endif
  #ifdef CONTACTPIN2
    pinMode(CONTACTPIN2, INPUT_PULLUP);
  #endif

  #if defined(useOLED128X64) || defined(useOLED128X32)
    u8g2.begin();
    u8g2.clearBuffer();          // clear the internal memory
  #endif

  #if defined(useLCD2004)
    lcd.init(); //initialize the lcd
    lcd.backlight(); //enable the backlight
  #endif


// SETUP NETWORK
  #if defined (ESP32) || defined (ESP8266)
    setup_wifi(false);
  #elif defined(__AVR__)
    Ethernet.init(ethPin);
   
    // start the Ethernet connection and the server:
    if (ip[0]==0 && ip[1]==0 && ip[2]==0 && ip[3]==0) {
      Ethernet.begin(mac); // USE DHCP
    } else {
      Ethernet.begin(mac, ip); // USE STATIC IP ABOVE
    }
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      while (true) {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }

    // start the server
    server.begin();
    Serial.print("IP address: ");
    Serial.println(Ethernet.localIP());
  #endif

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  #ifdef useDHT
    pinMode(DHTPIN, INPUT);
    dht.begin();
    outputDHT(millis());
  #endif

  #ifdef useBME280
    unsigned status;
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"); Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"); Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n"); Serial.print("        ID of 0x61 represents a BME 680.\n");
        //while (1); --disable endless loop if BME not found
    }
    outputBME280(millis());
  #endif

  #ifdef useESP32Temp
    outputESP32Temp(millis());
  #endif

  #if defined(useOLED128X64) || defined(useOLED128X32)
    sceneNextMillis = millis() + sceneDuration;
  #endif

} // ---------------------------------------------------------------------------------------------------------- SETUP




// ---------------------------------------------------------------------------------------------------------- SET UP WIFI AND OTHER WIFI DEPENDENT ITEMS
void setup_wifi(bool reset_wifi) {
  #if defined (ESP32) || defined (ESP8266)
    delay(50);
    // STATUS LED ON WHILE ACQUIRING WIFI
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);   // Turn the LED on (Note that LOW is the voltage level

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    //WiFi.mode(WIFI_AP_STA);
    if (reset_wifi==true) {
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      delay(1000);
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      #if defined(useOLED128X64)
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
        u8g2.drawStr(0,12,"Trying SSID:");
        u8g2.drawStr(0,24,ssid);
        u8g2.drawStr(0,48,"for...");
        char buf[16];
        dtostrf(counter/2,0,0,buf);
        strcat(buf,(counter/2) == 1 ? " second" : " seconds");
        u8g2.drawStr(0,60,buf);
        u8g2.sendBuffer();          // transfer internal memory to the display
      #endif
      #if defined(useOLED128X32)
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
        char buf[16];
        dtostrf(counter/2,0,0,buf);
        u8g2.drawStr(0,12,"Trying SSID:");
        u8g2.drawStr(112,12,buf);
        u8g2.drawStr(0,24,ssid);
        u8g2.sendBuffer();          // transfer internal memory to the display
      #endif
      #if defined(useLCD2004)
        lcd.clear(); //initialize the lcd
        lcd.setCursor(0,0);
        lcd.print("Trying SSID:");
        lcd.setCursor(14,0); // set the cursor to column 14, line 0
        lcd.print(counter/2);
        lcd.setCursor(0,1);
        lcd.print(ssid);
      #endif

      counter++;
      if ( counter >= 120 ) { ESP.restart(); }
    }

    #if defined(useOLED128X64)
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0,12,"Success with SSID:");
      u8g2.drawStr(0,24,ssid);
      u8g2.drawStr(0,48,"Time to connect:");
      char buf[20];
      dtostrf(counter/2,0,0,buf);
      strcat(buf,(counter/2) == 1 ? " second" : " seconds");
      u8g2.drawStr(0,60,buf);
      u8g2.sendBuffer();          // transfer internal memory to the display
      delay(3000);
      //u8g2.clearBuffer();
    #endif
    #if defined(useOLED128X32)
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0,12,"Success with SSID:");
      u8g2.drawStr(0,24,ssid);
      u8g2.sendBuffer();
      delay(3000);
      u8g2.clearBuffer();
      u8g2.drawStr(0,12,"Time to connect:");
      char buf[20];
      dtostrf(counter/2,0,0,buf);
      strcat(buf,(counter/2) == 1 ? " second" : " seconds");
      u8g2.drawStr(0,24,buf);
      u8g2.sendBuffer();
      delay(3000);
      //u8g2.clearBuffer();
    #endif

    #if defined(useLCD2004)
      lcd.clear(); //initialize the lcd
      lcd.setCursor(0,0);
      lcd.print("Success with SSID:");
      lcd.setCursor(0,1); // set the cursor to column 14, line 0
      lcd.print(ssid);
      lcd.setCursor(0,2); // set the cursor to column 14, line 0
      lcd.print("Time to connect:");
      lcd.setCursor(0,3); // set the cursor to column 14, line 0
      String cnt = String(counter/2);
      cnt == "1" ? cnt+=" second" : cnt+=" seconds";
      lcd.print(cnt);
      delay(3000);
    #endif

    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    // STATUS LED OFF
    digitalWrite(STATUS_LED, LOW);  // Turn the LED off by making the voltage HIGH

    //OTA & JSON
    if (WiFi.status() == WL_CONNECTED) {
      String mqttDeviceNameSTRING = String(mqttDeviceName);
      mqttDeviceNameSTRING.replace(" ","_");
      MDNS.begin(mqttDeviceNameSTRING.c_str());
      server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.sendHeader("charset", "ISO-8859-1");
        server.send(200, "text/json", (char*) jsonString().c_str());
      });
      server.on("/favicon.ico", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(404, "text/html", "404 Page not found");
      });
      server.on("/reboot", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        String rebootIndexSTRING = String(rebootIndex);
        rebootIndexSTRING.replace("~~~mqttDeviceName~~~",String(mqttDeviceName));
        server.send(200, "text/html", rebootIndexSTRING);
      });
      server.on("/rebootnow", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", rebootNow);
        delay(1000);
        ESP.restart();
      });
      server.on("/info", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", (char*) infoPageString().c_str() );
      });
      #ifdef ESP32
        server.on("/update", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          String updateIndexSTRING = String(updateIndex);
          updateIndexSTRING.replace("~~~mqttDeviceName~~~",String(mqttDeviceName));
          server.send(200, "text/html", updateIndexSTRING);
        });
        server.on("/updatepost", HTTP_POST, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", (Update.hasError()) ? "FAIL" : updateDone);
          delay(1000);
          ESP.restart();
        }, []() {
          HTTPUpload& upload = server.upload();
          if (upload.status == UPLOAD_FILE_START) {
            Serial.setDebugOutput(true);
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin()) { //start with max available size
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { //true to set the size to the current progress
              Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
              Update.printError(Serial);
            }
            Serial.setDebugOutput(false);
          }
        });
      #endif
      #ifdef ESP8266
        httpUpdater.setup(&server);
      #endif

      server.begin();
      MDNS.addService("http", "tcp", 80);

      Serial.printf("Ready! Open http://%s.local in your browser\n", mqttDeviceNameSTRING.c_str());
    }
  #endif
} // ---------------------------------------------------------------------------------------------------------- setup_wifi

String infoPageString() {
  #if defined (ESP32) || defined (ESP8266)
    String infoData = "<html><head>";
    infoData.concat("<link href=\"data:image/x-icon;base64,AAABAAEAEBAAAAAAAABoBQAAFgAAACgAAAAQAAAAIAAAAAEACAAAAAAAAAEAAAAAAAAAAAAAAAE\
AAAAAAAAAAAAAsmw5AK1pOAC0bTsArmk4AK9qOACxazoAtm87ALJsOgCzbToAr2o5ALBqOQCwazkAtW46ALFrOQAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQQCAgQBAAAAAAAAAAAMAgIBDAwBAgIMAAAAAAACAg4AAAAAAAAOAgQ\
AAAAMAgsAAAAAAAAAAAwCCwAAAgIAAAAAAAAAAAAAAgIACQIEAAAAAAAAAAAAAAQCDQgCCgAAAAAAAAAAAAAKAgcAAgIAAAAAAgIAAAAAAgIAAAI\
CAAAAAAICAAAAAAICAAAAAgIAAAACAgAAAAICAAAAAA4CCQAAAgIAAAMFBgAAAAAAAAAAAAICAAAAAAAAAAAAAAAAAAACAgAAAAAAAAAAAAAAAAA\
ABQoAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP//AAD4HwAA4AcAAMfjAACP8QAAn/kAAB/4AAAf+AAAnnkAAJ55AADOcwAAxmMAAP5/AAD+fwAA/n8\
AAP//AAA=\" rel=\"icon\" type=\"image/x-icon\" />");
    infoData.concat("</head><body style=\"font-size: 14px;\"><table border=1 cellpadding=3 style=\"display:inline-block;\"><tbody>");
    infoData.concat("\r\n<tr><td>Update Page</td><td><a href=\"http://"); infoData.concat(WiFi.localIP().toString()); infoData.concat("/update\">");
    infoData.concat("http://"); infoData.concat(WiFi.localIP().toString()); infoData.concat("/update");infoData.concat("</a> OR <a href=\"http://");
    String mqttDeviceNameSTRING = String(mqttDeviceName);
    mqttDeviceNameSTRING.replace(" ","_");
    infoData.concat(mqttDeviceNameSTRING); infoData.concat(".local/update\">");
    infoData.concat("http://"); infoData.concat(mqttDeviceNameSTRING); infoData.concat(".local/update");infoData.concat("</a></td></tr>");
    infoData.concat("\r\n<tr><td>Reboot"); infoData.concat("</td><td><a href=\"/reboot\">Reboot Page</a>"); infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>Wireless SSID");infoData.concat("</td><td>");infoData.concat(ssid);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>Version");infoData.concat("</td><td>");infoData.concat(version_number);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>Use 5 volt relay?</td><td>"); infoData.concat( Use5Vrelay ? "true" : "false" ); infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>Switch 1</td><td>on pin "); infoData.concat(switch1); infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>Switch 2</td><td>on pin "); infoData.concat(switch2); infoData.concat("</td></tr>");
    #ifdef CONTACTPIN1
      infoData.concat("\r\n<tr><td>Contact Sensor 1</td><td>Enabled on pin "); infoData.concat(CONTACTPIN1); infoData.concat("</td></tr>");
    #endif
    #ifdef CONTACTPIN2
      infoData.concat("\r\n<tr><td>Contact Sensor 2</td><td>Enabled on pin "); infoData.concat(CONTACTPIN2); infoData.concat("</td></tr>");
    #endif
    #if defined(useOLED128X64)
      infoData.concat("\r\n<tr><td>OLED128X64 Display</td><td>Enabled</td></tr>");
    #endif
    #if defined(useOLED128X32)
      infoData.concat("\r\n<tr><td>OLED128X32 Display</td><td>Enabled</td></tr>");
    #endif
    #if defined(useLCD2004)
      infoData.concat("\r\n<tr><td>LCD2004 Display</td><td>Enabled</td></tr>");
    #endif
    #ifdef useDHT
      infoData.concat("\r\n<tr><td>DHT"); infoData.concat(DHTTYPE); infoData.concat(" Multi-Sensor</td><td>Enabled on pin "); infoData.concat(DHTPIN); infoData.concat("</td></tr>");
    #endif
    #ifdef useBME280
      infoData.concat("\r\n<tr><td>BME280 Multi-Sensor</td><td>Enabled</td></tr>");
    #endif
    #ifdef useESP32Temp
      infoData.concat("\r\n<tr><td>ESP32 Internal Temperature Sensor</td><td>Enabled</td></tr>");
    #endif
    #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
      infoData.concat("\r\n<tr><td>Temperature Offset</td><td>"); infoData.concat(temperatureOffset); infoData.concat("</td></tr>");
    #endif
    #if defined(useDHT) || defined(useBME280)
      infoData.concat("\r\n<tr><td>Humidity Offset</td><td>"); infoData.concat(humidityOffset); infoData.concat("</td></tr>");
    #endif
    infoData.concat("\r\n<tr><td>mqttServer");infoData.concat("</td><td>");infoData.concat(mqttServer);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttDeviceName");infoData.concat("</td><td>");infoData.concat(mqttDeviceName);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch1Topic");infoData.concat("</td><td>");infoData.concat(mqttSwitch1Topic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch2Topic");infoData.concat("</td><td>");infoData.concat(mqttSwitch2Topic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch1StateTopic");infoData.concat("</td><td>");infoData.concat(mqttSwitch1StateTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch2StateTopic");infoData.concat("</td><td>");infoData.concat(mqttSwitch2StateTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch1MomentaryTopic");infoData.concat("</td><td>");infoData.concat(mqttSwitch1MomentaryTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttSwitch2MomentaryTopic");infoData.concat("</td><td>");infoData.concat(mqttSwitch2MomentaryTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttContact1Topic");infoData.concat("</td><td>");infoData.concat(mqttContact1Topic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttContact2Topic");infoData.concat("</td><td>");infoData.concat(mqttContact2Topic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttTemperatureTopic");infoData.concat("</td><td>");infoData.concat(mqttTemperatureTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n<tr><td>mqttHumidityTopic");infoData.concat("</td><td>");infoData.concat(mqttHumidityTopic);infoData.concat("</td></tr>");
    infoData.concat("\r\n</tbody></table>");
    infoData.concat("\r\n<div style=\"display: inline-block;vertical-align: top;margin: 20px;border-style: dashed;padding: 20px;\"><h3>JSON Data From the Root Page</h3>\r\n");
    String jsonTemp = jsonString(); jsonTemp.replace("\n","</br>");
    infoData.concat(jsonTemp);
    infoData.concat("\r\n</div></body><html>\r\n");
    return infoData;
  #else
    return "";
  #endif
}

String jsonString() {
  #if defined (ESP32) || defined (ESP8266) || defined (__AVR__)
    String serverIP = "";
    #if defined (__AVR__)
      //serverIP = String(Ethernet.localIP());
      return "";
    #else
      serverIP = WiFi.localIP().toString();
      String jsonData = "{";
      jsonData.concat("\r\n\"DeviceName\":\""); jsonData.concat(mqttDeviceName); jsonData.concat("\",");
      jsonData.concat("\r\n\"DeviceIP\":\""); jsonData.concat(serverIP); jsonData.concat("\",");
      jsonData.concat("\r\n\"Uptime\":\""); jsonData.concat(uptime()); jsonData.concat("\",");
      jsonData.concat("\r\n\"InfoPage\":\"http://");
      jsonData.concat(serverIP);
      jsonData.concat("/info\",");
      if (Use5Vrelay == true ) {
        jsonData.concat("\r\n\"Switch1\":\""); jsonData.concat(digitalRead(switch1) ? "off" : "on"); jsonData.concat("\",");
        jsonData.concat("\r\n\"Switch2\":\""); jsonData.concat(digitalRead(switch2) ? "off" : "on"); jsonData.concat("\"");
      } else {        
        jsonData.concat("\r\n\"Switch1\":\""); jsonData.concat(digitalRead(switch1) ? "on" : "off"); jsonData.concat("\",");
        jsonData.concat("\r\n\"Switch2\":\""); jsonData.concat(digitalRead(switch2) ? "on" : "off"); jsonData.concat("\"");
      }
      #ifdef CONTACTPIN1
        jsonData.concat(",");
        jsonData.concat("\r\n\"Contact1\":\""); jsonData.concat(digitalRead(CONTACTPIN1) ? "open" : "closed"); jsonData.concat("\"");
      #endif
      #ifdef CONTACTPIN2
        jsonData.concat(",");
        jsonData.concat("\r\n\"Contact2\":\""); jsonData.concat(digitalRead(CONTACTPIN2) ? "open" : "closed"); jsonData.concat("\"");
      #endif
      #if defined(useDHT) || defined(useBME280)
        if (lastTemperaturePayload != -1) { jsonData.concat(",\r\n\"Temperature\":\""); jsonData.concat(int(lastTemperaturePayload)); jsonData.concat("\""); }
        if (lastHumidityPayload != -1) { jsonData.concat(",\r\n\"Humidity\":\""); jsonData.concat(int(lastHumidityPayload)); jsonData.concat("\""); }
      #endif
      #if defined(useESP32Temp)
        if (lastTemperaturePayload != -1) { jsonData.concat(",\n\"Temperature\":\""); jsonData.concat(int(lastTemperaturePayload)); jsonData.concat("\""); }
      #endif
      jsonData.concat("\r\n}");
      return jsonData;
    #endif
  #else
    return "";  
  #endif
}


unsigned long loopMillis = 0;

void loop() { //---------------------------------------------------------------------------------------------------------- LOOP
  loopMillis = millis();
  //Serial.print("-------------------------LOOP MILLIS: "); Serial.println(loopMillis);

  // MQTT CONNECTED EVERY 15 MINUTES AND INITIAL
  if (loopMillis % 900000 < 200 || (loopMillis>=30000 && loopMillis<=30200)) { // every 15 minutes and at 30 seconds
    client.publish(mqttSwitch1StateTopic, "connected");
    client.publish(mqttSwitch2StateTopic, "connected");
    Serial.print("--------------------------------------------------UpTime: "); Serial.println(uptime());
  }

  // REBOOT FREQUENCY
  if (RebootFrequencyDays > 0 && loopMillis >= (86400000 * RebootFrequencyDays)) { //86400000 per day
    #if defined (ESP32) || defined (ESP8266)
      ESP.restart();
    #elif defined(__AVR__)
      while(1) {};
    #endif
  }

  #if defined (ESP32) || defined (ESP8266)
    // WIFI RECONNECT
    if (WiFi.status() != WL_CONNECTED) {
      setup_wifi(true);
    }
  #endif

  if (!client.connected()) { // MQTT RECONNECT
    reconnect();
  }
  client.loop();

  #ifdef ESP8266
    MDNS.update();
  #endif

  #if defined (ESP32) || defined (ESP8266)
    server.handleClient(); // HANDLE HTTP CLIENT
  #endif

  #if defined (__AVR__)
    // listen for incoming clients
    EthernetClient ethHTTPclient = server.available();
    if (ethHTTPclient) {
      //Serial.println("New HTTP client");
      boolean currentLineIsBlank = true;
      while (ethHTTPclient.connected()) {
        if (ethHTTPclient.available()) {
          char c = ethHTTPclient.read();
          //Serial.write(c);
          if (c == '\n' && currentLineIsBlank) {
            ethHTTPclient.println("HTTP/1.1 200 OK");
            ethHTTPclient.println("Content-Type: text/plain");
            ethHTTPclient.println("Connection: close");  // the connection will be closed after completion of the response
            ethHTTPclient.println("Refresh: 15");  // refresh the page automatically every 5 sec
            ethHTTPclient.println(); // DO NOT REMOVE
            ethHTTPclient.println(mqttDeviceName);
            ethHTTPclient.println(uptime());
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
          } else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        }
      }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      ethHTTPclient.stop();
      //Serial.println("Client disconnected");
    }
  #endif

  #if defined(useOLED128X64)
    if (loopMillis > sceneNextMillis) {
      DisplayScene(loopMillis, sceneNext);
      sceneNextMillis = loopMillis + sceneDuration;
      // 1=name&IP --- 2=temp&hmdt --- 3=switch&contacts
      if (sceneNext == 1) {
        #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
          if (lastTemperaturePayload != -1 || lastHumidityPayload != -1) {
            sceneNext=2;
          } else {
            sceneNext=3;
          }
        #else
          sceneNext=3;
        #endif
        //sceneNextMillis -= 3000;
      } else if (sceneNext == 2) {
        sceneNext=3;
      } else if (sceneNext == 3) {
        sceneNext=1;
      }
    } else { // SHOW CURRENT
      DisplayScene(loopMillis, sceneNext);
    }
  #endif
  #if defined(useOLED128X32)
    if (loopMillis > sceneNextMillis) {
      DisplayScene(loopMillis, sceneNext);
      sceneNextMillis = loopMillis + sceneDuration;
      // 1=name&IP --- 2=temp&hmdt --- 3=switch&contacts
      if (sceneNext == 1) { // UPTIME
        sceneNext=2;
      } else if (sceneNext == 2) { // SSID & IP
        sceneNext=3;
        #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
          if (lastTemperaturePayload != -1 || lastHumidityPayload != -1) {
            sceneNext=3;
          } else {
            sceneNext=4;
          }
        #else
          sceneNext=4;
        #endif
      } else if (sceneNext == 3) { // TEMP & HUMIDITY
        sceneNext=4;
      } else if (sceneNext == 4) { // SWITCHES
        sceneNext=5;
        #if defined(CONTACTPIN1) || defined(CONTACTPIN2)
          sceneNext=5;
        #else
          sceneNext=1;
        #endif
      } else if (sceneNext == 5) { // CONTACTS
        sceneNext=1;
        sceneNextMillis -= 1000;
      }
    } else { // SHOW CURRENT
      DisplayScene(loopMillis, sceneNext);
    }
  #endif
  #if defined(useLCD2004)
    if (loopMillis > sceneNextMillis) {
      DisplayScene(loopMillis, sceneNext);
      sceneNextMillis = loopMillis + sceneDuration;
      if (sceneNext == 1) {
        #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
          if (lastTemperaturePayload != -1 || lastHumidityPayload != -1) {
            sceneNext=2;
          } else {
            sceneNext=3;
          }
        #else
          sceneNext=3;
        #endif
        //sceneNextMillis -= 3000;
      } else if (sceneNext == 2) {
        sceneNext=3;
      } else if (sceneNext == 3) {
        sceneNext=1;
      }
    } else { // SHOW CURRENT
      DisplayScene(loopMillis, sceneNext);
    }
  #endif


  #ifdef useDHT
    if (loopMillis-lastDHTread > 30000 ) { // DHT every 30 seconds
      lastDHTread += 30000;
      outputDHT(loopMillis);
    }
  #endif // DHT
  #ifdef useBME280
    if (loopMillis-lastBME280read > 30000 ) { // BME280 every 30 seconds
      lastBME280read += 30000;
      outputBME280(loopMillis);
    }
  #endif // BME280
  #ifdef useESP32Temp
    if (loopMillis-lastESP32Tempread > 30000 ) { // ESP32 Temp every 30 seconds
      lastESP32Tempread += 30000;
      outputESP32Temp(loopMillis);
    }
    outputESP32Temp(millis());
  #endif
  

  #ifdef CONTACTPIN1
    // CONTACT SENSOR 1
    bool currentContact1Read = digitalRead(CONTACTPIN1);
    if (lastContact1Payload!=(currentContact1Read ? "open" : "closed")) {
      client.publish(mqttContact1Topic,currentContact1Read ? "open" : "closed");
      lastContact1Payload = currentContact1Read ? "open" : "closed";

      Serial.print ("mqttSensor1 published: "); Serial.println (lastContact1Payload);
      delay(1000);
    }
  #endif
  #ifdef CONTACTPIN2
    // CONTACT SENSOR 2
    bool currentContact2Read = digitalRead(CONTACTPIN2);
    if (lastContact2Payload!=(currentContact2Read ? "open" : "closed")) {
      client.publish(mqttContact2Topic,currentContact2Read ? "open" : "closed");
      lastContact2Payload = currentContact2Read ? "open" : "closed";

      Serial.print ("mqttSensor2 published: "); Serial.println (lastContact2Payload);
      delay(1000);
    }
  #endif
} // ---------------------------------------------------------------------------------------------------------- LOOP



void callback(char* topic, byte* payload, unsigned int length) { // MQTT CALLBACK
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("] ");

  String fullPayload="";
  for (unsigned int i=0;i<length;i++) {
    fullPayload += (char)payload[i];
  }
  Serial.println(fullPayload);

  if ((String(topic)==mqttSwitch1Topic && fullPayload=="on") || (String(topic)==mqttSwitch1Topic && fullPayload=="off") \
        || (String(topic)==mqttSwitch1MomentaryTopic && fullPayload=="pushed") ) {
      Serial.println("Starting Switch 1 MQTT Callback...");
      if (String(topic)==mqttSwitch1Topic && fullPayload=="on") {
        digitalWrite(switch1, Use5Vrelay == true ? LOW : HIGH);
        //client.publish(mqttSwitch1Topic,"", true);
      }
      if (String(topic)==mqttSwitch1Topic && fullPayload=="off") {
        digitalWrite(switch1, Use5Vrelay == true ? HIGH : LOW);
        //client.publish(mqttSwitch1Topic,"", true);
      }
      if (String(topic)==mqttSwitch1MomentaryTopic && fullPayload=="pushed") {
        digitalWrite(switch1, Use5Vrelay == true ? LOW : HIGH);
        delay(300);
        digitalWrite(switch1, Use5Vrelay == true ? HIGH : LOW);
        client.publish(mqttSwitch1MomentaryTopic,"momentary_done", true);
        client.publish(mqttSwitch1MomentaryTopic,"", true);
      }
    }

  if ((String(topic)==mqttSwitch2Topic && fullPayload=="on") || (String(topic)==mqttSwitch2Topic && fullPayload=="off") \
        || (String(topic)==mqttSwitch2MomentaryTopic && fullPayload=="pushed") ) {
      Serial.println("Starting Switch 2 MQTT Callback...");
      if (String(topic)==mqttSwitch2Topic && fullPayload=="on") {
        digitalWrite(switch2, Use5Vrelay == true ? LOW : HIGH);
        //client.publish(mqttSwitch2Topic,"", true);
      }
      if (String(topic)==mqttSwitch2Topic && fullPayload=="off") {
        digitalWrite(switch2, Use5Vrelay == true ? HIGH : LOW);
        //client.publish(mqttSwitch2Topic,"", true);
      }
      if (String(topic)==mqttSwitch2MomentaryTopic && fullPayload=="pushed") {
        digitalWrite(switch2, Use5Vrelay == true ? LOW : HIGH);
        delay(300);
        digitalWrite(switch2, Use5Vrelay == true ? HIGH : LOW);
        client.publish(mqttSwitch2MomentaryTopic,"momentary_done", true);
        client.publish(mqttSwitch2MomentaryTopic,"", true);
      }
  }
} // MQTT CALLBACK

void reconnect() { // MQTT RECONNECT
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32-";
    clientId += String(mqttDeviceName);
    clientId += "-";
    clientId += String(random(0xffff), HEX);
    Serial.print(clientId);
    // Attempt to connect
    if (client.connect(clientId.c_str(),mqttSwitch1StateTopic,0,false,"LWT disconnected")) {
      Serial.println("...connected");
      // Once connected, publish an announcement...
      client.publish(mqttSwitch1StateTopic, "connected");
      client.publish(mqttSwitch2StateTopic, "connected");
      // ... and resubscribe
      client.subscribe(mqttSwitch1Topic);
      client.subscribe(mqttSwitch2Topic);
      client.subscribe(mqttSwitch1MomentaryTopic);
      client.subscribe(mqttSwitch2MomentaryTopic);
      mqtt_reconnect_count = 0;
    } else {
      if ( mqtt_reconnect_count >= 12 ) {
        #if defined (ESP32) || defined (ESP8266)
          ESP.restart();
        #elif defined(__AVR__)
          while(1) {};
        #endif
      } // REBOOT IN 1 MINUTE

      //if ( mqtt_reconnect_count >= 12 ) { setup_wifi(true); } // RESET WIFI IN 1 MINUTE
      mqtt_reconnect_count++;
      Serial.print(" - failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
} // MQTT RECONNECT


String uptime() {
  float d, hr, m, s;
  String dstr, hrstr, mstr, sstr;
  unsigned long currentMillis, over;
  currentMillis = millis();
  d = int( currentMillis / (3600000 * 24));
  dstr = String(d, 0);
  dstr.replace(" ", "");
  over = currentMillis % (3600000 * 24);
  hr = int(over / 3600000);
  hrstr = String(hr, 0);
  if (hr<10) { hrstr = hrstr = "0" + hrstr; }
  hrstr.replace(" ", "");
  over = over % 3600000;
  m = int(over / 60000);
  mstr = String(m, 0);
  if (m<10) { mstr = mstr = "0" + mstr; }
  mstr.replace(" ", "");
  over = over % 60000;
  s = int(over / 1000);
  sstr = String(s, 0);
  if (s<10) { sstr = "0" + sstr; }
  sstr.replace(" ", "");
  if (d == 0) {
    return hrstr + ":" + mstr + ":" + sstr;
  } else if (d == 1) {
    return dstr + " Day " + hrstr + ":" + mstr + ":" + sstr;
  } else if (d > 1 && d <= 99 ) {
    return dstr + " Days " + hrstr + ":" + mstr;
  } else if (d >= 100 ) {
    return dstr + " D " + hrstr + ":" + mstr;
  } else {
    return dstr + " D " + hrstr + ":" + mstr;
  }
}


float probeDHT(int whichSensor) {
  // Reading temperature or humidity takes about 250 milliseconds. Sensor readings may also be up to 2 seconds old
  #ifdef useDHT
    float h = -1000;
    float tc = -1000;
    int counter = 1;
    if (whichSensor == 0) {
      h = dht.readHumidity();
      while (counter <= 3 && (isnan(h) || h == -1000)) {
        if (isnan(h) || h == -1000) { h = dht.readHumidity(); } // re-read
        counter += 1; delay(50);
      }
    }
    else if (whichSensor == 1) {
      tc = dht.readTemperature();
      while (counter <= 3 && (isnan(tc) || tc == -1000)) {
        if (isnan(tc) || tc == -1000) { tc = dht.readTemperature(); } // re-read
        counter += 1; delay(50);
      }
    }
    if (whichSensor == 0) {
      if (isnan(h) || h == -1000) { return -1000; }
        else { return h; }
    } else if (whichSensor == 1) {
      if (isnan(tc) || tc == -1000) { return -1000; }
        else { return tc; }
    }
  #else
    return 0;
  #endif
  return -1;
}

void outputDHT(unsigned long varMillis) { // OUTPUT DHT
  #ifdef useDHT
    //unsigned long currentMillis = millis();
    float h = probeDHT(0)+humidityOffset;
    float tc = probeDHT(1);
    float tf = int((tc * 9.0 / 5.0) + 32.0)+temperatureOffset;
    //float tf = round(tc*10)/10; // comment out this line or one above for Celcius conversion and see below comments for more changes
    char buf[8];
    //Serial.println (round(lastTemperaturePayload)); Serial.println (tf); Serial.println (round(tf));
    if ( ( varMillis>15000 && tc != -1000 ) && ( ( int(lastTemperaturePayload)!=int(tf) ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Fahrenheit
    //if ( ( varMillis>15000 && tc != -1000 ) && ( ( lastTemperaturePayload)!=tc ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Celcius
      //client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 1, buf), true);  // uncomment for Celcius
      client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 0, buf), true);  // uncomment for Fahrenheit
      lastTemperaturePayload=int(tf); // remove int for Celcius and add it for F
      //Serial.println ("mqttTemperature published");
    } else if (tc == -1000) {
      Serial.println ("DHT Temperature Reading Failed.");
    }
    //Serial.println (round(lastHumidityPayload)); Serial.println (h); Serial.println (round(h));
    if ( ( varMillis>15000 && h!=-1000 ) && ( ( int(lastHumidityPayload)!=int(h) && abs(lastHumidityPayload-h)>2 ) || ( varMillis % 300000 < 200 || lastHumidityPayload==-1 ) ) ) {
      //Serial.println ("mqttHumidity published");
      client.publish(mqttHumidityTopic,dtostrf(int(h), 0, 0, buf), true);
      lastHumidityPayload=int(h);
    } else if (h == -1000) {
      Serial.println ("DHT Humidity Reading Failed.");
    }
  #endif
} // OUTPUT DHT


void outputBME280(unsigned long varMillis) { // OUTPUT BME280
  #ifdef useBME280
    //unsigned long currentMillis = millis();
    float h = bme.readHumidity()+humidityOffset;
    float tc = bme.readTemperature();
    float tf = int((tc * 9.0 / 5.0) + 32.0)+temperatureOffset;
    //float tf = round(tc*10)/10;; // comment out the line below and uncomment this line for Celcius conversion and see below for more changes
    char buf[8];
    //Serial.println (round(lastTemperaturePayload)); Serial.println (tf); Serial.println (round(tf));
    //if (tc < 10) { Serial.print("----------BAD TEMP----------   "); Serial.println(tc); }
    if ( ( varMillis>15000 && tc > -80 && tc!=0.00) && ( ( int(lastTemperaturePayload)!=int(tf) ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Fahrenheit
    //if ( ( varMillis>15000 && tc > -80 && tc!=0.00) && ( ( lastTemperaturePayload)!=tc ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Celcius
      //client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 1, buf), true);  // uncomment for Celcius
      client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 0, buf), true);  // uncomment for Fahrenheit
      lastTemperaturePayload=int(tf); // remove int for Celcius and add it for F
      //Serial.println ("mqttTemperature published");
    }
    //Serial.println (round(lastHumidityPayload)); Serial.println (h); Serial.println (round(h));
    if ( ( varMillis>15000 && h!=0 && h!=100 && h!=2147483647 ) && ( ( int(lastHumidityPayload)!=int(h) && abs(lastHumidityPayload-h)>2 ) || ( varMillis % 300000 < 200 || lastHumidityPayload==-1 ) ) ) {
      client.publish(mqttHumidityTopic,dtostrf(int(h), 0, 0, buf), true);
      lastHumidityPayload=int(h);
      //Serial.println ("mqttHumidity published");
    }
  #endif
} // OUTPUT BME280

void outputESP32Temp(unsigned long varMillis) { // OUTPUT ESP32TEMP
  #ifdef useESP32Temp
    float tf = temprature_sens_read()+temperatureOffset;
    float tc = (tf - 32)/1.8;
    //float tf = round(tc*10)/10;; // comment out the line below and uncomment this line for Celcius conversion and see below for more changes
    //Serial.println (round(lastTemperaturePayload)); Serial.println (tf); Serial.println (round(tf));
    if ( ( varMillis>15000 && tc > -80 ) && ( ( int(lastTemperaturePayload)!=int(tf) ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Fahrenheit
    //if ( ( varMillis>15000 && tc > -80 ) && ( ( lastTemperaturePayload)!=tc ) || ( varMillis % 300000 < 200 || lastTemperaturePayload==-1 ) ) ) { // uncomment for Celcius
      char buf[8];
      //client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 1, buf), true);  // uncomment for Celcius
      client.publish(mqttTemperatureTopic,dtostrf(tf, 0, 0, buf), true);  // uncomment for Fahrenheit
      lastTemperaturePayload=int(tf); // remove int for Celcius and add it for F
      //Serial.println ("mqttTemperature published");
    }
  #endif
} // OUTPUT ESP32TEMP


void DisplayScene(unsigned long varMillis, int whichScene) {
  #if defined(useOLED128X64)
    u8g2.clearBuffer();
    if (whichScene==1) {
      // NAME & IP
      u8g2.drawFrame(0,0,128,20);
      u8g2.setFont(u8g2_font_crox1tb_tf);
      u8g2.drawStr(3,14,mqttDeviceName);
      u8g2.setFont(u8g2_font_profont11_tf);
      u8g2.drawStr(0,34,"IP: ");
      #if defined (ESP32) || defined (ESP8266)
        u8g2.drawStr(25,34,(char*) WiFi.localIP().toString().c_str());
      #elif defined(__AVR__)
        u8g2.drawStr(25,34,(char*) String(Ethernet.localIP().c_str()));
      #endif
      
      // UPTIME
      char upt[32];
      uptime().toCharArray(upt, 32);
      u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
      u8g2.drawStr(0,48,"Uptime:");
      u8g2.drawStr(0,60,upt);
    } else if (whichScene==2) {
      // TEMP & HUMIDITY
      #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
        if (lastTemperaturePayload != -1) {
          char tTemp[5];
          String(int(lastTemperaturePayload)).toCharArray(tTemp, 5); // remove int for Celcius and add int for F
          char temp[7];
          strcpy(temp,tTemp);
          strcat(temp,"\xB0");
          strcat(temp,"F");
          u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
          u8g2.drawStr(0,12,"Temp: ");
          u8g2.drawStr(50,12,temp);
        }
      #endif
      #if defined(useDHT) || defined(useBME280)
        if (lastHumidityPayload != -1) {
          char tHmdt[4];
          String(int(lastHumidityPayload)).toCharArray(tHmdt, 4);
          char hmdt[5];
          strcpy(hmdt,tHmdt);
          strcat(hmdt,"%");
          u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
          u8g2.drawStr(0,24,"Humidity: ");
          u8g2.drawStr(80,24,hmdt);
        }
      #endif
    } else if (whichScene==3) {
      u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
      if (Use5Vrelay == true ) {
        u8g2.drawStr(0,12,digitalRead(switch1) ? "Switch 1: off" : "Switch 1: on");
        u8g2.drawStr(0,24,digitalRead(switch2) ? "Switch 2: off" : "Switch 2: on");
      } else {        
        u8g2.drawStr(0,12,digitalRead(switch1) ? "Switch 1: on" : "Switch 1: off");
        u8g2.drawStr(0,24,digitalRead(switch2) ? "Switch 2: on" : "Switch 2: off");
      }
      u8g2.setFont(u8g2_font_6x10_tf);
      #ifdef CONTACTPIN1
        u8g2.drawStr(0,36,digitalRead(CONTACTPIN1) ? "Contact 1: open" : "Contact 1: closed");
      #endif
      #ifdef CONTACTPIN2
        u8g2.drawStr(0,48,digitalRead(CONTACTPIN2) ? "Contact 2: open" : "Contact 2: closed");
      #endif
    }
    u8g2.sendBuffer();          // transfer internal memory to the display
  #endif

  #if defined(useOLED128X32)
    u8g2.clearBuffer();
    if (whichScene==1) {
      // NAME & IP
      u8g2.drawFrame(0,0,128,20);
      u8g2.setFont(u8g2_font_crox1tb_tf);
      u8g2.drawStr(3,14,mqttDeviceName);
      u8g2.setFont(u8g2_font_profont11_tf);
      u8g2.drawStr(0,32,"IP: ");

      #if defined (ESP32) || defined (ESP8266)
        u8g2.drawStr(25,32,(char*) WiFi.localIP().toString().c_str());
      #elif defined(__AVR__)
        u8g2.drawStr(25,32,(char*) String(Ethernet.localIP()).c_str());
      #endif
      
    } else if (whichScene==2) {
      // UPTIME
      char upt[32];
      uptime().toCharArray(upt, 32);
      u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
      u8g2.drawStr(0,14,"Uptime:");
      u8g2.drawStr(0,30,upt);
    } else if (whichScene==3) {
      // TEMP & HUMIDITY
      #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
        if (lastTemperaturePayload != -1) {
          char tTemp[5];
          String(int(lastTemperaturePayload)).toCharArray(tTemp, 5); // remove int for Celcius and add int for F
          char temp[7];
          strcpy(temp,tTemp);
          strcat(temp,"\xB0");
          strcat(temp,"F");
          u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
          u8g2.drawStr(0,14,"Temp: ");
          u8g2.drawStr(50,14,temp);
        }
      #endif
      #if defined(useDHT) || defined(useBME280)
        if (lastHumidityPayload != -1) {
          char tHmdt[4];
          String(int(lastHumidityPayload)).toCharArray(tHmdt, 4);
          char hmdt[5];
          strcpy(hmdt,tHmdt);
          strcat(hmdt,"%");
          u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);
          u8g2.drawStr(0,30,"Humidity: ");
          u8g2.drawStr(80,30,hmdt);
        }
      #endif
    } else if (whichScene==4) {
      u8g2.setFont(u8g2_font_pxplusibmcgathin_8f);

      if (Use5Vrelay == true ) {
        u8g2.drawStr(0,14,digitalRead(switch1) ? "Switch 1: off" : "Switch 1: on");
        u8g2.drawStr(0,30,digitalRead(switch2) ? "Switch 2: off" : "Switch 2: on");
      } else {        
        u8g2.drawStr(0,14,digitalRead(switch1) ? "Switch 1: on" : "Switch 1: off");
        u8g2.drawStr(0,30,digitalRead(switch2) ? "Switch 2: on" : "Switch 2: off");
      }
    } else if (whichScene==5) {
      u8g2.setFont(u8g2_font_6x10_tf);
      #ifdef CONTACTPIN1
        u8g2.drawStr(0,14,digitalRead(CONTACTPIN1) ? "Contact 1: open" : "Contact 1: closed");
      #endif
      #ifdef CONTACTPIN2
        u8g2.drawStr(0,30,digitalRead(CONTACTPIN2) ? "Contact 2: open" : "Contact 2: closed");
      #endif
    }
    u8g2.sendBuffer();          // transfer internal memory to the display
  #endif

  #if defined(useLCD2004)
    if (millis() > lcdNextRefresh) {
      lcd.clear(); //initialize the lcd
      if (whichScene==1) {
        lcd.setCursor(0,0);
        lcd.print(mqttDeviceName);
        lcd.setCursor(0,1);
        String serverIP = "";
        #if defined (ESP32) || defined (ESP8266)
          serverIP = WiFi.localIP().toString();
        #elif defined(__AVR__)
          serverIP = String(Ethernet.localIP());
        #endif

        lcd.print("IP: " + serverIP);
        lcd.setCursor(0,2);
        lcd.print("Uptime: ");
        lcd.setCursor(0,3);
        lcd.print(uptime());
      } else if (whichScene==2) {
        // TEMP & HUMIDITY
        #if defined(useDHT) || defined(useBME280) || defined(useESP32Temp)
          if (lastTemperaturePayload != -1) {
            lcd.setCursor(0,0);
            lcd.print("Temperature: " + String(int(lastTemperaturePayload)) + char(223) + "F");
          }
        #endif
        #if defined(useDHT) || defined(useBME280)
          if (lastHumidityPayload != -1) {
            lcd.setCursor(0,1);
            lcd.print("Humidity: " + String(int(lastHumidityPayload)) + "%");
          }
        #endif
      } else if (whichScene==3) {
        if (Use5Vrelay == true ) {
          lcd.setCursor(0,0);
          lcd.print(digitalRead(switch1) ? "Switch 1: off" : "Switch 1: on");
          lcd.setCursor(0,1);
          lcd.print(digitalRead(switch2) ? "Switch 2: off" : "Switch 2: on");
        } else {        
          lcd.setCursor(0,0);
          lcd.print(digitalRead(switch1) ? "Switch 1: on" : "Switch 1: off");
          lcd.setCursor(0,1);
          lcd.print(digitalRead(switch2) ? "Switch 2: on" : "Switch 2: off");
        }
        #ifdef CONTACTPIN1
          lcd.setCursor(0,2);
          lcd.print(digitalRead(CONTACTPIN1) ? "Contact 1: open" : "Contact 1: closed");
        #endif
        #ifdef CONTACTPIN2
          lcd.setCursor(0,3);
          lcd.print(digitalRead(CONTACTPIN2) ? "Contact 2: open" : "Contact 2: closed");
        #endif
      }
      lcdNextRefresh = millis()+1000;
    }
  #endif
}