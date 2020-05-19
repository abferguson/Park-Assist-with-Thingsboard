/* Car Parking Assistant
 Program to use the HC-SR04 ultrasonic sensor to
 measure distance to moving object (car) and blink led
 faster or slower depending on distance.  As car pulls
 into garage and approaches sensor, it starts to blink.
 As car pulls forward more, blinking speed increases.
 Once preset "stop" distance is reached, white led goes
 off and red led goes on.
 A neopixel LED strip is also used to visually indicate
 position of car relative to sensor. When car is far 
 from sensor, all LEDs light. As you apporach the stop
 position, fewer LEDs light until just before stop the
 center LED is lit.

 During startup,the 5mm LED is green until the unit successfully
 joins the network.

 Hardware:
 1 x NodeMCU v1.0 (ESP-12E) microcontroller
 1 x 5mm RGB LED
 3 x 470 Ohm resistor (for RGB LED)
 1 x 10k Ohm resistor (momentary switch)
 1 x momentary contact tactile button
 1 x HC-SR04 ultrasonic sensor
 1 x WS2912b LED strip
 1 x 470 Ohm resistor (WS2912b data line)
 1 x 4 channel level converter (HC-SR04 trigger and echo lines, WS2812b data line)
 1 x 1000 uF capacitor (for WS2812B LED strip)
 
 Firmware:
 Arduino IDE

 Tony Ferguson
 5/13/2020
*/

#include "ThingsBoard.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SR04.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ezTime.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

const char redLED = D1;
const char grnLED = D2;
const char bluLED = D3;
const char LEDpin = D4;
const char TRIG_PIN = D5;
const char ECHO_PIN = D6;
const char BUTTON_PIN = D7;

int fw_version = 1;

const char ssid[] = "Fergnet";
const char WIFI_PASSWORD[] = "ms4e4qyrlkbc";

const char node_name[] = "ferg_park_assist_subaru";
const char OTAname[] = "park_assist_subaru";

const char THINGSBOARD_SERVER[] = "192.168.1.107";
const char TOKEN[] = "MbJMlafjemYqUc6iOFIf"; // park assist device token

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
int distance; // cm
float distanceft; // ft
int oldDistance = 15; // cm
int dialstopdist = 10; // cm, dial value
int memstopdist = 61; // default
bool ledred = false;
bool ledblu = false;

int begindist = 305; // cm
int upperdelay = 80; // msec
int lowerdelay = 0; // msec
int delayval;
bool subscribed = false;
bool Rebootstat = true;
unsigned long currentMillis = 0;
long Delaymillis = 2000;
unsigned int currentAttrMillis = 0;
int DelayAttrMillis = 5000;
int delayAttrSec = DelayAttrMillis / 1000;
bool commitFalse = false;

bool closeTrigger = false;
bool middleTrigger = false;
bool farTrigger = false;
bool sleepLEDs = false;

//String IPaddress;
int status = WL_IDLE_STATUS;

WiFiClient espClient; // Initialize ThingsBoard client

ThingsBoardSized<400, 10> tb(espClient); // Initialize ThingsBoard instance

const char server[] = "http://192.168.1.107:8080/api/v1/";
const char clientKeys[] = "/attributes?clientKeys=bootTime";
char getlinkClient[200];

Timezone myTZ;
unsigned long thingsboard_timestamp;
unsigned long bootTime;
float rebootInterval;

int numLEDs = 21;
Adafruit_NeoPixel strip(numLEDs, LEDpin, NEO_GRB + NEO_KHZ800);

void(* resetFunc) (void) = 0; // must be at top. Declare reset function @ address 0

//----------------------------------------------------------------------------------
void setup(){
  delay(10);
  pinMode(LEDpin, OUTPUT);
  uint32_t color = strip.Color(0,0,0);
  strip.setBrightness(20);
  strip.fill(color,0);
  strip.show();

  Serial.begin(115200);
  Serial.println("");
  Serial.println("");

  pinMode(redLED, OUTPUT);
  pinMode(grnLED, OUTPUT);
  pinMode(bluLED, OUTPUT);
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, HIGH);
  digitalWrite(bluLED, LOW);

  
  SPIFFS.begin();
  ReadFromSPIFFS();
  
  WiFi.hostname(node_name);
  InitWiFi();
  InitNTP();  
  subscribeThingsboard();

  strcat(getlinkClient, server);
  strcat(getlinkClient, TOKEN);
  strcat(getlinkClient,clientKeys);
  Serial.print("  getlinkClient: ");
  Serial.println(getlinkClient);

  getClientAttributes(getlinkClient);

  float rbinterval = thingsboard_timestamp - bootTime;
  rebootInterval = rbinterval/86400.0; // days

  Attribute attributesSetup[6] = {
    { "bootTime", thingsboard_timestamp },
    { "rebootInterval", rebootInterval },
    { "fw_version", fw_version },
    { "rebootstate", Rebootstat },
    { "delayattrsec", delayAttrSec },
    { "commitFalse", commitFalse },
  };
  tb.sendAttributes(attributesSetup, 6);
 
  delay(2000);
  digitalWrite(grnLED, LOW);

  pinMode(BUTTON_PIN, INPUT);  // button pin setup
  int buttonState = LOW;
  
  OTAsetup();

  Serial.println("Going into loop");
} // void setup()

//----------------------------------------------------------------------------------

// set value on switch 1 (Reboot toggle)
RPC_Response processSetSW1(const RPC_Data &data){
  Rebootstat = false;  // turn off reboot led
  Attribute attributesSetSW1[1] = {
    { "rebootstate", Rebootstat },
  };
  tb.sendAttributes(attributesSetSW1, 1);
  return RPC_Response(NULL, Rebootstat);
}

// get switch 1 (Reboot toggle)
RPC_Response processGetSW1(const RPC_Data &data){
  return RPC_Response(NULL, Rebootstat);
}

// set value on stop distance control knob
RPC_Response processSetStopDist(const RPC_Data &data){
  dialstopdist = data;
  Attribute attributesDial[1] = {
      { "stopdist", dialstopdist },
  };
  tb.sendAttributes(attributesDial, 1);
  return RPC_Response("setstop", dialstopdist);
}

// get value of stop distance control knob
RPC_Response processGetStopDist(const RPC_Data &data){
  return RPC_Response(NULL, dialstopdist);
}

// save stop distance to nodemcu memory (Commit Stop Distance)
RPC_Response processSetSW2(const RPC_Data &data){
  memstopdist = dialstopdist;
  Attribute attributesCommit[2] = {
    { "memstopdist", memstopdist },
    { "commitFalse", commitFalse },
  };
    tb.sendAttributes(attributesCommit, 2);
  WriteToSPIFFS();
  return RPC_Response(NULL, commitFalse);
}

// get switch 2 (Reboot toggle)
RPC_Response processGetSW2(const RPC_Data &data){
  Serial.println("Set reboot toggle to off");
  return RPC_Response(NULL, commitFalse);
}

int callbacksize = 6;
RPC_Callback callbacks[6] = {
  { "setSW1", processSetSW1 },
  { "getSW1", processGetSW1 },
  { "setStopDist", processSetStopDist },
  { "getStopDist", processGetStopDist },
  { "setSW2", processSetSW2 },
  { "getSW2", processGetSW2 },
};

//=======================================================================
void loop(){
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi reconnecting");
    reconnect();
  }

  subscribeThingsboard();
  subscribeRPC();
  
  getSR04Distance();  
  ledHandler();
  checkButton();
  AttributeUpdate();
  tb.loop();
} // void loop()
//=======================================================================

void getSR04Distance() {
  distance = sr04.Distance();
  distanceft = distance*0.0328084;
  if (distance <= 0) {
    Serial.print("bad sensor. Value = ");
    Serial.println(distance);
    ledblu = true;
  }
  else {
    ledblu = false;
  }
} // void getSR04Distance()

void checkButton() {  // handle momentary button
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    Serial.println("button on");
    dialstopdist = distance;
    memstopdist = distance;
// ,    WriteToSPIFFS();
    Serial.println("Button-saved stop distance to SPIFFS memory:");
    Serial.println(memstopdist);
  }
}  // void checkButton()

void ledHandler() {
  if(distance != oldDistance) {
//    Serial.print("ledHandler: oldDistance, distance: ");
//    Serial.print(oldDistance);
//    Serial.print(", ");
//    Serial.println(distance);
    oldDistance = distance;
//    Serial.println("ledHandler: distance != oldDistance");
  }
  
  if(distance <= memstopdist && ledblu == false) {  // determine if red LED intensity is high or low
    ledred = true;
    if(!closeTrigger) { // on first call to < memstopdist, trigger
      currentMillis = millis();  // reset timer if distance changes
      sleepLEDs = false;
    }
    if(millis() - currentMillis < 5000) {
//      Serial.println("ledHandler: <5 sec since distance change");
      digitalWrite(redLED, HIGH);

      uint32_t color = strip.Color(255,0,0);
      strip.fill(color,0);
      strip.show();
    }
    else {
//      Serial.println("ledHandler: >5 sec since distance change");
      if(!sleepLEDs) { // execute once after 5 sec delay
        int j = random(1,4);
        switch (j) {
          case 1:
            theaterChaseRainbow(40);
            break;
          case 2:
             rainbow(5);
             break;
          case 3:
            for (int i = 1; i < 4; i++) {
               CylonBounce(255,0,0,3,50,50);
            }
            break;
          case 4:
            for (int i = 1; i<300; i++) {
              Sparkle(random(255),random(255),random(255),10);
            }
            break;
        }
         
        strip.clear();
        strip.show();
        analogWrite(redLED, 100); // do after led strip to minimize pwm interference
        sleepLEDs = true;
      }
    }
    digitalWrite(grnLED, LOW);
    digitalWrite(bluLED, LOW);

    delay(100); // slow down SR04 reads when car is in park range
    closeTrigger = true;
    middleTrigger = false;
    farTrigger = false;
  } // < memstopdist
  
  if (distance > memstopdist && distance <= begindist) {
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED, HIGH);
    digitalWrite(bluLED, HIGH);
    delay(10);
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(bluLED, LOW);
    ledred = false;
    delayval = (distance-begindist)*(lowerdelay-upperdelay)/(memstopdist-begindist)+upperdelay;
//   Serial.write(delayval);
    delay(delayval);

    int litLEDs = (((numLEDs-1)/2)*(distance-memstopdist))/(begindist-memstopdist)+1;
//    Serial.print("distance ");
//    Serial.print(distance);
//    Serial.print(", memstopdist ");
//    Serial.print(memstopdist);
//    Serial.print(", begindist ");
//    Serial.print(begindist);
//    Serial.print(", litLEDs ");
//    Serial.println(litLEDs);
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.show();

    color = strip.Color(0,255,0); // set litLEDs starting at middle
    for (int i = 10; i < ((numLEDs-1)/2)+litLEDs; i++) {
      strip.setPixelColor(i, color);
//      Serial.print(" i=");
//      Serial.print(i);
      strip.setPixelColor(i-litLEDs+1, color);
    }
    strip.show();

    closeTrigger = false;
    middleTrigger = true;
    farTrigger = false;
  } // between

  if ( distance > begindist ) {
//    Serial.println("ledHandler: too far away");
    delay(100);  // slow down sensor reads when car is > begindist (far away)

    if(!farTrigger) { // execute once
      strip.clear();
      strip.show();
    }
    
    closeTrigger = false;
    middleTrigger = false;
    farTrigger = true;
  } // too far away

  if(ledblu) {
    uint32_t color = strip.Color(0,0,30);
    strip.fill(color,0);
    strip.show();
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(bluLED, HIGH);
  }
 
} // void ledHandler()

void getClientAttributes(String location) {
  Serial.print("Get ThingsBoard client attribute...");
  HTTPClient http;  //Declare an object of class HTTPClient
  http.begin(location);  //Specify request destination
  int httpCode = http.GET();  //Send the request
  
  if (httpCode > 0) { //Check the returning code
    const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + 100;
    DynamicJsonDocument doc(capacity);
    const char* json = http.getString().c_str();
    Serial.print("getClientAttributes(): json: ");
    Serial.print(json);
    
    deserializeJson(doc, json);
    JsonObject clientAttr = doc["client"];
    
    bootTime = clientAttr["bootTime"];
  }   
  http.end();   //Close connection
  Serial.println("  getClientAtributes() finished");
} // void getClientAttributes()

void AttributeUpdate() {
  if ((unsigned long)(millis() - currentAttrMillis) >= DelayAttrMillis) {
    Serial.print("AttributeUpdate()...  ");
    
    char ipaddrChar[16];
    WiFi.localIP().toString().toCharArray(ipaddrChar, 16);
    const char * ipaddr = ipaddrChar;

    Attribute attributes[7] = {
      { "stopdist", dialstopdist },
      { "currentdist", distanceft },
      { "memstopdist", memstopdist },
      { "ledred", ledred },
      { "ledblu", ledblu },
      { "ipaddr", ipaddr },
      { "rssi", WiFi.RSSI() },
    };
    tb.sendAttributes(attributes, 7);
 
    currentAttrMillis = millis();
    Serial.println("finished Attributeupdate()");
  }
} // void AttributeUpdate()

void ReadFromSPIFFS() {
  Serial.print("Reading from SPIFFS... ");
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r"); // Open file for reading
  if (!myDataFile) {
    Serial.println("  readspiffs: Failed to open file");
    FirstTimeRun(); // no file there -> initializing
  }
  
  String memstopdistString = myDataFile.readStringUntil('\n');
  memstopdist = memstopdistString.toInt();  
//  Serial.print("  memstopdist from memory: ");
//  Serial.print(memstopdist);
//  Serial.print("cm; ");
//  Serial.print(memstopdist*0.0328084);
//  Serial.print("ft");
  dialstopdist = memstopdist;
  
  myDataFile.close();
  Serial.println("  Success. Completed read from SPIFFS");
} // void ReadFromSPIFFS()

void WriteToSPIFFS() {
  Serial.println("Writing to SPIFFS...");
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w"); // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("  Error: WriteToSPIFFS: Failed to open file");
  }
  
  myDataFile.println(memstopdist);
  
  Serial.println("  Saved stop distance to memory:");
  Serial.print(memstopdist);
  Serial.print("cm; ");
  Serial.print(memstopdist*0.0328084);
  Serial.println("ft");
    
  myDataFile.close();
  Serial.println("  Success. Data written to SPIFFS.");
} // void WriteToSPIFFS()

void FirstTimeRun() {
  Serial.println("Starting SPIFFS FirstTimeRun() ");
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w"); // Open a file for writing
  Serial.print("myDataFile: ");
  Serial.println(myDataFile);
  if (!myDataFile) {
    Serial.println("  FirsTimeRun(): Failed to open file");
    Serial.println("  Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(61); // 61cm / 24in
  Serial.println("  Saved default value (61) of memstopdist");
  myDataFile.close();
  Serial.println("  Doing a system reset now.");
  resetFunc();
}  // void FirstTimeRun()

void InitWiFi(){
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to Fergnet");
  // attempt to connect to WiFi network
  WiFi.begin(ssid, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Success: connected to Fergnet");
 
  Serial.print("  IP Address: ");
  Serial.println(WiFi.localIP().toString());
  
  Serial.print("  signal strength: ");
  Serial.println(WiFi.RSSI());
  
} // void InitWiFi

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    Serial.print("Reconnecting to Fergnet");
    WiFi.begin(ssid, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println(" Success: reconnected to Fergnet");
  }
} // void reconnect

void InitNTP() {
  Serial.print("Starting InitNTP()... ");
  waitForSync();  // ezTime library
  myTZ.setLocation(F("America/Chicago"));
  setInterval(86400);  // NTP server update interval
  thingsboard_timestamp = myTZ.tzTime(myTZ.now(),LOCAL_TIME);  // UTC value for Thingsboard
  Serial.println(" Success: finished InitNTP()");
}  // void InitNTP()

void subscribeThingsboard() {
  if (!tb.connected()) {
    subscribed = false;
    Serial.print("Subscribing to ThingsBoard...");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.print(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println(" Failed to connect. retrying...");
      delay(1000);
      return;
    }
    Serial.println(" Success: ThingsBoard subscribe finished");
  }
} // void subscribeThingsboard()

void subscribeRPC() {
  if (!subscribed) {
    Serial.print("Subscribing to RPC...");
    if (!tb.RPC_Subscribe(callbacks, 6)) {
      Serial.println(" Failed to subscribe to RPC");
      return;
    }
    subscribed = true;
    Serial.println(" Success: RPC subscribing done");
  }
} // void subscribeRPC

void OTAsetup() {
  Serial.print("Starting ArduinoOTA...");
  ArduinoOTA.setHostname(OTAname);
  ArduinoOTA.setPassword("YOUR_PASSWORD_HERE");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("   ArduinoOTA Ready. ");
} // void OTAsetup

//====================================================================================
// Effects

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<90; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
} // theaterChaseRainbow()

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
} // rainbow()

//*
void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){
  for(int i = 0; i < numLEDs-EyeSize-1; i++) {
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.setPixelColor(i,red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue);
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = numLEDs-EyeSize-2; i > 0; i--) {
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.setPixelColor(i, red/10, green/10,blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue);
    }
    strip.setPixelColor(i+EyeSize, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }
 
  delay(ReturnDelay);
} // CylonBounce()
//*/


void Sparkle(byte red, byte green, byte blue, int SpeedDelay) {
  int Pixel = random(numLEDs);
  strip.setPixelColor(Pixel,red,green,blue);
  strip.show();
  delay(SpeedDelay);
  strip.setPixelColor(Pixel,0,0,0);
} // Sparkle()
