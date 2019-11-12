#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>


  #define led 22
  #define STEPS 48
  #define SLEEP   12
  #define HALL_SENSOR_Tvhome_Front  36
  //#define HALL_SENSOR_Tvhome_Back 
  //#define HALL_SENSOR_LIMIT_Top
  //#define HALL_SENSOR_LIMIT_BOTTOM
  #define RELAY_PIN_1 32
  #define RELAY_PIN_2 33
#else
  #include <ESP8266WiFi.h>
  #define RF_RECEIVER 5
  #define RELAY_PIN_1 4
  #define RELAY_PIN_2 14
#endif
#include "fauxmoESP.h"
#include <stepper.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <IRrecv.h>
#include <JC_Button.h>
#include <ESP32Encoder.h>

#define SERIAL_BAUDRATE 115200

#define WIFI_SSID "NETGEAR28"
#define WIFI_PASS "sweetnest364"

#define LAMP_1 "lamp one"
#define LAMP_2 "lamp two"
#define TV_ROTATION "rotate tv"
#define BLUE_LIGHT_SPECIAL "blue light special"
fauxmoESP fauxmo;


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
Stepper stepper(STEPS, 13, 2); // Pin 13 connected to DIRECTION & Pin 2 connected to STEP Pin of Driver
#define motorInterfaceType 1
const uint16_t kRecvPin = 15;

IRrecv irrecv(kRecvPin);

decode_results results;

//////////flags/////////////
bool shouldMove = false;
bool shouldHome = false;
bool shouldPowerOn = false;
bool shouldPowerOff = false;



////////////////////////////
void tvPowerOn(){
  //power up procedure here
}

void moveleft(){
  if(shouldMove == true){
    stepper.setSpeed(200);
    digitalWrite(SLEEP, HIGH);
    stepper.step(48);
    delay(2);
    digitalWrite(SLEEP, LOW);
    shouldMove = false;
    }
}

void tvPowerOff(){
  //power down procedure here
}

void homefunction() {
  //if(shouldHome == true){
  while (digitalRead(HALL_SENSOR_Tvhome_Front)== 1) {
    digitalWrite(SLEEP, HIGH);
    stepper.step(-360);
    
  }
  //}
}


void flash(){//test for functonality, remove once completed
digitalWrite(led, HIGH);
delay(1);
digitalWrite(led, LOW);
delay(1000);
digitalWrite(led, HIGH);
delay(1000);
digitalWrite(led, LOW);
}


// Wi-Fi Connection
void wifiSetup() {
  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);

  // Connect
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Connected!
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void setup() {
  // Init serial port and clean garbage
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();

  // Wi-Fi connection
  wifiSetup();

  irrecv.enableIRIn();
  pinMode(led, OUTPUT);
  pinMode(HALL_SENSOR_Tvhome_Front, INPUT);
  //pinMode(HALL_SENSOR_Tvhome_Back, INPUT);
  //pinMode(HALL_SENSOR_LIMIT_Top, INPUT);
  //pinMode(HALL_SENSOR_LIMIT_BOTTOM, INPUT);
  pinMode(SLEEP, OUTPUT);

  // LED
  pinMode(RELAY_PIN_1, OUTPUT);
  digitalWrite(RELAY_PIN_1, HIGH);

  pinMode(RELAY_PIN_2, OUTPUT);
  digitalWrite(RELAY_PIN_2, HIGH);
  
   // Receiver on interrupt 0 => that is pin #2

  // By default, fauxmoESP creates it's own webserver on the defined port
  // The TCP port must be 80 for gen3 devices (default is 1901)
  // This has to be done before the call to enable()
  fauxmo.createServer(true); // not needed, this is the default value
  fauxmo.setPort(80); // This is required for gen3 devices

  // You have to call enable(true) once you have a WiFi connection
  // You can enable or disable the library at any moment
  // Disabling it will prevent the devices from being discovered and switched
  fauxmo.enable(true);
  // You can use different ways to invoke alexa to modify the devices state:
  // "Alexa, turn lamp two on"

  // Add virtual devices
  fauxmo.addDevice(LAMP_1);
  fauxmo.addDevice(LAMP_2);
  fauxmo.addDevice(TV_ROTATION);
  fauxmo.addDevice(BLUE_LIGHT_SPECIAL);//test device remove on final version
  
  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value){
 
  
   // Callback when a command from Alexa is received. 
    // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
    // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    // If you have to do something more involved here set a flag and process it in your main loop.
   
    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
    if ( (strcmp(device_name, LAMP_1) == 0) ) {
      // this just sets a variable that the main loop() does something about
      Serial.println("RELAY 1 switched by Alexa");
      //digitalWrite(RELAY_PIN_1, !digitalRead(RELAY_PIN_1));
      if (state) {
        digitalWrite(RELAY_PIN_1, HIGH);
      } else {
        digitalWrite(RELAY_PIN_1, LOW);
      }
    }
    if ( (strcmp(device_name, LAMP_2) == 0) ) {
      // this just sets a variable that the main loop() does something about
      Serial.println("RELAY 2 switched by Alexa");
      if (state) {
        digitalWrite(RELAY_PIN_2, HIGH);
      } else {
        digitalWrite(RELAY_PIN_2, LOW);
      }
    }  
    if ( (strcmp(device_name, TV_ROTATION) == 0) ){
      if (state) {
        shouldMove = true;
      }else{
        digitalWrite(SLEEP, LOW);
      }
    }
    if ( (strcmp(device_name, BLUE_LIGHT_SPECIAL) == 0) ){
      if (state) {
        flash();
      }else{
        digitalWrite(led, LOW);
      }
    }
    
  });
}


void loop() {
  // fauxmoESP uses an async TCP server but a sync UDP server
  // Therefore, we have to manually poll for UDP packets
  fauxmo.handle();
  moveleft();
  
//int Hallreading = digitalRead(HALL_SENSOR);  // for reading hall effect senor output, note that this negates home function
//int hallreading = analogRead(HALL_SENSOR);
//Serial.println(hallreading);
//Serial.println(Hallreading);
//delay(2000);
  static unsigned long last = millis();
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }
    
  if(irrecv.decode(&results)) {
  switch(results.value)
  {
    case 0x20DF12ED:
    stepper.setSpeed(200);
    digitalWrite(SLEEP, HIGH);
    stepper.step(48);
    delay(20);
    digitalWrite(SLEEP, LOW);
    break;

    case 0x20DFE21D:
  stepper.setSpeed(200);  
  digitalWrite(SLEEP, HIGH);  
  stepper.step(-48);
  delay(20);
  digitalWrite(SLEEP, LOW);
  break;

  case 0x20DF10EF:
  //powerbutton
  break;

  case 0x20DF22DD:
  //play/pause/select button
  homefunction();
  if(digitalRead(HALL_SENSOR_Tvhome_Front)==0){
    digitalWrite(SLEEP, LOW);
  }
  break;
  }
irrecv.resume();
  delay(1);
 }  
 
}