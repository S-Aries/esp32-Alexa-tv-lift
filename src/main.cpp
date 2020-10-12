/* Code by Scot-Alan, open for public use, not to be monetized! */


#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #define led 22
  #define STEPS 48
  #define SLEEP   12
  #define HALL_SENSOR_Tvhome_Front  36
  //#define HALL_SENSOR_Tvhome_Back 39
  #define HALL_SENSOR_LIMIT_Top 34
  #define HALL_SENSOR_LIMIT_BOTTOM 35
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
#include <Encoder.h>
#include <HardwareSerial.h>
#include "soc/uart_reg.h"
#include "esp32_rmt.h"
#define SERIAL_BAUDRATE 115200

#define WIFI_SSID ""
#define WIFI_PASS ""

#define LAMP_1 "lamp one"
#define LAMP_2 "lamp two"
#define TV_ROTATION "rotate tv"
#define TV "tv"
#define JUST_TV "just the tv"
//#define BLUE_LIGHT_SPECIAL "blue light special"
fauxmoESP fauxmo;


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver
Stepper stepper(STEPS, 13, 2); // Pin 13 connected to DIRECTION & Pin 2 connected to STEP Pin of Driver
#define motorInterfaceType 1
//const byte interruptPin[] = {21,23};

const int RELAY[] = {14,27};  //RELAY[0] and RELAY[1] to access the pins; physicaly pins 31,30 respectively
const int BTN_EXTEND = 0; //physicaly pin 6
const int BTN_RETRACT = 4; //physical pin 7
const uint8_t MANUAL = 1;    //a constant to indicate manual mode
const uint8_t AUTOMATIC = 2; //a constant to indicate automatic mode
const int BTN_MEM_PIN[] = {16,17,5}; //physicaly pins 8,9,10 respectively
const int BTN_SET_MEM = 18; //physical pin 11

const uint16_t kRecvPin = 15;

IRrecv irrecv(kRecvPin);

decode_results results;

//int stauts[] = {0};
//Set up the linear actuator encoder
//On many of the Arduino boards pins 2 and 3 are interrupt pins
// which provide the best performance of the encoder data.
Encoder myEnc(21, 23); //physicaly pins 14,18 respectively 
long oldPosition  = -999;
long targetPosition = 0;
#define ACCURACY 10       //How close to your target position is close enough. Higher accuracy may result in 
               // a bit of jitter as the actuator nears the position
#define DEBOUNCE_MS 20     //A debounce time of 20 milliseconds usually works well for tactile button switches.
//#define PULLUP false        //To keep things simple, we use the Arduino's internal pullup resistor.
#define INVERT true        //Since the pullup resistor will keep the pin high unless the
               //switch is closed, this is negative logic, i.e. a high state
               //means the button is NOT pressed. (Assuming a normally open switch.)
uint8_t MODE = MANUAL;

Button btnExtend(BTN_EXTEND, PULLUP, INVERT, DEBOUNCE_MS);
Button btnRetract(BTN_RETRACT, PULLUP, INVERT, DEBOUNCE_MS);
Button btnSetPos(BTN_SET_MEM, PULLUP, INVERT, DEBOUNCE_MS);
Button btnPos1(BTN_MEM_PIN[0], PULLUP, INVERT, DEBOUNCE_MS);
Button btnPos2(BTN_MEM_PIN[1], PULLUP, INVERT, DEBOUNCE_MS);
Button btnPos3(BTN_MEM_PIN[2], PULLUP, INVERT, DEBOUNCE_MS);


long memPosition[] = {0,0,0};
void retractActuator();
void extendActuator();
void stopActuator();





//////////flags/////////////
bool shouldMove = false;
bool shouldHome = false;
bool shouldPowerOn = false;
bool shouldPowerOff = false;
bool shouldPowerCycle = false;


////////////////////////////
void tvPowerOn(){
  //power up procedure here
  if (shouldPowerOn == true){
    MODE = AUTOMATIC;
    targetPosition = memPosition[2];
    //TODO add hall effect limit code
    
  
  if (digitalRead(HALL_SENSOR_LIMIT_Top)== 0){
    stepper.setSpeed(200);
    digitalWrite(SLEEP, HIGH);
    stepper.step(96);
    delay(2);
    digitalWrite(SLEEP, LOW);
    //TODO add send power signal to tv here
    /*Serial.begin(115200);
    Serial2.begin(115200);
    HardwareSerial Serial2 (2); //gpio 26, physically pin 29
    WRITE_PERI_REG(0x20DF10EF,);*/
    shouldPowerOn = false;
    }

  }
}

//add a function to cycle tv power, just in case

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
  if(shouldPowerOff == true){
  //power down procedure here
  //TODO add send power signal to tv here
  //homefunction();
  while (digitalRead(HALL_SENSOR_Tvhome_Front)== 1) {
    digitalWrite(SLEEP, HIGH);
    stepper.step(-360);
  }
  if (digitalRead(HALL_SENSOR_Tvhome_Front)== 0){
  MODE = AUTOMATIC;
  targetPosition = memPosition[0];
  //TODO add hall effect limit switch here
  shouldPowerOff = false;
  }
  
  }
}

void homefunction() {
  //if(shouldHome == true){
  while (digitalRead(HALL_SENSOR_Tvhome_Front)== 1) {
    digitalWrite(SLEEP, HIGH);
    stepper.step(-360);
    
  }
  //}
}


/*void flash(){//test for functonality, remove once completed
digitalWrite(led, HIGH);
delay(1);
digitalWrite(led, LOW);
delay(1000);
digitalWrite(led, HIGH);
delay(1000);
digitalWrite(led, LOW);
}*/


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

  pinMode(RELAY[0], OUTPUT);
  pinMode(RELAY[1], OUTPUT);
  btnExtend.begin();
  btnRetract.begin();
  btnSetPos.begin();
  btnPos1.begin();
  btnPos2.begin();
  btnPos3.begin();
  //Serial.begin(115200);
  irrecv.enableIRIn();
  pinMode(led, OUTPUT);
  pinMode(HALL_SENSOR_Tvhome_Front, INPUT);
  //pinMode(HALL_SENSOR_Tvhome_Back, INPUT);
  pinMode(HALL_SENSOR_LIMIT_Top, INPUT);
  pinMode(HALL_SENSOR_LIMIT_BOTTOM, INPUT);
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
  fauxmo.addDevice(TV);
  //fauxmo.addDevice(BLUE_LIGHT_SPECIAL);//test device remove on final version
  
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
    if( (strcmp(device_name, TV) == 0) ){
      if (state) {
        shouldPowerOn = true;
      }else{
        shouldPowerOff = true;
      }
    }
    /*if ( (strcmp(device_name, BLUE_LIGHT_SPECIAL) == 0) ){
      if (state) {
        flash();
      }else{
        digitalWrite(led, LOW);
      }
    }*/
    
  });
}


void loop() {
  // fauxmoESP uses an async TCP server but a sync UDP server
  // Therefore, we have to manually poll for UDP packets
  fauxmo.handle();
  moveleft();
  tvPowerOn();
  tvPowerOff();
  btnExtend.read();
  btnRetract.read();
  btnSetPos.read();
  btnPos1.read();
  btnPos2.read();
  btnPos3.read();
//int Hallreading = digitalRead(HALL_SENSOR_LIMIT_Top);  // for reading hall effect senor output, note that this negates home function
//int hallreading = analogRead(HALL_SENSOR);
//Serial.println(hallreading);
//Serial.println(Hallreading);
//delay(2000);
  static unsigned long last = millis();
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }
    
  if (btnExtend.isPressed()) {
    extendActuator();
    MODE = MANUAL;
  }

  if (btnRetract.isPressed()) {
    retractActuator();
    MODE = MANUAL;
  }

  if (!btnExtend.isPressed() && !btnRetract.isPressed() && MODE == MANUAL) {
    stopActuator();
    MODE = MANUAL;
  }

  if(btnPos1.wasReleased()) {
    Serial.println("btnPos1");
    MODE = AUTOMATIC;
    targetPosition = memPosition[0]; 
  }
  if(btnPos2.wasReleased()) {
    Serial.println("btnPos2");
    MODE = AUTOMATIC;
    targetPosition = memPosition[1]; 
  }
  if(btnPos3.wasReleased()) {
    Serial.println("btnPos3");
    MODE = AUTOMATIC;
    targetPosition = memPosition[2]; 
  }


  //check the encoder to see if the position has changed
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  if(MODE == AUTOMATIC && newPosition != targetPosition) {
    Serial.print("Target/Actual:");Serial.print(targetPosition);Serial.print(" / ");Serial.print(newPosition);Serial.print(" [");Serial.print(abs(targetPosition - newPosition));Serial.println("]");
    if(targetPosition > newPosition) {
      Serial.println("AUTO RETRACT");
      retractActuator();
      MODE = AUTOMATIC;
    }
    if(targetPosition < newPosition) {
      Serial.println("AUTO EXTEND");
      extendActuator();
      MODE = AUTOMATIC;
    }
    if( (targetPosition == newPosition) || abs(targetPosition - newPosition) <= ACCURACY) {
      Serial.println("AUTO STOP");
      stopActuator();
      MODE = MANUAL;
    }
  }
  
  if(btnSetPos.isPressed()) {
   if(btnPos1.isPressed())
     memPosition[0] = newPosition;
   if(btnPos2.isPressed())
     memPosition[1] = newPosition;
   if(btnPos3.isPressed())
     memPosition[2] = newPosition;
       
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
 
 if (digitalRead(HALL_SENSOR_LIMIT_BOTTOM)== 1 && digitalRead(HALL_SENSOR_LIMIT_Top)== 0) {
    shouldPowerOff = true;
  }else if (digitalRead(HALL_SENSOR_LIMIT_BOTTOM)== 0 && digitalRead(HALL_SENSOR_LIMIT_Top)== 1) {
    shouldPowerOn = true;
  }
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

void extendActuator() {
  //Serial.println("extendActuator");
  digitalWrite(RELAY[0], HIGH);
  digitalWrite(RELAY[1], LOW);
}

void retractActuator() {
  //Serial.println("retractActuator");
  digitalWrite(RELAY[0], LOW);
  digitalWrite(RELAY[1], HIGH);
}

void stopActuator() {
  //Serial.println("stopActuator");
  digitalWrite(RELAY[0], HIGH);
  digitalWrite(RELAY[1], HIGH);
}
