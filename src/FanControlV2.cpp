/*
 * Project Fan Controller
 * Description:  FailSafe controller for exhaust fan at Century Signs
 * Author:       Reed Searle
 * Date:         15 January 2024
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "credentials.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
// SYSTEM_MODE(MANUAL);// for testing at home

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_WARN);

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
Adafruit_MQTT_Publish   mqttObj1 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-relayValue");
Adafruit_MQTT_Publish   mqttObj2 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-relayState");
Adafruit_MQTT_Publish   mqttObj3 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-switchState");
Adafruit_MQTT_Publish   mqttObj4 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-VOCReturn");
Adafruit_MQTT_Publish   mqttObj5 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-dataValid");
Adafruit_MQTT_Publish   mqttObj6 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-eveningRun");
Adafruit_MQTT_Publish   mqttObj7 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-fanSpeed");
Adafruit_MQTT_Publish   mqttObj8 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/signcompany-dateTime");


// Adafruit_MQTT_Subscribe theTemperatureObject = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-temperature");
// Adafruit_MQTT_Subscribe theHumidityObject    = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-humidity");
// Adafruit_MQTT_Subscribe theTVOCObject        = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-tvoc");
// Adafruit_MQTT_Subscribe theCO2Object         = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-eco2");
// Adafruit_MQTT_Subscribe theResetObject       = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-reset");

Adafruit_MQTT_Subscribe theTemperatureObject = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/test-temperature");
Adafruit_MQTT_Subscribe theHumidityObject    = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/test-humidity");
Adafruit_MQTT_Subscribe theTVOCObject        = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/test-tvoc");
Adafruit_MQTT_Subscribe theCO2Object         = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/test-eco2");
Adafruit_MQTT_Subscribe theResetObject       = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/test-reset");


/************Declare Variables*************/
const int MILLIS_PER_DAY = 1000*60*60*24;
const int PUBLISH_TIME = 30000;
const int INVALID = -1;

const int RELAY_PIN        = A0; // Input of the relay sampler
const int RELAY_STATE_PIN  = A1; // State of the relay; Manual, Auto, Floating
const int SWITCH_STATE_PIN = A2; // State of the relay; Manual, Auto, Floating

const int WDT_ST_PIN       = D0; // Watchdog timer strobe output on Pin A1
const int WDT_RSTN_PIN     = D1; // Reset(low) from the DS1232. Used for instrumentation
const int TPL5010_WAKE_PIN = D2; // Wake pin from tpl5010
const int TPL5010_DONE_PIN = D3; // Done pin to tpl5010
const int FAN_CTL_OUT_PIN  = D4; // Fan control output pin to OpAmp
const int D7_LED_PIN       = D7; // Onboard LED
const int WDT_PBRSTN_PIN   = D9; // Reset(low) to the  DS1232. Used to control relay through BJT

const int WDT_Half_Time    = 250;//  Half time pulse for WDT.  Used to change the state ot the WDT Strobe output
bool      WDT_ST_State;          //  State of the watchdog timer strobe output

const int  FAN_OFF    = 0;   // 0.0 on manual fan dial
const int  FAN_IDLE   = 65;  // 2.6 on manual fan dial
const int  FAN_NORMAL = 90;  // 3.5 on manual fan dial
const int  FAN_HIGH   = 200; // 8.0 on manual fan dial
long startFanTest;
const long FAN_TEST_DELAY = 5000;
int fanTest[] = {FAN_OFF, FAN_IDLE, FAN_NORMAL,FAN_HIGH};
int fanCount;

bool firstRead;  //  indicator of the first time the subscription has been read since the device turned on

float temperature = INVALID; // temperature variable set to Invalid Data
int   humidity    = INVALID; // humidity variable set to Invalid Data
int   VOC         = INVALID; // VOC variable set to Invalid Data
int   CO2         = INVALID; // CO2 variable set to Invalid Data

bool dataValid;   //  Flag for MQTT subscription data receiving on all four subscriptions
bool eveningRun;  //  Flag for nightly high speed fan

//  Time & timing Variables
u_int64_t last, lastTime;  // These timing variables work with System.millis()
String    dateTime;   

//  Sample Variables
int relaySample;  //  Value of the output of the relay
int relayState;   //  Currrent state of the relay; manual, auto, or floating(BAD)
int switchState;  //  Current selected position of the manual/automatic switch;
int wdtResetnIn;  //  Current reset state of the Watch Dog Timer, DS1232 ACTIVE LOW
bool resetMQTT;   // Forced reset from dashboard through MQTT broker
int fanSpeed;     // Integer value of fan speed sent to MQTT

//  INSTANTIATIONS  //
void watchDogISR();
void MQTT_connect(bool* valid);
bool MQTT_ping(String dateTime);
void MQTT_publish(int relaySample, int relayState, int switchState, int VOC, 
                    bool dataValid, bool eveningRun, int fanSpeed, String dateTime);
void MQTT_subscribe(float* temp, int* hum, int* volit, int* carbon, bool* resetin);
int  fanControl(String dateTime, int hum, int volit, bool dataValid, bool* eveningRun);
void blinkD7(int blinkNum);

Timer watchDogTimer(WDT_Half_Time, watchDogISR);  //  Instantiate a Timer interrupt with period WDT_Half_Time and interrupt service routinte watchDogISR



/////////////////////////////////////////////////////////
//        SETUP
/////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

  pinMode(WDT_ST_PIN,      OUTPUT);
  pinMode(WDT_PBRSTN_PIN,  OUTPUT);
  pinMode(WDT_RSTN_PIN,    INPUT);
  pinMode(FAN_CTL_OUT_PIN, OUTPUT);

  pinMode(TPL5010_DONE_PIN, OUTPUT);
  pinMode(TPL5010_WAKE_PIN, INPUT);

  pinMode(D7_LED_PIN, OUTPUT);

  pinMode(RELAY_PIN, INPUT);
  pinMode(RELAY_STATE_PIN, INPUT);
  pinMode(SWITCH_STATE_PIN, INPUT);


  digitalWrite(WDT_PBRSTN_PIN, HIGH);  //  Force Watchdog into reset
  // Serial.printf("reset High\n");

  WDT_ST_State = 0;
  watchDogTimer.stop();

 
  // Connect to WiFi without going to Particle Cloud
  // WiFi.on();
  // WiFi.clearCredentials();
  // WiFi.setCredentials("CNM_GUEST");
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }

  // Setup MQTT subscription Century Signs Environmental Data.
  mqtt.subscribe(&theTemperatureObject);
  mqtt.subscribe(&theHumidityObject);
  mqtt.subscribe(&theTVOCObject);
  mqtt.subscribe(&theCO2Object);
  mqtt.subscribe(&theResetObject);

  dataValid  = FALSE;  //  Initialize dataValid flag to FALSE
  eveningRun = FALSE;  //  Initialize eveningRun flag to FALSE

  //  Setup TIME
  Particle.connect();
  Time.zone(-6);   //  Set time zone to MST -6 from UTC
  Particle.syncTime();

  watchDogTimer.start();
  last     = millis();
  lastTime = millis();

} // setup


/////////////////////////////////////////////////////////
//        LOOP
/////////////////////////////////////////////////////////
void loop() {
  //  Sync TIME once per day
  if((millis() - last) > MILLIS_PER_DAY) {
    Particle.syncTime();
  }
  dateTime = Time.timeStr();                          //  get current value of date and time

  //  D7 Blink codes.  Add more as required
  if(dataValid) blinkD7(1);
  else blinkD7(3);

  // Read state of input pins
  switchState = analogRead(SWITCH_STATE_PIN);
  relayState  = analogRead(RELAY_STATE_PIN);
  relaySample = analogRead(RELAY_PIN);
  wdtResetnIn = digitalRead(WDT_RSTN_PIN); 

  /*  MQTT Function Block*/
// Validate connected to MQTT Broker
  MQTT_connect(&dataValid); // Check connection.  Trip dataValid flag if connection lost
  MQTT_ping(dateTime);

  // publish to cloud every 30 seconds
  if((System.millis()-lastTime > PUBLISH_TIME)) {
    MQTT_publish(relaySample, relayState, switchState, VOC, dataValid, eveningRun, fanSpeed, dateTime);
    lastTime = millis();
  }

  // this is our 'wait for incoming subscription packets' busy subloop
  // Check for change in the four variables from the MQTT broker
  MQTT_subscribe(&temperature, &humidity, &VOC, &CO2, &resetMQTT);

  //  Check for invalid data as indicated by a value of -1
  if (temperature == INVALID || 
      humidity == INVALID || 
      VOC == INVALID || 
      CO2 == INVALID) {
    dataValid = FALSE;
    watchDogTimer.stop();
  }  else if (!dataValid) {  // Data just turned valid
    dataValid = TRUE;
    Serial.printf("Data just turned valid\n\n");
    // digitalWrite(WDT_PBRSTN_PIN, LOW);  // Force watchdog to stop.
    watchDogTimer.start();              // Start watchdog keep-alive signal
  }


  // This is the fan control function call
  fanSpeed = fanControl(dateTime, humidity, VOC, dataValid, &eveningRun);


} // loop

////////////////////////////////////////////////
////////////////////////////////////////////////
//   FUNCTIONS
////////////////////////////////////////////////
////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
//          MQTT_CONNECT
//////////////////////////////////////////////////////////////
// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect(bool* valid) {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      //  Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
      //  Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       *valid = false;
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

///////////////////////////////////////////////////////////////
//          MQTT_PING
//////////////////////////////////////////////////////////////
bool MQTT_ping(String dateTime) {
  static unsigned int last;
  const int TWO_MINUTE_PING = 120000;
  bool pingStatus = FALSE;

  if ((millis()-last)>TWO_MINUTE_PING) {
      Serial.printf("Pinging MQTT at %s\n", dateTime.c_str());
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting at %s\n", dateTime.c_str());
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

///////////////////////////////////////////////////////////////
//          MQTT_PUBLISH
//////////////////////////////////////////////////////////////
void MQTT_publish(int relaySample, int relayState, int switchState, int VOC, 
                  bool dataValid, bool eveningRun, int fanSpeed, String dateTime) {
    if(mqtt.Update()) {
      mqttObj1.publish(relaySample);
      mqttObj2.publish(relayState);
      mqttObj3.publish(switchState);
      mqttObj4.publish(VOC);
      mqttObj5.publish(dataValid);
      mqttObj6.publish(eveningRun);
      mqttObj7.publish(fanSpeed);
      mqttObj8.publish(dateTime);
  }
}


///////////////////////////////////////////////////////////////
//          MQTT_SUBSCRIBE
//////////////////////////////////////////////////////////////
void MQTT_subscribe(float* temp, int* hum, int* volit, int* carbon, bool* resetin){
  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    if (subscription == &theTemperatureObject) {
      *temp = atof((char *)theTemperatureObject.lastread);
    }
    if (subscription == &theHumidityObject) {
      *hum = atof((char *)theHumidityObject.lastread);
    }
    if (subscription == &theTVOCObject) {
      *volit = atof((char *)theTVOCObject.lastread);
    }
    if (subscription == &theCO2Object) {
      *carbon = atof((char *)theCO2Object.lastread);
    }
    if (subscription == &theResetObject) {
      *resetin = ((atoi((char *)theResetObject.lastread))==1);
    }
  }
}


///////////////////////////////////////////////////////////////
//          FAN_CONTROL
//////////////////////////////////////////////////////////////
  //  Fan control logic needs valid data to work with.  During startup, the
  //  environmental data can be invalid for several minutes while the MQTT
  //  broker connections are established.  All local environmental variables
  //  are initialized to -1 to show they are invalid.  Once the MQTT broker
  //  is online, the data will be valid and fan control may begin.
  ////////////////////////////////////////////////////////////////////
int fanControl(String dateTime, int hum, int volit, bool dataValid, bool* eveningRun) {

  String    timeOnly, hourOnly;   
  const int  FAN_OFF    = 0;   // 0.0 on manual fan dial
  const int  FAN_IDLE   = 65;  // 2.6 on manual fan dial
  const int  FAN_NORMAL = 90;  // 3.5 on manual fan dial
  const int  FAN_HIGH   = 200; // 8.0 on manual fan dial

  const int VOC_HI_ON_LIMIT    = 350; // VOC limit above which the fan will turn on - from Clint Wolf's Code
  const int VOC_LOW_OFF_LIMIT  = 320; // VOC limit below which the fan will turn off - from Clint Wolf's Code
  const int HUMIDITY_LOW_LIMIT = 20;  // Humidity limit to determine fan speed - from Clint Wolf's Code
  const int PWM_FREQ_50Kh = 50000;
  const String EVENING_RUN_HOUR = "20";
  static int fanSpeed=FAN_OFF;
  static int fanSpeedOld=FAN_HIGH;
  timeOnly = dateTime.substring(11,16); //  Extract value of time from dateTime
  hourOnly = dateTime.substring(11,13); //  Extract value of time from dateTime
    // Serial.printf("Hour is: %s \n", hourOnly.c_str());


  //////////////////////////////////////////////////////////////////////////
  //  Fan control has two distinct phases: Evening phase where the fan is
  //  run for an hour at night to remove all latent VOCs.  The fan is then turned
  //  off to the remainder of the evening to allow the humidity to recover.
  //  Normal operation phase checks for VOC peaks and runs the fan to remove 
  //  VOCs with the fan speed dependant on the relative humidity.
  ///////////////////////////////////////////////////////////////////////////

  // Evening fan run to remove latent VOCs
  // Supercedes all other requirements
  if (hourOnly == EVENING_RUN_HOUR && !(*eveningRun)) {
    analogWrite(FAN_CTL_OUT_PIN, FAN_HIGH, PWM_FREQ_50Kh);  
    Serial.printf("Evening run started at %s \n", timeOnly.c_str());
    fanSpeed = FAN_HIGH;
    *eveningRun = TRUE;
  } else if (hourOnly != EVENING_RUN_HOUR && *eveningRun) {
    analogWrite(FAN_CTL_OUT_PIN, FAN_OFF);  
    Serial.printf("Evening run ended at %s \n", timeOnly.c_str());
    *eveningRun = FALSE;
    fanSpeed = FAN_OFF;
 }

  // Fan is not running and VOC limit exceeded
  // - Inactive during evening run of fan
  if (volit > VOC_HI_ON_LIMIT && dataValid && !(*eveningRun)) {
    if (hum > HUMIDITY_LOW_LIMIT) {
      analogWrite(FAN_CTL_OUT_PIN, FAN_NORMAL, PWM_FREQ_50Kh);  // Humidity is high so use higher fan speed
      fanSpeed = FAN_NORMAL;
    } else {
      analogWrite(FAN_CTL_OUT_PIN, FAN_IDLE, PWM_FREQ_50Kh);  // Humidity is low so use lower fan speed
      fanSpeed = FAN_IDLE;
    }
  } else if (volit <= VOC_LOW_OFF_LIMIT && !(*eveningRun)) {
    analogWrite(FAN_CTL_OUT_PIN, FAN_OFF);  // VOCs have dropped below threshold, turn fan off
      fanSpeed = FAN_OFF;
  }

    if(fanSpeedOld != fanSpeed) {
      fanSpeedOld = fanSpeed; 
      switch (fanSpeed){
        case FAN_HIGH:
          Serial.printf("1. Fan High started at %s\nVOC: %i\nHumidity: %i\n\n", timeOnly.c_str(), volit, hum); 
          break;
        case FAN_NORMAL:
          Serial.printf("2. Fan Normal started at %s\nVOC: %i\nHumidity: %i\n\n", timeOnly.c_str(), volit, hum); 
          break;
        case FAN_IDLE:
          Serial.printf("3. Fan High started at %s\nVOC: %i\nHumidity: %i\n\n", timeOnly.c_str(), volit, hum); 
          break;
        case FAN_OFF:
          Serial.printf("4. Fan Off at %s \nVOC: %i\nHumidity: %i\n\n", timeOnly.c_str(), volit, hum); 
          break;
        }
    }
  return fanSpeed;

}

/////////////////////////////////////////////
//  D7 Blink
//////////////////////////////////////////////
void blinkD7(int blinkNum) {
  for (int i = 0; i<blinkNum; i++){
    digitalWrite(D7_LED_PIN, HIGH);
    delay(100);
    digitalWrite(D7_LED_PIN, LOW);
    delay(100);
  }
  delay(200);
}

/////////////////////////////////////////////
//  Watchdog ISR function
//////////////////////////////////////////////
void watchDogISR() {
  WDT_ST_State = !WDT_ST_State;
  digitalWrite(WDT_ST_PIN, WDT_ST_State);
  digitalWrite(TPL5010_DONE_PIN, WDT_ST_State);
  return;
}
