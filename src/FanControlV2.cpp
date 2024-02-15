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

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
Adafruit_MQTT_Publish   mqttObj1 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/nmmep.dialValue");
Adafruit_MQTT_Publish   mqttObj2 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/nmmep.opampValue");
Adafruit_MQTT_Publish   mqttObj3 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/nmmep.relayValue");
Adafruit_MQTT_Publish   mqttObj4 = Adafruit_MQTT_Publish  (&mqtt, AIO_USERNAME "/feeds/nmmep.switchValue");


Adafruit_MQTT_Subscribe theTemperatureObject = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-temperature");
Adafruit_MQTT_Subscribe theHumidityObject    = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-humidity");
Adafruit_MQTT_Subscribe theTVOCObject        = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-tvoc");
Adafruit_MQTT_Subscribe theCO2Object         = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-eco2");
Adafruit_MQTT_Subscribe theResetObject       = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nmmep.signcompany-reset");


/************Declare Variables*************/

const int relayPin = A0;  // Input of the relay sampler
const int relayStatePin = A1;  // State of the relay; Manual, Auto, Floating
const int switchStatePin = A2;  // State of the relay; Manual, Auto, Floating

const int WDT_ST_PIN     = D0;  //  Watchdog timer strobe output on Pin A1
const int WDT_RSTN_PIN   = D1;  // Reset(low) from the DS1232. Used for instrumentation
const int tpl5010Wake    = D2; // Wake pin from tpl5010
const int tpl5010Done    = D3; // Done pin to tpl5010
const int fanCtlOutPin   = D4;  //  Fan control output pin to OpAmp
const int WDT_PBRSTN_PIN = D9;  // Reset(low) to the  DS1232. Used to control relay through BJT

const int WDT_Half_Time  = 250; //  Half time pulse for WDT.  Used to change the state ot the WDT Strobe output
bool      WDT_ST_State;         //  State of the watchdog timer strobe output


bool firstRead;  //  indicator of the first time the subscription has been read since the device turned on

float temperature = -1; // temperature variable set to Invalid Data
int   humidity    = -1; // humidity variable set to Invalid Data
int   VOC         = -1; // VOC variable set to Invalid Data
int   CO2         = -1; // CO2 variable set to Invalid Data

const int VOC_HiLimit      = 350;  // VOC limit above which the fan will turn on - from Clint Wolf's Code
const int humidityLowLimit = 20;  //  Humidity limit to determine fan speed - from Clint Wolf's Code

bool dataValid;   //  Flag for MQTT subscription data receiving on all four subscriptions
bool eveningRun;  //  Flag for nightly high speed fan

const int  fanOff    = 0;   // 0.0 on manual fan dial
const int  fanIdle   = 65;  // 2.6 on manual fan dial
const int  fanNormal = 90;  // 3.5 on manual fan dial
const int  fanHigh   = 200; // 8.0 on manual fan dial

//  Time & timing Variables
String    dateTime, timeOnly, timeOnlyOld, hourOnly;   
u_int64_t last, lastTime;  // These timing variables work with System.millis()

//  Sample Variables
  int dialSample;   //  Value of the manual dial output to the relay
  int opampSample;  //  Value of the opamp output to the relay
  int relaySample;  //  Value of the output of the relay

  int switchSample;  //  Current selected position of the manual/automatic switch;




//  INSTANTIATIONS  //
void watchDogISR();
void MQTT_connect();
bool MQTT_ping();


Timer watchDogTimer(WDT_Half_Time, watchDogISR);  //  Instantiate a Timer interrupt with period WDT_Half_Time and interrupt service routinte watchDogISR



/////////////////////////////////////////////////////////
//        SETUP
/////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

  pinMode(WDT_ST_PIN,    OUTPUT);
  pinMode(WDT_PBRSTN_PIN, OUTPUT);
  pinMode(fanCtlOutPin,  OUTPUT);

  pinMode(tpl5010Done, OUTPUT);
  pinMode(tpl5010Wake, INPUT);


  digitalWrite(WDT_PBRSTN_PIN, HIGH);  //  Force Watchdog into reset
  // Serial.printf("reset High\n");

  WDT_ST_State = 0;
  watchDogTimer.stop();

 
  // Connect to WiFi without going to Particle Cloud
  WiFi.connect();
  while(WiFi.connecting()) {
    // Serial.printf(".");
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
  Time.zone(-6);                                         //  Set time zone to MDT -6 from UTC
  Particle.syncTime();
  dateTime    = Time.timeStr();                          //  get current value of date and time
  timeOnlyOld = dateTime.substring(11,16);               //  Extract value of time from dateTime

  watchDogTimer.start();

  Particle.connect();

} // setup


// loop() runs over and over again, as quickly as it can execute.
void loop() {

  /*  Get current time*/
  dateTime = Time.timeStr();                          //  get current value of date and time
  timeOnly = dateTime.substring(11,16);               //  Extract value of time from dateTime
  hourOnly = dateTime.substring(11,13);               //  Extract value of time from dateTime

  /*  MQTT Function Block*/
// Validate connected to MQTT Broker
  MQTT_connect();
  MQTT_ping();

  // publish to cloud every 30 seconds
  if((System.millis()-lastTime > 30000)) {
    MQTT_publish(float *value)
    lastTime = millis();
  }

  // this is our 'wait for incoming subscription packets' busy subloop
  // Check for change in the four variables from the MQTT broker
  MQTT_subscribe(*value);



  
  
  //  Automatic Fan Control Output

  ///////////////////////////////////////////////////////////////////////////
  //  Fan control logic needs valid data to work with.  During startup, the
  //  environmental data can be invalid for several minutes while the MQTT
  //  broker connections are established.  All local environmental variables
  //  are initialized to -1 to show they are invalid.  Once the MQTT broker
  //  is online, the data will be valid and fan control may begin.
  ////////////////////////////////////////////////////////////////////

  //  Check for invalid data as indicated by a value of -1
  if (temperature == -1 || humidity == -1 || VOC == -1 || CO2 == -1) {
    dataValid = FALSE;
    watchDogTimer.stop();
  }  else if (!dataValid) {  // Data just turned valid
    dataValid = TRUE;
    digitalWrite(WDT_PBRSTN_PIN, LOW);  // Force watchdog to run.
    watchDogTimer.start();              // Start watchdog keep-alive signal
    // Serial.printf("Data Valid \n");

  }

  //////////////////////////////////////////////////////////////////////////
  //  Fan control has two distinct phases: Evening phase where the fan is
  //  run for an hour at night to remove all latent VOCs.  The fan is then turned
  //  off to the remainder of the evening to allow the humidity to recover.
  //  Normal operation phase checks for VOC peaks and runs the fan to remove 
  //  VOCs with the fan speed dependant on the relative humidity.
  ///////////////////////////////////////////////////////////////////////////

  // Evening fan run to remove latent VOCs
  // Supercedes all other requirements
  if (hourOnly == "20" && !eveningRun) {
    analogWrite(fanCtlOutPin, fanHigh);  
    Serial.printf("Evening run started at %s \n", timeOnly.c_str());
    eveningRun = TRUE;
  } else if (hourOnly != "20" && eveningRun) {
      analogWrite(fanCtlOutPin, fanOff);  
    Serial.printf("Evening run ended at %s \n", timeOnly.c_str());
      eveningRun = FALSE;
  }

      // Fan is not running and VOC limit exceeded
      // - Inactive during evening run of fan
      if (VOC > VOC_HiLimit && dataValid && !eveningRun) {
        if (humidity > humidityLowLimit) {
          analogWrite(fanCtlOutPin, fanNormal);  // Humidity is high so use higher fan speed
          Serial.printf("Fan Normal, VOC=%i, Humidity=%i \n", VOC, humidity);
        }  else {
          analogWrite(fanCtlOutPin, fanIdle);  // Humidity is low so use lower fan speed
          Serial.printf("Fan Idle, VOC=%i, Humidity=%i \n", VOC, humidity);
        }
      } else if (VOC <= VOC_HiLimit && !eveningRun) {
        analogWrite(fanCtlOutPin, fanOff);  // VOCs have dropped below threshold, turn fan off
        Serial.printf("Fan off \n");
      }
   

////////////////////////////////////////////////////////////////////////////
//  Fan output measurement samples three locations to determine which device,
//  the manual dial or the Argon,  is controlling the fan.  The output of the dial,
//  the output of the opamp, and the output of the relay are all sampled.
//  The input that most closely matches the output is determined to be the driver.
/////////////////////////////////////////////////////////////////////////////

  dialSample  = analogRead(dialPin);
  opampSample = analogRead(opampPin);
  relaySample = analogRead(relayPin);

/////////////////////////////////////////////////////////////////////////////
//  The user has the option of running the fan in either manual mode or
//  automatic mode.  The choice is determined by the position of a switch.  The 
//  position of the switch can be read by the Argon
//////////////////////////////////////////////////////////////////////////////

  switchSample = digitalRead(switchPin);


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
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  // Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      //  Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
      //  Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  // Serial.printf("MQTT Connected!\n");
}

///////////////////////////////////////////////////////////////
//          MQTT_PING
//////////////////////////////////////////////////////////////
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus = FALSE;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

///////////////////////////////////////////////////////////////
//          MQTT_PUBLISH
//////////////////////////////////////////////////////////////
void MQTT_publish(float *value) {
    if(mqtt.Update()) {
      mqttObj1.publish(dialSample);
      mqttObj2.publish(opampSample);
      mqttObj3.publish(relaySample);
      mqttObj4.publish(switchSample);
      Serial.printf("Publishing %0.2f \n",*value); 
  }
}


///////////////////////////////////////////////////////////////
//          MQTT_SUBSCRIBE
//////////////////////////////////////////////////////////////
void MQTT_subscribe(float *value){
  float tempVar;
  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    // Serial.printf("Inside subscribe\n");
    if (subscription == &theTemperatureObject) {
      temperature = atof((char *)theTemperatureObject.lastread);
      // Serial.printf("Received %0.1f from Adafruit.io feed theTemperatureObject \n",temperature);
    }

    if (subscription == &theHumidityObject) {
      humidity = atof((char *)theHumidityObject.lastread);
      // Serial.printf("Received %i from Adafruit.io feed theHumidityObject \n",humidity);
    }

    if (subscription == &theTVOCObject) {
      VOC = atof((char *)theTVOCObject.lastread);
      // Serial.printf("Received %i from Adafruit.io feed theTVOCObject \n",VOC);
    }

    if (subscription == &theCO2Object) {
      CO2 = atof((char *)theCO2Object.lastread);
      // Serial.printf("Received %i from Adafruit.io feed theeCO2Object \n",CO2);
    }

    if (subscription == &theResetObject) {
      CO2 = atof((char *)theResetObject.lastread);
      // Serial.printf("Received %i from Adafruit.io feed theResetObject \n",CO2);
    }
  }

}

/////////////////////////////////////////////
//  Watchdog ISR function
//////////////////////////////////////////////
void watchDogISR() {
  WDT_ST_State = !WDT_ST_State;
  digitalWrite(WDT_ST_PIN, WDT_ST_State);
  return;
}
