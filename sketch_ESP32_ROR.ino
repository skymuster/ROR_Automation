/**************************************************************************************************************************************************
* File name     : sketch_ESP32_ROR_NEWBY.c
* Version:      : 1.0
* Author        : Mike Newby   
* Created       : 27-May-2025
* Last modified :
*
* Description   :
    This sketch provides 3 separate control mechanisms for controlling the ROR Observatory:
    1. Manual Control:
        Open, Close, Stop buttons situated inside observatory
    2. HTTP Control:
         Controlled via HTTP calls (GET/POST) over Wi-Fi connection between ESP32 and miniPC. This allows integration into imaging sequence
         in N.I.N.A. USING advanced sequencer calling PowerShell scripts to automatically open/close roof at beginning and end of imaging 
         sessions, or when weather requires.
         Example Powershell script for Roof Open command:
                        $response = Invoke-WebRequest -Uri http:{hostname}}/status -UseBasicParsing
                        $data = $response.Content | ConvertFrom-Json

                        if ($data.telescope_safe -eq $true) {
                            Invoke-WebRequest -Uri http://{hostname}}/roof/open -Method POST -UseBasicParsing
                            Write-Output "Roof open command sent."
                            exit 0
                        } else {
                            Write-Output "Telescope not safe. Aborting."
                            exit 1
                        }

    3. Home Assistant / MQTT:
        Controlled using MQTT via WiFi connection between ESP32 and MQTT broker running on Home Assistant server. This allows Home Assistant to 
        monitor and control roof operation via smart switches, phone apps and touchscreens.

    Note: All 3 mechanisms only allow roof movement when telescope is “safe” (i.e. RA and DEC axes are both parked in the safe position, as 
    measured by the external reed switches on the mount. A local buzzer will also sound if roof movement is not allowed.  LEDs provide visual
    indicators of roof and telescope status.
*
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

#include <WiFi.h>
#include <PubSubClient.h>
//#include "BluetoothSerial.h"
#include <WebServer.h>

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Configuration --------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// WiFi credentials
const char*           ssid = "{SSID}";
const char*           password = "{WIFI_PASSWORD}";
const char*           hostname = "esp32_ror_controller";

// MQTT credentials
const char*           mqtt_server = "{SERVER_ADDRESS}";
const int             mqtt_port = 1883;
const char*           mqtt_user = "ROR_Controller";
const char*           mqtt_pass = "telescope";

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Global Variables -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// MQTT and WiFi clients
WiFiClient            espClient;
PubSubClient          mqtt_client(espClient);

// Bluetooth variables
//BluetoothSerial       SerialBT;
//String                serialin;
//String                str;

// Webserver
WebServer             http_server(80);

// Timers and flags
unsigned long         end_time;
int                   ROOF_LOST_DURATION = 30000;
bool                  lost = false;
bool                  telescope_safe = false;
bool                  ra_safe = false;
bool                  dec_safe = false;
int                   mqttCounterConn = 0;
bool                  roof_open = false;
bool                  roof_closed = false;
unsigned long         lastLostFlashTime = 0;
unsigned long         lastRoofFlashTime = 0;
bool                  roofLedState = false;
bool                  lostLedState = false;

// Debounce variables
const unsigned long   debounceDelay = 50; // milliseconds
bool                  lastOpenState = HIGH;
bool                  lastCloseState = HIGH;
bool                  lastStopState = HIGH;
bool                  lastOSCState = HIGH;
unsigned long         lastDebounceOpenTime = 0;
unsigned long         lastDebounceCloseTime = 0;
unsigned long         lastDebounceStopTime = 0;
unsigned long         lastDebounceOSCTime = 0;


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

#define USE_SINGLE_OSC_BUTTON false  // This flag sets whether the motor controller has a single button  to control Open/Stop/Close (OSC) or uses 3 separate buttons

#define RA_SWITCH           32
#define DEC_SWITCH          33
#define RA_LED_RED          16
#define RA_LED_GREEN        17
#define DEC_LED_RED         18
#define DEC_LED_GREEN       19
#define ROOF_OPENED_SENSOR   4
#define ROOF_CLOSED_SENSOR   5
#define ROOF_LED_OPEN       21
#define ROOF_LED_CLOSED     22
#define ROOF_LED_MOVING     23
#define PUSHBUTTON_OPEN     12
#define PUSHBUTTON_CLOSE    13
#define PUSHBUTTON_STOP     14
#define PUSHBUTTON_OSC      12
#define RELAY_OPEN          25
#define RELAY_CLOSE         26
#define RELAY_STOP          27
#define BUZZER_PIN           2
#define RELAY_OSC           25

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT Auto-discovery Payloads-------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// MQTT discovery payload for binary_sensor: “Roof Open” 
const char* DISCOVERY_ROOF_OPEN =
  R"({
    "name": "Roof Open",
    "state_topic": "home/observatory/roof/state/opened",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "opening",
    "unique_id": "esp32_ror_controller_roof_open_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “Roof Closed”
  const char* DISCOVERY_ROOF_CLOSED =
  R"({
    "name": "Roof Closed",
    "state_topic": "home/observatory/roof/state/closed",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "opening",
    "unique_id": "esp32_ror_roof_closed_id",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “Roof Moving”
const char* DISCOVERY_ROOF_MOVING =
  R"({
    "name": "Roof Moving",
    "state_topic": "home/observatory/roof/state/moving",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "moving",
    "unique_id": "esp32_ror_controller_roof_moving_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]
    }
  })";

// MQTT discovery payload for binary_sensor: “Roof Lost”
const char* DISCOVERY_ROOF_LOST =
  R"({
    "name": "Roof Lost",
    "state_topic": "home/observatory/roof/state/lost",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "problem",
    "unique_id": "esp32_ror_controller_roof_lost_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “RA Axis Safe”
const char* DISCOVERY_RA_SAFE =
  R"({
    "name": "RA Axis Safe",
    "state_topic": "home/observatory/roof/state/ra_safe",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "safety",
    "unique_id": "esp32_ror_controller_ra_safe_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “DEC Axis Safe”
const char* DISCOVERY_DEC_SAFE =
  R"({
    "name": "DEC Axis Safe",
    "state_topic": "home/observatory/roof/state/dec_safe",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "safety",
    "unique_id": "esp32_ror_controller_dec_safe_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “Telescope Safe”
const char* DISCOVERY_TELESCOPE_SAFE =
  R"({
    "name": "Telescope Safe",
    "state_topic": "home/observatory/roof/state/telescope_safe",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "safety",
    "unique_id": "esp32_ror_controller_telescope_safe_sensor",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "open"
const char* DISCOVERY_BUTTON_OPEN =
  R"({
    "name": "Roof Open Command",
    "command_topic": "home/observatory/roof/cmd/open",
    "payload_press": "ON",
    "unique_id": "ror_roof_open_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";


// MQTT discovery payload for MQTT switch: "close"
const char* DISCOVERY_BUTTON_CLOSE =
  R"({
    "name": "Roof Close Command",
    "command_topic": "home/observatory/roof/cmd/close",
    "payload_press": "ON",
    "unique_id": "ror_roof_close_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "stop"
const char* DISCOVERY_BUTTON_STOP =
  R"({
    "name": "Roof Stop Command",
    "command_topic": "home/observatory/roof/cmd/stop",
    "payload_press": "ON",
    "unique_id": "ror_roof_stop_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "osc_toggle"
const char* DISCOVERY_BUTTON_OSC =
  R"({
    "name": "Roof OSC Toggle",
    "command_topic": "home/observatory/roof/cmd/osc_toggle",
    "payload_press": "ON",
    "unique_id": "ror_roof_osc_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {

  int counter = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise onboard serial connection (for debugging)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(9600);

  // Set roof lost timer. Change to suit your rquirments to determine if roof is lost
  end_time=millis() + ROOF_LOST_DURATION;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise Bluetooth Connection: Pair device "ESP32-Telescope" from the mini-PC and Windows will assign it a COM port (e.g. COM4).
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //SerialBT.begin("ESP32-Telescope");
  //Serial.println("Setup: Bluetooth initialised");

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise WiFi Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED && counter++ < 8) 
  {
      delay(1000);
      Serial.print(".");
  }
  Serial.println("");

  WiFi.setHostname(hostname);
  
  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(WiFi.getHostname());
    
  } else
  {
    Serial.println("WiFi NOT connected!!!");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise Webserver
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  http_server.on("/roof/open", HTTP_POST, OpenRoof);
  http_server.on("/roof/close", HTTP_POST, CloseRoof);
  http_server.on("/roof/stop", HTTP_POST, StopRoof);
  http_server.on("/roof/toggle", HTTP_POST, ToggleRoof);
  http_server.on("/status", HTTP_GET, getStatus);
  http_server.on("/safety_status", HTTP_GET, getSafetyStatus);
  http_server.begin();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise MQTT Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(callback);
  mqtt_client.setBufferSize(600);    // Important to increase MQTT payload buffer as default is too small and will fail to send auto-discover message
  Serial.println("Connecting to MQTT…");
  if (mqtt_client.connect("ESP32-Roof", mqtt_user, mqtt_pass)) {
    Serial.println("MQTT connected in setup");
    mqtt_client.subscribe("home/observatory/roof/cmd/#");
    publishDiscovery();
  } else {
    Serial.print("MQTT failed in setup, rc=");
    Serial.println(mqtt_client.state());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise GPIO Pin Definitions
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set  modes for GPIO pins
  pinMode(RA_SWITCH, INPUT_PULLUP);
  pinMode(DEC_SWITCH, INPUT_PULLUP);
  pinMode(RA_LED_RED, OUTPUT);
	pinMode(RA_LED_GREEN, OUTPUT);
  pinMode(DEC_LED_RED, OUTPUT);
	pinMode(DEC_LED_GREEN, OUTPUT);
  pinMode(ROOF_OPENED_SENSOR, INPUT_PULLUP);
  pinMode(ROOF_CLOSED_SENSOR, INPUT_PULLUP);
  pinMode(ROOF_LED_OPEN, OUTPUT);
	pinMode(ROOF_LED_CLOSED, OUTPUT);
	pinMode(ROOF_LED_MOVING, OUTPUT);
  pinMode(PUSHBUTTON_OPEN, INPUT_PULLUP);
  pinMode(PUSHBUTTON_CLOSE, INPUT_PULLUP);
  pinMode(PUSHBUTTON_STOP, INPUT_PULLUP);
  pinMode(PUSHBUTTON_OSC, INPUT_PULLUP);
  pinMode(RELAY_OPEN, OUTPUT);
  pinMode(RELAY_CLOSE, OUTPUT); 
  pinMode(RELAY_STOP, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
	
	// Set initial states of relays and buzzer
	digitalWrite(RELAY_OPEN, LOW);
	digitalWrite(RELAY_CLOSE, LOW);
	digitalWrite(RELAY_STOP, LOW);
  digitalWrite(RELAY_OSC, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  //SerialBT.print("RRCI#"); //init string
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

void loop() {

  // Call the LostTimer() function to detemine whether the roof is "lost"
	LostTimer();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MQTT Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if(WiFi.status() == WL_CONNECTED)
  {
      if(!mqtt_client.connected())
          MqttReconnect();
      else
          mqtt_client.loop();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Webserver Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if(WiFi.status() == WL_CONNECTED) http_server.handleClient();  // Listen for HTTP requests
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Check sensor states and reflect the status in 1) LEDs, 2) MQTT
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Check the safety status of the RA and DEC sensors
  ra_safe = (digitalRead(RA_SWITCH) == LOW);
  dec_safe = (digitalRead(DEC_SWITCH) == LOW);
  telescope_safe = (digitalRead(RA_SWITCH) == LOW) && (digitalRead(DEC_SWITCH) == LOW);

  // Check the status of the roof sensors
  roof_open = (digitalRead(ROOF_OPENED_SENSOR) == LOW);
  roof_closed = (digitalRead(ROOF_CLOSED_SENSOR) == LOW);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Set LED states according to sensor states
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // RA-axis status: If RA-axis is safe (parked) then tuen on the GREEN RA status LED and turn off the RED RA status LED
  digitalWrite(RA_LED_GREEN, ra_safe ? HIGH : LOW);
  digitalWrite(RA_LED_RED,   ra_safe ? LOW : HIGH);

  // DECaxis status: If DEC-axis is safe (parked) then tuen on the GREEN DEC status LED and turn off the RED DEC status LED
  digitalWrite(DEC_LED_GREEN, dec_safe ? HIGH : LOW);
  digitalWrite(DEC_LED_RED,   dec_safe ? LOW : HIGH);

  unsigned long now = millis();

  // ROOF OPEN (STATIONARY)
  if (roof_open && !roof_closed) {
    digitalWrite(ROOF_LED_OPEN, HIGH);
    digitalWrite(ROOF_LED_CLOSED, LOW);
    digitalWrite(ROOF_LED_MOVING, LOW);
    roofLedState = false;
    lostLedState = false;
  }
  // ROOF CLOSED (STATIONARY)
  else if (!roof_open && roof_closed) {
    digitalWrite(ROOF_LED_OPEN, LOW);
    digitalWrite(ROOF_LED_CLOSED, HIGH);
    digitalWrite(ROOF_LED_MOVING, LOW);
    roofLedState = false;
    lostLedState = false;
  }
  // ROOF MOVING (not lost)
  else if (!roof_open && !roof_closed && !lost) {
    digitalWrite(ROOF_LED_OPEN, LOW);
    digitalWrite(ROOF_LED_CLOSED, LOW);
    if (now - lastRoofFlashTime >= 500) {
      lastRoofFlashTime = now;
      roofLedState = !roofLedState;
      digitalWrite(ROOF_LED_MOVING, roofLedState ? HIGH : LOW);
    }
    lostLedState = false;
  }
  // ROOF LOST
  else if (!roof_open && !roof_closed && lost) {
    if (now - lastLostFlashTime >= 500) {
      lastLostFlashTime = now;
      lostLedState = !lostLedState;
      digitalWrite(ROOF_LED_OPEN, lostLedState ? HIGH : LOW);
      digitalWrite(ROOF_LED_CLOSED, lostLedState ? HIGH : LOW);
      digitalWrite(ROOF_LED_MOVING, lostLedState ? HIGH : LOW);
    }
  }
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Publish sensor states to MQTT
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  mqtt_client.publish("home/observatory/roof/state/opened", roof_open ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/closed", roof_closed ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/ra_safe", !ra_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/dec_safe", !dec_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/telescope_safe", !telescope_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/moving", (!roof_open && !roof_closed && !lost) ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/lost", lost ? "true" : "false", true);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RESPOND TO MANUAL PUSHBUTTONS (includes debounce logic)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// READ CURRENT STATES
  bool readingOSC = digitalRead(PUSHBUTTON_OSC);
  bool readingOpen = digitalRead(PUSHBUTTON_OPEN);
	bool readingClose = digitalRead(PUSHBUTTON_CLOSE);
	bool readingStop = digitalRead(PUSHBUTTON_STOP);

	unsigned long currentTime = millis();

  if (USE_SINGLE_OSC_BUTTON) {
    if (readingOSC != lastOSCState) lastDebounceOSCTime = currentTime;
    if (((currentTime - lastDebounceOSCTime) > debounceDelay) && (readingOSC == LOW)) ToggleRoof();
    lastOSCState = readingOSC;
  } else {
    // OPEN BUTTON
    if (readingOpen != lastOpenState) lastDebounceOpenTime = currentTime;
    if (((currentTime - lastDebounceOpenTime) > debounceDelay) && (readingOpen == LOW)) OpenRoof();
    lastOpenState = readingOpen;

    // CLOSE BUTTON
    if (readingClose != lastCloseState) lastDebounceCloseTime = currentTime;
    if (((currentTime - lastDebounceCloseTime) > debounceDelay) && (readingClose == LOW)) CloseRoof();
    lastCloseState = readingClose;

    // STOP BUTTON
    if (readingStop != lastStopState) lastDebounceStopTime = currentTime;
    if (((currentTime - lastDebounceStopTime) > debounceDelay) && (readingStop == LOW)) StopRoof();
    lastStopState = readingStop;
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RESPOND TO SERIAL comms (over Bluetooth conenction)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
  
  //Verify connection by serial
  while (SerialBT.available() > 0) {

    //Read serial data from ASCOM and allocate to serialin variable
    serialin = SerialBT.readStringUntil('#');

    Serial.print("Debug checkpoint. Serial string: ");
    Serial.println(serialin);

		// COMMAND = "x" (Emergency stop whilst opening)
		if (serialin == "x") StopRoof();

    // COMMAND = "y" (Emergency stop whilst closing)
    else if (serialin == "y") StopRoof();			
	
		// COMMAND = "open", ACTION = Open roof if safe to do so
    else if (serialin == "open") OpenRoof();
    
    // COMMAND = "close", ACTION = Close roof if safe to do so
    else if (serialin == "close") CloseRoof();

		// COMMAND = "Parkstatus", ACTION = return the park status of the telescope
    else if (serialin == "Parkstatus") {
      SerialBT.println("0#");
      serialin = "";
    }

    //COMMAND: "Status", ACTION = Return the string "RoofOpen"
		else if (serialin == "Status") {
			SerialBT.println("RoofOpen#");
			serialin = "";
		}
		
    // COMMAND = "get", ACTION = return the status of the RRCI data in the format: "ROOF_STATUS, TELESCOPE_SAFETY_STATUS, MOBILITY_STATUS#"
    else if (serialin == "get"){ 
  
      // Report the roof status (opened, closed or unknown)
      if (digitalRead(ROOF_OPENED_SENSOR) == LOW) str += "opened,"; 
      else if (digitalRead(ROOF_CLOSED_SENSOR) == LOW) str += "closed,";
			else if ((digitalRead(ROOF_CLOSED_SENSOR) == HIGH) && (digitalRead(ROOF_OPENED_SENSOR) == HIGH)) str += "unknown,";

      // Report the safety status of the telescope (safe or unsafe)
      if (telescope_safe) str += "safe,";
      else str += "unsafe,";
    
      // Report mobility status of roof (not moving (open), not moving (closed), moving or unknown)
      if (digitalRead(ROOF_CLOSED_SENSOR) == HIGH && digitalRead(ROOF_OPENED_SENSOR) == LOW && !lost) {
        str += "not_moving_o#";
        end_time = millis() + 60000;
      } else if (digitalRead(ROOF_CLOSED_SENSOR) == LOW && digitalRead(ROOF_OPENED_SENSOR) == HIGH && !lost) {
        str += "not_moving_c#";
        end_time = millis() + 60000;
      } else if (digitalRead(ROOF_CLOSED_SENSOR) == HIGH && digitalRead(ROOF_OPENED_SENSOR) == HIGH && !lost) {
        str += "moving#";
      } else if (digitalRead(ROOF_CLOSED_SENSOR) == HIGH && digitalRead(ROOF_OPENED_SENSOR) == HIGH && lost) {
        str += "unknown#";
      }
      if (str.endsWith(",")) str += "unknown#";

      // Write output back to ASCOM via serial interface
			SerialBT.println(str);
	  
      str = "";
    }
		
		// Re-initialise the variables
		serialin  = "";
		
	}	  
*/

	//delay(500); // Publish every 0.5 seconds

}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// Function to OPEN the roof, including checking that the telescope is safe
void OpenRoof () {
    if (telescope_safe) {
      if (!roof_open && !roof_closed && !lost) {    // First stop the roof if it is currently moving
        StopRoof();
        delay(1000);
      }
			digitalWrite(RELAY_OPEN, HIGH);
      delay(500);
      digitalWrite(RELAY_OPEN, LOW);
			//SerialBT.println("Opening roof...");
      Serial.println("Opening roof...");
      http_server.send(200, "text/plain", "Opening roof...");
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
    }	else {
      //SerialBT.println("Cannot open roof - telescope is UNSAFE");
      Serial.println("Cannot open roof - telescope is UNSAFE");
      http_server.send(200, "text/plain", "Cannot open roof - telescope is UNSAFE");
      tone(BUZZER_PIN, 1000);
      delay(1000);
      noTone(BUZZER_PIN);
    }
}

// Function to CLOSE the roof, including checking that the telescope is safe
void CloseRoof () {
    if (telescope_safe) {
      if (!roof_open && !roof_closed && !lost) {    // First stop the roof if it is currently moving
        StopRoof();
        delay(1000);
      }
			digitalWrite(RELAY_CLOSE, HIGH);
      delay(500);
      digitalWrite(RELAY_CLOSE, LOW);
			//SerialBT.println("Closing roof...");
      Serial.println("Closing roof...");
      http_server.send(200, "text/plain", "Closing roof...");
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
    } else {
      //SerialBT.println("Cannot close roof - telescope is UNSAFE");
      Serial.println("Cannot close roof - telescope is UNSAFE");
      http_server.send(200, "text/plain", "Cannot close roof - telescope is UNSAFE");
      tone(BUZZER_PIN, 1000);
      delay(1000);
      noTone(BUZZER_PIN);
    }
}

// Function to STOP the roof
void StopRoof () {
    digitalWrite(RELAY_OPEN, LOW);
    digitalWrite(RELAY_CLOSE, LOW);
    digitalWrite(RELAY_STOP, HIGH); 
    delay(500);
    digitalWrite(RELAY_STOP, LOW);
    //SerialBT.println("Stopping roof...");
    Serial.println("Stopping roof...");
    http_server.send(200, "text/plain", "Stopping roof...");
}

// Function to TOGGLE the roof (if using single OSC button), including checking that the telescope is safe
void ToggleRoof () {
    if (telescope_safe) {
			digitalWrite(RELAY_OSC, HIGH);
      delay(500);
      digitalWrite(RELAY_OSC, LOW);
			//SerialBT.println("Toggling roof...");
      Serial.println("Toggling roof...");
      http_server.send(200, "text/plain", "Toggling roof...");
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
    }	else {
      //SerialBT.println("Cannot toggle roof - telescope is UNSAFE");
      Serial.println("Cannot toggle roof - telescope is UNSAFE");
      http_server.send(200, "text/plain", "Cannot toggle roof - telescope is UNSAFE");
      tone(BUZZER_PIN, 1000);
      delay(1000);
      noTone(BUZZER_PIN);
    }
}

// Function to handle incoming MQTT commands
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Received callback...");

  String cmd = "";
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  
  Serial.print("Topic = ");  
  Serial.println(topic);
  Serial.print("CMD = ");  
  Serial.println(cmd);

  if (USE_SINGLE_OSC_BUTTON) {
    if (String(topic) == "home/observatory/roof/cmd/osc_toggle" && cmd == "ON") ToggleRoof();
  } else {
    if (String(topic) == "home/observatory/roof/cmd/open" && cmd == "ON") OpenRoof();
    if (String(topic) == "home/observatory/roof/cmd/close" && cmd == "ON") CloseRoof();
    if (String(topic) == "home/observatory/roof/cmd/stop" && cmd == "ON") StopRoof();
  }
}

// Function to publish discovery messages to MQTT broker running on home assistant
void publishDiscovery() {
  if(mqtt_client.connected())
    {    
      // sensors
      mqtt_client.publish("homeassistant/binary_sensor/roof_open/config",       DISCOVERY_ROOF_OPEN, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_closed_v2/config",  DISCOVERY_ROOF_CLOSED, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_moving/config",     DISCOVERY_ROOF_MOVING, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_lost/config",       DISCOVERY_ROOF_LOST, true);
      mqtt_client.publish("homeassistant/binary_sensor/ra_safe/config",         DISCOVERY_RA_SAFE, true);
      mqtt_client.publish("homeassistant/binary_sensor/dec_safe/config",        DISCOVERY_DEC_SAFE, true);
      mqtt_client.publish("homeassistant/binary_sensor/telescope_safe/config",  DISCOVERY_TELESCOPE_SAFE, true);

      // buttons
      if (USE_SINGLE_OSC_BUTTON) {
        mqtt_client.publish("homeassistant/button/roof_osc/config",             DISCOVERY_BUTTON_OSC, true);
      } else {
        mqtt_client.publish("homeassistant/button/roof_open/config",            DISCOVERY_BUTTON_OPEN, true);
        mqtt_client.publish("homeassistant/button/roof_close/config",           DISCOVERY_BUTTON_CLOSE, true);
        mqtt_client.publish("homeassistant/button/roof_stop/config",            DISCOVERY_BUTTON_STOP, true);
      }
    }
}


// Function to define whether roof is "lost", which occurs when roof_open and roof_closed sensors are both HIGH (open) for greater than ROOF_LOST_DURATION seconds
void LostTimer() { 
	if (millis() >= end_time && !roof_closed && !roof_open)
		lost = true; //where the heck is the roof position?
	if (roof_closed || roof_open) {
		lost = false; //roof state is known
		end_time = millis() + ROOF_LOST_DURATION; //reset the timer  
	}
}

void MqttReconnect() 
{
    // Loop until we're reconnected, try 4 times
    while (!mqtt_client.connected()  && (mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqtt_client.connect("ESP32-Roof"), mqtt_user, mqtt_pass) {
            Serial.println("MQTT connected");
            // Subscribe
            mqtt_client.subscribe("home/observatory/roof/cmd/#");
            delay(100);
        } else {
            Serial.print("MQTT connection failed, reason code=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 1 seconds");
            delay(1000);
        }
    }  
    mqttCounterConn = 0;
}

String getRoofState() {
  if (lost) {
    return "lost";
  } else if (roof_open) {
    return "open";
  } else if (roof_closed) {
    return "closed";
  } else {
    return "moving";
  }
}

void getStatus() {
  String roofState = getRoofState();
  String safeState = telescope_safe ? "true" : "false";
  String json = "{\"roof\":\"" + roofState + "\", \"telescope_safe\":" + safeState + "}";
  http_server.send(200, "application/json", json);
}

void getSafetyStatus() {
    http_server.send(200, "application/json", telescope_safe ? "true" : "false");
  }
