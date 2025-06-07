/**************************************************************************************************************************************************
* File name     : sketch_ESP32_ROR.ino
* Version:      : 1.3
* Author        : Mike Newby   
* Created       : 2-Jun-2025
* Last modified : 7-Jun-2025
*
* Description   :
    This sketch provides 4 separate control mechanisms for controlling the ROR Observatory:
    
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

    3. Serial Control:
        Controlled via a wired serial connection between ESP32 and miniPC using UART pins on the ESP32. On the miniPC a CP2102 USB to serial adapter
        module is used as this allows longer cable runs than possible with a standard USB cable. The ASCOM Rolling Roof Computer Interface (RRCI) by 
        Chuck Faranda is installed on the miniPC which sends and receives serial commands to the ESP32. This sketch is designed to handle the specific
        commands from that interface.

    4. Home Assistant / MQTT:
        Controlled using MQTT via WiFi connection between ESP32 and MQTT broker running on Home Assistant server. This allows Home Assistant to 
        monitor and control roof operation via smart switches, phone apps and touchscreens.

    Note: All 3 mechanisms only allow roof movement when telescope is “safe” (i.e. RA and DEC axes are both parked in the safe position, as 
    measured by the external reed switches on the mount. A local buzzer will also sound if roof movement is not allowed.  LEDs provide visual
    indicators of roof and telescope status.

    Changelog
    =========
    27-May-2025   v1.0      Initial version (Manual, HTTP and MQTT interfaces)
    2-Jun-2025    v1.1      Addded Serial UART interface
    4-Jun-2025    v1.2      Replaced LED pins with serial interface to an external shift register interface (model: 74HC595). This allows up to 8 LEDs 
                            to be controlled using just 5 pins (data, clock, latch, VCC, GND). This allows for a CAT6 cable to be employed between the 
                            user interaface panel and the control box.
    7-Jun-2025    v1.3      Changed pushbutton interface to use a parallel-to-serial serial shift regisster (74HC165).  This allows up to 8 switches 
                            to be controlled using just one extra data pin.  Added 3 extra pushbutton switches to control observatory lighting (max,
                            dim, off) - these will be setup to control LED smart lights that are integrated into home assistant and will be controlled
                            via MQTT.
*
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Configuration --------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// WiFi credentials
const char*           ssid = "{SSID}";
const char*           password = "{PASSWORD}";
const char*           hostname = "esp32_ror_controller";

// MQTT credentials
const char*           mqtt_server = "{MQTT Server name}";
const int             mqtt_port = {MQTT Server Port};
const char*           mqtt_user = "{MQTT user name}";
const char*           mqtt_pass = "{MQTT password}";

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Global Variables -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// MQTT and WiFi clients
WiFiClient            espClient;
PubSubClient          mqtt_client(espClient);

// UART Serial variables
String                serialin;
String                str;
HardwareSerial        UART_Serial(2);  // Use UART2

// Webserver
WebServer             http_server(80);

// Timers and flags
unsigned long         roofLostStartTime;
int                   ROOF_LOST_DURATION = 30000;
bool                  roof_lost = false;
bool                  telescope_safe = false;
bool                  ra_safe = false;
bool                  dec_safe = false;
bool                  scope_pwr = false;
int                   mqttCounterConn = 0;
bool                  roof_open = false;
bool                  roof_closed = false;
unsigned long         lastLostFlashTime = 0;
unsigned long         lastRoofFlashTime = 0;
bool                  roofLedState = false;
bool                  lostLedState = false;
unsigned long         bootTime = 0;
unsigned long         lastMqttAttempt = 0;
int                   mqttFailureCount = 0;
unsigned long         now = 0;
unsigned long         lastWiFiCheck = 0;
unsigned long         lastSerialInitCheck = 0;
enum                  RoofStatusVal{OPEN, CLOSED, MOVING, LOST};
String                roof_status_text;
RoofStatusVal         roof_status;
byte                  ledState = 0b00000000;  // Used to track up to 8 LEDs

// Debounce variables
const unsigned long   debounceDelay = 50; // milliseconds
bool                  lastStopState = HIGH;
bool                  lastOSCState = HIGH;
bool                  lastPWRReading = HIGH;         // last physical reading (HIGH = unpressed)
bool                  lastPWRStableState = HIGH;     // last debounced stable state
bool                  lastLightsMaxState = HIGH;
bool                  lastLightsDimState = HIGH;
bool                  lastLightsOffState = HIGH;
unsigned long         lastDebounceStopTime = 0;
unsigned long         lastDebounceOSCTime = 0;
unsigned long         lastDebouncePWRTime = 0;
unsigned long         lastDebounceLightsMaxTime = 0;
unsigned long         lastDebounceLightsDimTime = 0;
unsigned long         lastDebounceLightsOffTime = 0;


// Shift register LED bit mappings
#define               ROOF_LED_OPEN         0
#define               ROOF_LED_CLOSED       1
#define               ROOF_LED_MOVING       2
#define               TELESCOPE_SAFE_LED    3
#define               TELESCOPE_UNSAFE_LED  4
#define               TELESCOPE_POWER_LED   5


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// Sensors
#define RA_SWITCH             32
#define DEC_SWITCH            33
#define ROOF_OPENED_SENSOR    4
#define ROOF_CLOSED_SENSOR    5
// Relays
#define RELAY_STOP            18
#define RELAY_OSC             21
#define RELAY_PWR             19
// Buzzer
#define BUZZER_PIN            25
// UART
#define UART_SERIAL_TX        17
#define UART_SERIAL_RX        16
// Shift register (for LED control)
#define SR_DATA_595           12  // DATA byte to send to pin SER/DS on the 74HC595 shift register (to control LEDs) 
#define SR_DATA_165           14  // DATA byte to receive from pin Q7/QH on the 74HC165 shift register (to respond to buttons)
#define SR_CLOCK              27  // CLOCK / SRCLK
#define SR_LATCH              26  // LATCH / RCLK



/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT Auto-discovery Payloads-------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

// MQTT discovery payload for binary_sensor: “Roof Open” 
const char* DISCOVERY_ROOF_OPEN =
  R"({
    "name": "Roof Open Sensor",
    "state_topic": "home/observatory/roof/state/opened",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "occupancy",
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
    "name": "Roof Closed Sensor",
    "state_topic": "home/observatory/roof/state/closed",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "occupancy",
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

// MQTT discovery payload for binary_sensor: Roof Status”
const char* DISCOVERY_ROOF_STATUS = R"({
  "name": "Roof Status",
  "state_topic": "home/observatory/roof/state/roof_status",
  "unique_id": "esp32_ror_controller_roof_status_sensor",
  "icon": "mdi:garage-variant",
  "device": {
    "name": "ROR_Controller",
    "model": "ESP32-WROOM",
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
    "retain": false,
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
    "retain": false,
    "unique_id": "ror_roof_osc_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "power"
const char* DISCOVERY_BUTTON_PWR =
  R"({
    "name": "Telescope Power",
    "command_topic": "home/observatory/roof/cmd/power",
    "payload_press": "ON",
    "retain": false,
    "unique_id": "ror_roof_telescope_pwr_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for binary_sensor: “Telescope Power Status"
const char* DISCOVERY_TELESCOPE_POWER =
  R"({
    "name": "Telescope Power State",
    "state_topic": "home/observatory/roof/state/power",
    "payload_on": "true",
    "payload_off": "false",
    "device_class": "power",
    "unique_id": "esp32_ror_controller_telescope_power_state",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "lights_max"
const char* DISCOVERY_BUTTON_LIGHTS_MAX =
  R"({
    "name": "Observatory Lights Max",
    "command_topic": "home/observatory/roof/cmd/lights_max",
    "payload_press": "ON",
    "retain": false,
    "unique_id": "ror_roof_lights_max_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

// MQTT discovery payload for MQTT switch: "lights_dim"
const char* DISCOVERY_BUTTON_LIGHTS_DIM =
  R"({
    "name": "Observatory Lights Dim",
    "command_topic": "home/observatory/roof/cmd/lights_dim",
    "payload_press": "ON",
    "retain": false,
    "unique_id": "ror_roof_lights_dim_btn",
    "device": {
      "name": "ROR_Controller",
      "model": "ESPRESSIF ESP32-WROOM",
      "manufacturer": "Lonely Binary",
      "sw_version": "1.0",
      "identifiers": ["esp32_ror_controller"]      
    }
  })";

  // MQTT discovery payload for MQTT switch: "lights_off"
const char* DISCOVERY_BUTTON_LIGHTS_OFF =
  R"({
    "name": "Observatory Lights Off",
    "command_topic": "home/observatory/roof/cmd/lights_off",
    "payload_press": "ON",
    "retain": false,
    "unique_id": "ror_roof_lights_off_btn",
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
  bootTime = millis(); // mark boot time

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise onboard serial connection (for debugging)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(9600);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise UART serial connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  UART_Serial.begin(9600, SERIAL_8N1, UART_SERIAL_RX, UART_SERIAL_TX);
  UART_Serial.flush(); // clear output buffer
  UART_Serial.print("UART2 ready!  Serial available?  ");
  UART_Serial.println(UART_Serial.available());
  
  Serial.print("UART2 ready!  Serial available?  ");
  Serial.println(UART_Serial.available());

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise WiFi Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

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
  // Initialise mDNS so device can be reached via DNS name
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (MDNS.begin("esp32_ror_controller")) Serial.println("mDNS responder started");
  

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialise Webserver and configure for how to respond to GET/POST messages
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  http_server.on("/roof/stop", HTTP_POST, StopRoof);
  http_server.on("/roof/toggle", HTTP_POST, ToggleRoof);
  http_server.on("/telescope/power", HTTP_POST, ToggleTelescopePower);
  http_server.on("/lights/max", HTTP_POST, TurnLightsMax);
  http_server.on("/lights/dim", HTTP_POST, TurnLightsDim);
  http_server.on("/lights/off", HTTP_POST, TurnLightsOff);
 
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
    delay(500);  // Give HA time to register entity
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
  pinMode(ROOF_OPENED_SENSOR, INPUT_PULLUP);
  pinMode(ROOF_CLOSED_SENSOR, INPUT_PULLUP);
  pinMode(RELAY_OSC, OUTPUT);
  pinMode(RELAY_STOP, OUTPUT);
  pinMode(RELAY_PWR, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SR_DATA_595, OUTPUT);
  pinMode(SR_DATA_165, INPUT);
  pinMode(SR_CLOCK, OUTPUT);
  pinMode(SR_LATCH, OUTPUT);
	
	// Set initial states of relays and buzzer
	digitalWrite(RELAY_STOP, LOW);
  digitalWrite(RELAY_OSC, LOW);
  digitalWrite(RELAY_PWR, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  UART_Serial.print("RRCI#"); //init string
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

void loop() {

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MQTT Connection
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // WiFi reconnect handling (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    now = millis();
    if (now - lastWiFiCheck > 10000) { // Try reconnect every 10 seconds
      lastWiFiCheck = now;
      Serial.println("WiFi disconnected. Attempting reconnect...");
      WiFi.disconnect(true);
      WiFi.begin(ssid, password);
    }
  }

  // MQTT handling
  if (!mqtt_client.connected()) {
    now = millis();
    if (now - lastMqttAttempt > 5000) { // Try every 5 seconds
      lastMqttAttempt = now;
      Serial.println("Attempting MQTT reconnect...");
      MqttReconnect();

      if (!mqtt_client.connected()) {
        mqttFailureCount++;
        Serial.print("MQTT reconnect failed. Count = ");
        Serial.println(mqttFailureCount);
      } else {
        mqttFailureCount = 0; // Reset counter on success
      }

      // If too many failures, restart WiFi
      if (mqttFailureCount > 10) {
        Serial.println("Too many MQTT failures. Restarting WiFi.");
        WiFi.disconnect(true);
        WiFi.begin(ssid, password);
        mqttFailureCount = 0;
        delay(1000);
        return;
      }
    }
  } else {
    mqtt_client.loop();  // Service MQTT connection
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
  telescope_safe = ra_safe && dec_safe;

  // Check the status of the roof sensors
  roof_open = (digitalRead(ROOF_OPENED_SENSOR) == LOW);
  roof_closed = (digitalRead(ROOF_CLOSED_SENSOR) == LOW);

  // Check if roof is lost (roof_open & roof_closed are both equal for > ROOF_LOST_DURATION)
  if (roof_closed && roof_open)  roof_lost = true;                   // Ambiguous state: both sensors are true so set roof to LOST
  else if (!roof_closed && !roof_open) {                             // Ambiguous state: both sensors are false, so assume roof is moving unless it's been in this state for longer than ROOF_LOST_DURATION
    if (roofLostStartTime == 0) roofLostStartTime = millis();
    if (millis() - roofLostStartTime >= ROOF_LOST_DURATION) roof_lost = true;
  } else {
    roof_lost = false;                                               // Sensors now give a consistent result (only one is true)
    roofLostStartTime = 0;  // Reset the timer
  }

  // Set roof status value variable
  if (roof_lost) roof_status = LOST;
  else if (roof_open && !roof_closed) roof_status = OPEN;
  else if (!roof_open && roof_closed) roof_status = CLOSED;
  else roof_status = MOVING;

  // Check telescope power status
  scope_pwr = (digitalRead(RELAY_PWR) == HIGH);
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Set LED states according to sensor states
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // TELESCOPE SAFE LEDs
  if (telescope_safe) {
    setLED(TELESCOPE_SAFE_LED, true);
    setLED(TELESCOPE_UNSAFE_LED, false);
  } else {
    setLED(TELESCOPE_SAFE_LED, false);
    setLED(TELESCOPE_UNSAFE_LED, true);
  }
  
  // ROOF STATUS LEDs
  unsigned long now = millis();
  switch (roof_status) {
      case OPEN:
        roof_status_text = "Open";
        setLED(ROOF_LED_OPEN, true);
        setLED(ROOF_LED_CLOSED, false);
        setLED(ROOF_LED_MOVING, false);
        roofLedState = false;
        lostLedState = false;
        break;

      case CLOSED:
        roof_status_text = "Closed";
        setLED(ROOF_LED_OPEN, false);
        setLED(ROOF_LED_CLOSED, true);
        setLED(ROOF_LED_MOVING, false);
        roofLedState = false;
        lostLedState = false;
        break;

      case MOVING:
        roof_status_text = "Moving";
        setLED(ROOF_LED_OPEN, false);
        setLED(ROOF_LED_CLOSED, false);
        if (now - lastRoofFlashTime >= 500) {
          lastRoofFlashTime = now;
          roofLedState = !roofLedState;
          setLED(ROOF_LED_MOVING, roofLedState);
        }
        break;

      case LOST: 
        roof_status_text = "Lost";
        if (now - lastLostFlashTime >= 500) {
          lastLostFlashTime = now;
          lostLedState = !lostLedState;
          setLED(ROOF_LED_OPEN, lostLedState);
          setLED(ROOF_LED_CLOSED, lostLedState);
          setLED(ROOF_LED_MOVING, lostLedState);
          break;
        }
  }

  // Telescope Power ON LED
  setLED(TELESCOPE_POWER_LED, scope_pwr);
  

  // Update the physical shift register to send the LED status to the LEDs
  digitalWrite(SR_LATCH, LOW);
  shiftOut(SR_DATA_595, SR_CLOCK, MSBFIRST, ledState);
  digitalWrite(SR_LATCH, HIGH);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Publish sensor states to MQTT
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  mqtt_client.publish("home/observatory/roof/state/opened", roof_open ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/closed", roof_closed ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/ra_safe", !ra_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/dec_safe", !dec_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/telescope_safe", !telescope_safe ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/moving", (!roof_open && !roof_closed && !roof_lost) ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/lost", roof_lost ? "true" : "false", true);
  mqtt_client.publish("home/observatory/roof/state/roof_status", roof_status_text.c_str(), true);
  mqtt_client.publish("home/observatory/roof/state/power", scope_pwr ? "true" : "false", true);
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RESPOND TO MANUAL PUSHBUTTONS (includes debounce logic)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Read current status of each of the buttons from the 74HC165 shift register
 
  byte buttonState = 0;

  // Latch current state of buttons (low pulse on PL)
  digitalWrite(SR_LATCH, LOW);  // shared with 74HC595
  delayMicroseconds(5);
  digitalWrite(SR_LATCH, HIGH);

  // Read bits from Q7
  for (int i = 0; i < 8; i++) {
    buttonState <<= 1;
    buttonState |= digitalRead(SR_DATA_165);
    digitalWrite(SR_CLOCK, HIGH);
    delayMicroseconds(5);
    digitalWrite(SR_CLOCK, LOW);
  }

  // Set boolean variables to reflect the status of each of buttons based on the state (0 or 1) of the Bits within the buttonState Byte
  bool isLightsMaxPressed = !(buttonState & (1 << 0));  // D0
  bool isLightsDimPressed = !(buttonState & (1 << 1));  // D1
  bool isLightsOffPressed = !(buttonState & (1 << 2));  // D2
  bool isPWRPressed       = !(buttonState & (1 << 3));  // D3
  bool isStopPressed      = !(buttonState & (1 << 4));  // D4
  bool isOSCPressed       = !(buttonState & (1 << 5));  // D5
 
	unsigned long currentTime = millis();

  // OSC BUTTON
  if (isOSCPressed != lastOSCState) lastDebounceOSCTime = currentTime;
  if (((currentTime - lastDebounceOSCTime) > debounceDelay) && (isOSCPressed == LOW)) ToggleRoof();
  lastOSCState = isOSCPressed;
 
  // STOP BUTTON
  if (isStopPressed != lastStopState) lastDebounceStopTime = currentTime;
  if (((currentTime - lastDebounceStopTime) > debounceDelay) && (isStopPressed == LOW)) StopRoof();
  lastStopState = isStopPressed;

  // Telescope Power button
  if (isPWRPressed != lastPWRReading) {
    lastDebouncePWRTime = currentTime;
    lastPWRReading = isPWRPressed;
  }
  if ((currentTime - lastDebouncePWRTime) > debounceDelay) {
    // Only toggle if the button state has changed from HIGH to LOW (i.e. button is pressed down)
    if (lastPWRStableState == HIGH && isPWRPressed == LOW) {
      ToggleTelescopePower();
    }
    lastPWRStableState = isPWRPressed;
  }

  // Lights MAX button
  if (isLightsMaxPressed != lastLightsMaxState) lastDebounceLightsMaxTime = currentTime;
  if (((currentTime - lastDebounceLightsMaxTime) > debounceDelay) && (isLightsMaxPressed == LOW)) TurnLightsMax();
  lastLightsMaxState = isLightsMaxPressed;

  // Lights DIM button
  if (isLightsDimPressed != lastLightsDimState) lastDebounceLightsDimTime = currentTime;
  if (((currentTime - lastDebounceLightsDimTime) > debounceDelay) && (isLightsDimPressed == LOW)) TurnLightsDim();
  lastLightsDimState = isLightsDimPressed;

  // Lights OFF button
  if (isLightsOffPressed != lastLightsOffState) lastDebounceLightsOffTime = currentTime;
  if (((currentTime - lastDebounceLightsOffTime) > debounceDelay) && (isLightsOffPressed == LOW)) TurnLightsOff();
  lastLightsOffState = isLightsOffPressed;


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RESPOND TO SERIAL comms (over UART serial conenction)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Send RRCI initialisation string every 2 seconds
  now = millis();
  if (now - lastSerialInitCheck > 2000) { 
    lastSerialInitCheck = now;
    UART_Serial.print("RRCI#"); //init string
  }
    
 // Verify connection by serial
  while (UART_Serial.available() > 0) {
    
    //Read serial data from ASCOM and allocate to serialin variable
    serialin = UART_Serial.readStringUntil('#');

		if (serialin == "x") {                            // COMMAND = "x" (Emergency stop whilst opening)
      StopRoof();
    }
    else if (serialin == "y") {           			      // COMMAND = "y" (Emergency stop whilst closing)
      StopRoof();
    }
    else if (serialin == "open") {                    // COMMAND = "open", ACTION = Open roof if safe to do so
      OpenRoof();
    }
    else if (serialin == "close") {                   // COMMAND = "close", ACTION = Close roof if safe to do so
      CloseRoof();
    }
		else if (serialin == "Parkstatus") {              // COMMAND = "Parkstatus", ACTION = return the park status of the telescope
      UART_Serial.println("0#");
    }
		else if (serialin == "Status") {                  // COMMAND = "Status", ACTION = Return the string "RoofOpen"
			UART_Serial.println("RoofOpen#");
		}
		else if (serialin == "get") {                     // COMMAND = "get", ACTION = return the status of the RRCI data in the format: "ROOF_STATUS, TELESCOPE_SAFETY_STATUS, MOBILITY_STATUS#"

      // ROOF STATUS: Report the roof status (opened, closed or unknown)
      switch (roof_status) {
        case OPEN:
          str = "opened,";
          break;
        case CLOSED:
          str = "closed,";
          break;
        case MOVING:
          str = "unknown,";
          break;
        case LOST: 
          str = "unknown,";
          break;        
      }

      // SAFETY STATUS: Report the safety status of the telescope (safe or unsafe)
      if (telescope_safe) {
        str += "safe,";
      }
      else {
        str += "unsafe,";
      }

       // MOBILITY STATUS: Report mobility status of roof (not moving (open), not moving (closed), moving or unknown)
      switch (roof_status) {
        case OPEN:
          str += "not_moving_o#";
          break;
        case CLOSED:
          str += "not_moving_c#";
          break;
        case MOVING:
          str += "moving#";
          break;
        case LOST: 
          str += "unknown#";
          break;
      }

      if (str.endsWith(",")) str += "unknown#";
      
      // Write output back to ASCOM via serial interface
			UART_Serial.println(str);
    }
    
    Serial.print("Serial command received: ");
    Serial.print(serialin);
    if (serialin=="get") {
      Serial.print(" : ");
      Serial.print(str);
    }
    Serial.println();

	}	  

}

// ************************************************************************************************************************************************
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
// ************************************************************************************************************************************************

// *******************************************************************************************************
// Function to OPEN the roof, including checking that the telescope is safe
// *******************************************************************************************************

void OpenRoof () {
    ToggleRoof();
}

// *******************************************************************************************************
// Function to CLOSE the roof, including checking that the telescope is safe
// *******************************************************************************************************

void CloseRoof () {
    ToggleRoof();
}


// *******************************************************************************************************
// Function to STOP the roof
// *******************************************************************************************************

void StopRoof () {
    //digitalWrite(RELAY_OPEN, LOW);
    //digitalWrite(RELAY_CLOSE, LOW);
    digitalWrite(RELAY_STOP, HIGH); 
    delay(500);
    digitalWrite(RELAY_STOP, LOW);
    UART_Serial.println("Stopping roof...");
    Serial.println("Stopping roof...");
    http_server.send(200, "text/plain", "Stopping roof...");
}


// *******************************************************************************************************
// Function to TOGGLE the roof (if using single OSC button), including checking that the telescope is safe
// *******************************************************************************************************

void ToggleRoof () {
    if (telescope_safe) {
			digitalWrite(RELAY_OSC, HIGH);
      delay(500);
      digitalWrite(RELAY_OSC, LOW);
			if (roof_open && !roof_closed) {
        UART_Serial.println("Closing roof...");
        Serial.println("Closing roof...");
        http_server.send(200, "text/plain", "Closing roof...");
      } else if (!roof_open && roof_closed) {
        UART_Serial.println("Opening roof...");
        Serial.println("Opening roof...");
        http_server.send(200, "text/plain", "Closing roof...");
      } else {
        UART_Serial.println("Stopping roof...");
        Serial.println("Stopping roof...");
        http_server.send(200, "text/plain", "Stopping roof...");
      }
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 1350);
      delay(100);
      noTone(BUZZER_PIN);
    }	else {
      UART_Serial.println("Cannot move roof - telescope is UNSAFE");
      Serial.println("Cannot move roof - telescope is UNSAFE");
      http_server.send(200, "text/plain", "Cannot move roof - telescope is UNSAFE");
      tone(BUZZER_PIN, 1000);
      delay(1000);
      noTone(BUZZER_PIN);
    }
}


// *******************************************************************************************************
// Function to turn on power to telescope equipment via relay (mount, miniPC, dew heater, etc.)
// *******************************************************************************************************

void ToggleTelescopePower () {
  scope_pwr = !scope_pwr;

  if (scope_pwr) {
    digitalWrite(RELAY_PWR, HIGH); 
    Serial.println("Telescope power ON");
  } else {
    digitalWrite(RELAY_PWR, LOW); 
    Serial.println("Telescope power OFF");
  }
}


// *******************************************************************************************************
// Function to handle incoming MQTT commands
// *******************************************************************************************************

void callback(char* topic, byte* payload, unsigned int length) {
  if (millis() - bootTime < 5000) {
    Serial.println("Ignoring early MQTT message after boot...");
    return;  // Ignore all MQTT messages for the first 5 seconds
  }

  Serial.println("Received callback...");

  String cmd = "";
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];

  Serial.print("Topic = ");
  Serial.println(topic);
  Serial.print("CMD = ");
  Serial.println(cmd);

  if (String(topic) == "home/observatory/roof/cmd/osc_toggle" && cmd == "ON") ToggleRoof();
  if (String(topic) == "home/observatory/roof/cmd/stop" && cmd == "ON") StopRoof();
  if (String(topic) == "home/observatory/roof/cmd/power" && cmd == "ON") ToggleTelescopePower();
  if (String(topic) == "home/observatory/roof/cmd/lights_max" && cmd == "ON") TurnLightsMax();
  if (String(topic) == "home/observatory/roof/cmd/lights_dim" && cmd == "ON") TurnLightsDim();
  if (String(topic) == "home/observatory/roof/cmd/lights_off" && cmd == "ON") TurnLightsOff();
}


// *******************************************************************************************************
// Function to publish discovery messages to MQTT broker running on home assistant
// *******************************************************************************************************

void publishDiscovery() {
  if(mqtt_client.connected())
    {    
      // sensors
      mqtt_client.publish("homeassistant/binary_sensor/roof_open_sensor/config",      DISCOVERY_ROOF_OPEN, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_closed_snesor/config",    DISCOVERY_ROOF_CLOSED, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_moving/config",           DISCOVERY_ROOF_MOVING, true);
      mqtt_client.publish("homeassistant/binary_sensor/roof_lost/config",             DISCOVERY_ROOF_LOST, true);
      mqtt_client.publish("homeassistant/binary_sensor/ra_safe_sensor/config",        DISCOVERY_RA_SAFE, true);
      mqtt_client.publish("homeassistant/binary_sensor/dec_safe_sensor/config",       DISCOVERY_DEC_SAFE, true);
      mqtt_client.publish("homeassistant/binary_sensor/telescope_safe/config",        DISCOVERY_TELESCOPE_SAFE, true);
      mqtt_client.publish("homeassistant/sensor/roof_status/config",                  DISCOVERY_ROOF_STATUS, true);
      mqtt_client.publish("homeassistant/binary_sensor/power/config",                 DISCOVERY_TELESCOPE_POWER, true);
      
      // buttons
      mqtt_client.publish("homeassistant/button/roof_osc/config",                     DISCOVERY_BUTTON_OSC, true);
      mqtt_client.publish("homeassistant/button/roof_stop/config",                    DISCOVERY_BUTTON_STOP, true);
      mqtt_client.publish("homeassistant/button/power/config",                        DISCOVERY_BUTTON_PWR, true);
      mqtt_client.publish("homeassistant/button/lights_max/config",                   DISCOVERY_BUTTON_LIGHTS_MAX, true);
      mqtt_client.publish("homeassistant/button/lights_dim/config",                   DISCOVERY_BUTTON_LIGHTS_DIM, true);
      mqtt_client.publish("homeassistant/button/lights_off/config",                   DISCOVERY_BUTTON_LIGHTS_OFF, true);
    }
}


// *******************************************************************************************************
// Function to reconnect MQQT if lost
// *******************************************************************************************************

void MqttReconnect() {
  if (!mqtt_client.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt_client.connect("ESP32-Roof", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT connected");
      mqtt_client.subscribe("home/observatory/roof/cmd/#");
      publishDiscovery();
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(mqtt_client.state());
      delay(2000); // Don't flood the broker
    }
  }
}
 

// *******************************************************************************************************
// Function to get roof and telescope safety status and send via HTTP (used in HTTP GET request)
// *******************************************************************************************************

void getStatus() {
  String safeState = telescope_safe ? "true" : "false";
  String json = "{\"roof\":\"" + roof_status_text + "\", \"telescope_safe\":" + safeState + "}";
  http_server.send(200, "application/json", json);
}


// *******************************************************************************************************
// Function to get telescope safety status and send via HTTP (used in HTTP GET request)
// *******************************************************************************************************

void getSafetyStatus() {
    http_server.send(200, "application/json", telescope_safe ? "true" : "false");
  }


// *******************************************************************************************************
// Function to set the LEDs using the shift register
// *******************************************************************************************************

void setLED(uint8_t ledIndex, bool on) {
  if (on)
    ledState |= (1 << ledIndex);
  else
    ledState &= ~(1 << ledIndex);
}


// *******************************************************************************************************
// Function to turn observatory lights to MAX
// *******************************************************************************************************

void TurnLightsMax() {

  // Send MQTT message to Home Assistant to enable the "Max" lights scene
  mqtt_client.publish("observatory/lights/scene", "max");

}

// *******************************************************************************************************
// Function to turn observatory lights to DIM
// *******************************************************************************************************

void TurnLightsDim() {

  // Send MQTT message to Home Assistant to enable the "Max" lights scene
  mqtt_client.publish("observatory/lights/scene", "dim");

}

// *******************************************************************************************************
// Function to turn OFF observatory lights
// *******************************************************************************************************

void TurnLightsOff() {

  // Send MQTT message to Home Assistant to enable the "Max" lights scene
  mqtt_client.publish("observatory/lights/scene", "off");

}