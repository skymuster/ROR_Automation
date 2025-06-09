# ROR_Automation
ESP32 sketch to automate roll-off-roof

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

4. Serial Control:
    Controlled via a wired serial connection between ESP32 and miniPC using UART pins on the ESP32. On the miniPC a CP2102 USB to serial adapter
    module is used as this allows longer cable runs than possible with a standard USB cable. The ASCOM Rolling Roof Computer Interface (RRCI) by 
    Chuck Faranda is installed on the miniPC which sends and receives serial commands to the ESP32. This sketch is designed to handle the specific
    commands from that interface.

5. Home Assistant / MQTT:
    Controlled using MQTT via WiFi connection between ESP32 and MQTT broker running on Home Assistant server. This allows Home Assistant to 
    monitor and control roof operation via smart switches, phone apps and touchscreens.

Note: All 4 mechanisms only allow roof movement when telescope is “safe” (i.e. RA and DEC axes are both parked in the safe position, as 
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


Roll-off Roof Automation Control System - Architecture Diagram:

![image](https://github.com/user-attachments/assets/47a95807-3e92-4d52-997b-7ef8150df374)

User Interface Panel - Wiring diragram:

![image](https://github.com/user-attachments/assets/ba2b9b0c-5f99-440d-9e75-b2855d36a397)

User Interface Panel - PCB Layout:

![image](https://github.com/user-attachments/assets/1ed39da3-4d87-4be0-92aa-10ca3a9a5c2e)

ROR Control Box - Wiring Diagram:

![image](https://github.com/user-attachments/assets/3ed933b1-76f4-4974-a50c-0ebb864d1b17)
