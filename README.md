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
