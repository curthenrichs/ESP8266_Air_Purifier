/**
 * @file ESP8266_Air_Purifier_v1.ino
 * @author Curt Henrichs
 * @brief ESP2866 Air Purifier Project
 * @version 1.1
 * @date 2023-02-12
 * @copyright Copyright (c) 2023
 * 
 * Controls "hacked" KIOIS air filter using ESP8266. Filter state is observed 
 * through two state input pins (HI and LO both connected to LEDs on original 
 * board). State is toggled by "clicking" the button on original board. By 
 * clicking I mean using a CMOS to toggle the input pin. To transition from
 * current state to target state requires potentially several pulses each
 * needing to be spaced out in time to prevent the original board from
 * filtering out the extra pulses.
 * 
 * This device uses standard Polip-lib workflow to push current state on
 * unexpected change, poll server for its current state of this device, and
 * interface with the get value behavior for server-client communication. We
 * will not use push sensors or push errors in this code.
 * 
 * Dependency installation:
 * ---
 * - Arduino_JSON (via Arduino IDE)
 * - ESP8266 Board (via Arduino IDE)
 * - WiFiManager (via Arduino IDE)
 * - NTPClient (via Arduino IDE)
 * - Arduino Crypto (via Arduino IDE)
 *      - (I had to manually rename the library from Crypto.h to ArduinoCrypto.h)
 * 
 * Debug Serial:
 * ---
 * Several commands available for testing behavior
 * 
 *  [reset] -> Requests hardware state reset (will wipe EEPROM)
 *  [state?] -> Queries current state code
 *  [next] -> Triggers single state change pulse ("next state")
 *  [low] -> Sets state to low speed
 *  [high] -> Sets state to high speed
 *  [off] -> Sets state to off
 *  [error?] -> Queries error status of polip lib
 * 
 * Hardware Theory of Operation:
 * ---
 * Soldered onto PCB ~
 *  Blue Wire = LO = 1.8V on (read high on ESP8266), 0V off
 *   - Indicates state
 *  White Wire = HI = 2.3V on (read high on ESP8266), 0V off
 *   - Indicates state
 *  Green Wire = BTN = 5V pulled up, 0V on click
 *   - Control state
 * 
 * For Button ~
 *   We tied green wire to a MOSFET actuated by ESP8266
 *   Fast click to ground toggles state [Off, High, Low, Off, ...]
 *   If on and hold button
 *     - turns off
 *       - next state is low if prev state is high
 *       - next state is high if prev state is low
 *     - holding button when off does nothing
 * 
 * For ESP8266 Pinout, followed guide ~
 * https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 *   D0 for wifi-manger reset button (pulled high at boot and we also pull high for btn)
 *   D5 CTRL button (D5 has no pull up/down)
 *   D6, D7 for Hi and Lo state from purifier (no pull up / down so safe for external interface)
 *   Builtin LED for status
 * 
 * Tracking state transition:
 * ---
 * All queued button presses are tracked as an integer counter. When a new press
 * is triggered, the firmware flips a bool to true. State change tracker
 * checks this flag to determine if the state change was induced by firmware
 * or user press. If unexpected change (transition bool is false but state
 * changed) then we know to push new state to polip server.
 */

//==============================================================================
//  Libraries
//==============================================================================

#include <Arduino.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <polip-client.h>
#include <ESP8266HTTPClient.h>

//==============================================================================
//  Preprocessor Constants
//==============================================================================

#define RESET_BTN_PIN                           (D0)
#define CTRL_BTN_PIN                            (D5)
#define LED_HI_STATE_PIN                        (D6)
#define LED_LO_STATE_PIN                        (D7)
#define STATUS_LED_PIN                          (LED_BUILTIN)

#define __NUM_STATES__                          (3)

#define RESET_BTN_TIME_THRESHOLD                (200L)
#define PULSE_TIME                              (100L)
#define COOLDOWN_THRESHOLD                      (500L)

#define DEBUG_SERIAL_BAUD                       (115200)

#define NTP_URL                                 "pool.ntp.org"
#define FALLBACK_AP_NAME                        "AP-ESP8266-Air-Purifier"

//==============================================================================
//  Preprocessor Macros
//==============================================================================

#define readResetBtnState() ((bool)digitalRead(RESET_BTN_PIN))

#define readPurifierState() (                                                  \
    (purifier_state_t)(digitalRead(LED_LO_STATE_PIN) << 1                      \
    | digitalRead(LED_HI_STATE_PIN) << 0)                                      \
)

#define writeStatusLED(state) (digitalWrite(STATUS_LED_PIN, (bool)(state)))

//==============================================================================
//  Enumerated Constants
//==============================================================================

typedef enum purifier_state {
    PURIFIER_OFF    = 0,
    PURIFIER_HI     = 1,
    PURIFIER_LO     = 2,
    PURIFIER_FAULT  = 3    //! Shouldn't happen during normal operation
} purifier_state_t;

//==============================================================================
//  Data Structure Declaration
//==============================================================================

typedef struct transition_count_table_entry {
    purifier_state_t current;
    purifier_state_t target;
    int transitionCount;
} transition_count_table_entry_t;

//==============================================================================
//  Declared Constants
//==============================================================================

const char* SERIAL_STR = "air-purifier-0-0000";
const char* KEY_STR = "revocable-key-0";    //NOTE: Should be configurable
const char* HARDWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1);
const char* FIRMWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1);

// We only fill out values that are non-zero. Errors are treated as zero transition
const transition_count_table_entry_t TRANSITION_COUNT_LOOKUP_TABLE[] = {
    {   PURIFIER_OFF,  PURIFIER_LO,    2    },
    {   PURIFIER_OFF,  PURIFIER_HI,    1    },
    {   PURIFIER_HI,   PURIFIER_OFF,   2    },
    {   PURIFIER_HI,   PURIFIER_LO,    1    },
    {   PURIFIER_LO,   PURIFIER_HI,    2    }, 
    {   PURIFIER_LO,   PURIFIER_OFF,   1    },
};

const int TRANSITION_COUNT_LOOKUP_TABLE_SIZE = (sizeof(TRANSITION_COUNT_LOOKUP_TABLE) 
                                             / sizeof(transition_count_table_entry_t));

//==============================================================================
//  Private Module Variables
//==============================================================================

static StaticJsonDocument<POLIP_MIN_RECOMMENDED_DOC_SIZE> _doc;
static WiFiUDP _ntpUDP;
static NTPClient _timeClient(_ntpUDP, NTP_URL, 0);
static WiFiManager _wifiManager;
static polip_device_t _polipDevice;
static polip_workflow_t _polipWorkflow;

static bool _blinkState = false;
static unsigned long _blinkTime, _resetTime, _cooldownTime;
static purifier_state_t _currentState, _prevState;
static char _rxBuffer[50];
static int _rxBufferIdx = 0;
static bool _prevBtnState = false;
static int _queuedPulses = 0;
static bool _flag_reset = false;
static bool _transitionActive = false;

static char _transmissionBuffer[POLIP_MIN_ARBITRARY_MSG_BUFFER_SIZE];

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static unsigned long _blinkDurationByState(purifier_state_t state);
static void _transitionStatePulse(void);
static int _numberOfPulses(purifier_state_t currentState, purifier_state_t targetState);
static purifier_state_t _targetState(purifier_state_t currentState, int numberPulses);
static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc);
static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc);
static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source, polip_ret_code_t error);

//==============================================================================
//  MAIN
//==============================================================================

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.flush();

    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(CTRL_BTN_PIN, OUTPUT);
    pinMode(LED_HI_STATE_PIN, INPUT);
    pinMode(LED_LO_STATE_PIN, INPUT);

    digitalWrite(CTRL_BTN_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    _wifiManager.autoConnect(FALLBACK_AP_NAME);

    _timeClient.begin();

    POLIP_BLOCK_AWAIT_SERVER_OK();

    _polipDevice.serialStr = SERIAL_STR;
    _polipDevice.keyStr = (const uint8_t*)KEY_STR;
    _polipDevice.keyStrLen = strlen(KEY_STR);
    _polipDevice.hardwareStr = HARDWARE_STR;
    _polipDevice.firmwareStr = FIRMWARE_STR;
    _polipDevice.skipTagCheck = false;
    _polipDevice.buffer = _transmissionBuffer;
    _polipDevice.bufferLen = sizeof(_transmissionBuffer);

    _polipWorkflow.device = &_polipDevice;
    _polipWorkflow.hooks.pushStateSetupCb = _pushStateSetup;
    _polipWorkflow.hooks.pollStateRespCb = _pollStateResponse;
    _polipWorkflow.hooks.workflowErrorCb = _errorHandler;

    unsigned long currentTime = millis();
    polip_workflow_initialize(&_polipWorkflow, currentTime);
    _cooldownTime = _blinkTime = _resetTime = currentTime;
    _prevBtnState = readResetBtnState();
    _prevState = _currentState = readPurifierState();
    _flag_reset = false;
    _transitionActive = false;
}

void loop() {
    // We do most things against soft timers in the main loop
    unsigned long currentTime = millis();

    // Refresh time
    _timeClient.update();

    // State input is either HI LED or LO LED not both. But it could be neither = OFF
    // pin reads are shifted to produce state
    // If both pins are high then we are in a hardware fault state
    _prevState = _currentState;
    _currentState = readPurifierState();
    if (_prevState != _currentState) {
        if (_transitionActive) { // Expected transition occurred
            _transitionActive = false;
            Serial.println(F("Expected state transition detected"));
        } else {
            Serial.println(F("Unexpected state transition - will push new state to server"));
            POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            _queuedPulses = 0; // Override any future changes since user must have pressed button
        }
    }

    // Update Polip Server
    polip_workflow_periodic_update(&_polipWorkflow, _doc, _timeClient.getFormattedDate().c_str(), currentTime);

    // Serial debugging interface provides full state control
    while (Serial.available() > 0) {
        if (_rxBufferIdx >= (sizeof(_rxBuffer) - 1)) {
            Serial.println(F("Error - Buffer Overflow ~ Clearing"));
            _rxBufferIdx = 0;
        }

        char c = Serial.read();
        if (c != '\n') {
            _rxBuffer[_rxBufferIdx] = c;
            _rxBufferIdx++;
        } else {
            _rxBuffer[_rxBufferIdx] = '\0';
            _rxBufferIdx = 0;
      
            String str = String(_rxBuffer);
      
            if (str == "next") {
                _queuedPulses += 1;
            } else if (str == "state?") {
                Serial.print(F("STATE = "));
                Serial.println(_currentState);  
            } else if (str == "reset") {
                Serial.println(F("Debug Reset Requested"));
                _flag_reset = true;
            } else if (str == "low") {
                _queuedPulses += _numberOfPulses(_currentState, PURIFIER_LO);
            } else if (str == "high") {
                _queuedPulses += _numberOfPulses(_currentState, PURIFIER_HI);
            } else if (str == "off") {
                _queuedPulses += _numberOfPulses(_currentState, PURIFIER_OFF);
            } else if (str == "error?") {
                if (POLIP_WORKFLOW_IN_ERROR(&_polipWorkflow)) {
                    Serial.println(F("Error in PolipLib: "));
                    Serial.println((int)_polipWorkflow.flags.error);
                } else {
                    Serial.println(F("No Error"));
                }
                POLIP_WORKFLOW_ACK_ERROR(&_polipWorkflow);
            } else {
                Serial.print(F("Error - Invalid Command ~ `"));
                Serial.print(str);
                Serial.println(F("`"));
            }
        }
    }

    // Handle Reset button
    bool btnState = readResetBtnState();
    if (!btnState && _prevBtnState) {
        _resetTime = currentTime;
    } else if (btnState && !_prevBtnState) {
        if ((currentTime - _resetTime) >= RESET_BTN_TIME_THRESHOLD) {
            Serial.println("Reset Button Held");
            _flag_reset = true;
        }
        // Otherwise button press was debounced.
    }
    _prevBtnState = btnState;

     // Reset State Machine
    if (_flag_reset) {
        _flag_reset = false;
        Serial.println("Erasing Config, restarting");
        _wifiManager.resetSettings();
        ESP.restart();
    }

    // Handle our queued pulses. This will block for one pulse at a time!
    if (_queuedPulses > 0 && (currentTime - _cooldownTime) > COOLDOWN_THRESHOLD) {
        _queuedPulses -= 1;
        _cooldownTime = currentTime;
        _transitionActive = true;
        _transitionStatePulse();
    }

    // Blink the LED at a certain rate dependent on state of purifier
    //  Useful for debugging
    if ((currentTime - _blinkTime) > _blinkDurationByState(_currentState)) {
        _blinkTime = currentTime;
        writeStatusLED(_blinkState);
        _blinkState = !_blinkState;
    }

    delay(1);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static unsigned long _blinkDurationByState(purifier_state_t state) {
    switch (state) {
        case PURIFIER_OFF:
            return 2000;    // 2 Sec - Really slow
        case PURIFIER_HI:
            return 500;     // Normal speed
        case PURIFIER_LO:
            return 500;     // Normal speed
        case PURIFIER_FAULT:
            return 250;     // Fast == Sad
        default:
            return 250; 
    }
}

static void _transitionStatePulse() {
    Serial.println("Pulse");
    digitalWrite(CTRL_BTN_PIN, HIGH);
    delay(PULSE_TIME);
    digitalWrite(CTRL_BTN_PIN, LOW);
}

static int _numberOfPulses(purifier_state_t currentState, purifier_state_t targetState) {
    for (int i=0; i<TRANSITION_COUNT_LOOKUP_TABLE_SIZE; i++) {
        if (TRANSITION_COUNT_LOOKUP_TABLE[i].current == currentState 
                && TRANSITION_COUNT_LOOKUP_TABLE[i].target == targetState) {
            return TRANSITION_COUNT_LOOKUP_TABLE[i].transitionCount;
        }
    }

    return PURIFIER_FAULT; // If no match in table, default to zero
}

static purifier_state_t _targetState(purifier_state_t currentState, int numberPulses) {
    if (numberPulses < 0) {
        return PURIFIER_FAULT;
    }

    purifier_state_t finalState = currentState;
    while (numberPulses > 0) {
        for (int i=0; i<TRANSITION_COUNT_LOOKUP_TABLE_SIZE; i++) {
            if (TRANSITION_COUNT_LOOKUP_TABLE[i].current == currentState 
                    && (numberPulses - TRANSITION_COUNT_LOOKUP_TABLE[i].transitionCount) >= 0) {
                numberPulses -= TRANSITION_COUNT_LOOKUP_TABLE[i].transitionCount;
                finalState = TRANSITION_COUNT_LOOKUP_TABLE[i].target;
            }
        }
    }

    return finalState;
}

static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc.createNestedObject("state");

    switch (_targetState(_currentState, _queuedPulses)) {
        case PURIFIER_OFF:
            stateObj["power"] = "off";
            stateObj["mode"] = "hi";
            break;
        case PURIFIER_HI:
            stateObj["power"] = "on";
            stateObj["mode"] = "hi";
            break;
        case PURIFIER_LO:
            stateObj["power"] = "on";
            stateObj["mode"] = "lo";
            break;
        case PURIFIER_FAULT:
            stateObj["power"] = "on";
            stateObj["mode"] = "error";
            break;
    }
}

static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc["state"];
    const char* power = stateObj["power"];
    const char* mode = stateObj["mode"];

    purifier_state_t serverState;
    if (strcmp(power,"off") == 0) {
        serverState = PURIFIER_OFF;
    } else if (strcmp(power,"on") == 0) {
        if (strcmp(mode,"hi") == 0) {
            serverState = PURIFIER_HI;
        } else if (strcmp(mode,"lo") == 0) {
            serverState = PURIFIER_LO;
        } else {
            serverState = PURIFIER_FAULT;
            Serial.println(F("INVALID MODE PROVIDED BY SERVER"));
        }
    } else {
        serverState = PURIFIER_FAULT;
        Serial.println(F("INVALID POWER PROVIDED BY SERVER"));
    }

    if ((serverState != PURIFIER_FAULT) 
            && (_targetState(_currentState, _queuedPulses + (int)_transitionActive) != serverState)) {

        if (_targetState(_currentState, (int)_transitionActive) == serverState) {
            _queuedPulses = 0;
        } else {
            _queuedPulses = _numberOfPulses(_currentState, serverState) - (int)_transitionActive;
        }
    }
}

static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source, polip_ret_code_t error) { 
    Serial.print(F("Error Handler ~ polip server error during OP="));
    Serial.print((int)source);
    Serial.print(F(" with CODE="));
    Serial.println((int)error);
} 