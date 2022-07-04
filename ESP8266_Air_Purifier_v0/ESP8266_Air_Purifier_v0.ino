/**
 * ESP8266 Air Purifier Control
 * @author Curt Henrichs
 * @date 5-9-2022
 * 
 * We want to control an air purifier
 * 
 * Blue Wire = LO = 1.8V on (read high on ESP8266), 0V off
 *  - Indicates state
 * White Wire = Hi = 2.3V on (read high on ESP8266), 0V off
 *  - Indicates state
 * Green Wire = BTN = 5V pulled up, 0V on click
 *  - Controls state
 * 
 * For Button
 *  - Fast click to ground toggles state [Off, High, Low, Off, ...]
 *  - If on and hold button
 *    - turns off
 *      - next state is low if prev state is high
 *      - next state is high if prev state is low
 *  - holding button when off does nothing
 *  
 *  
 *  How to pick pinout
 *  https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 *  - D0 for wifi-manager reset button (since its pulled high at boot and we want to pull high for button) - not a problem as long as user isn't holding button during boot
 *  - D5 CTRL button (D5 has no pull up/down so it is safe for external output interface)
 *  - D6, D7 for Hi and Lo state from purifier (6, 7 have no pull up/down so again safe for external interface - though less of a problem)
 *  
 *  For Wifi using
 *  - https://github.com/vintlabs/fauxmoESP
 *  - https://github.com/tzapu/WiFiManager
 *  
 *  
 *  
 *  
 *  
 *  
 *  To answer your query, please use "other devices" and not Phillips Hue on the Alexa app when discovering the lights.

Phillips Hue bridge is not required,

As far as my experience with this library goes, a physical alexa device (Atleast an echo dot) is necessary for discovering devices.




There is a color mode -  its in development. Perhaps fork repo work on it then merge in?


TODO we need to properly format and finish documetation. It is a hot mess RN.


 */

//==============================================================================
//  Libraries
//==============================================================================

#include <WiFiManager.h>
#include <fauxmoESP.h>

//==============================================================================
//  Preprocessor Constants
//==============================================================================

#define RESET_BTN_PIN                           (D0)
#define CTRL_BTN_PIN                            (D5)
#define LED_HI_STATE_PIN                        (D6)
#define LED_LO_STATE_PIN                        (D7)

#define __NUM_STATES__                          (3)

#define RESET_BTN_TIME_THRESHOLD                (100L)
#define PULSE_TIME                              (100L)
#define COOLDOWN_THRESHOLD                      (500L)

//==============================================================================
//  Preprocessor Macros
//==============================================================================

#define readResetBtnState() ((bool)digitalRead(RESET_BTN_PIN))
#define readPurifierState() ((purifier_state_t)(digitalRead(LED_LO_STATE_PIN) << 1 | digitalRead(LED_HI_STATE_PIN) << 0))

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
//  Private Module Variables
//==============================================================================

static WiFiManager wifiManager;
static fauxmoESP fauxmo;

static bool blinkState = false;
static unsigned long blinkTime, resetTime, cooldownTime;
static purifier_state_t hwState;

static char rxBuffer[50];
static int rxBufferIdx = 0;

static purifier_state_t currentState;
static bool flag_reset = false;
static bool prevBtnState = false;
static int queuedPulses = 0;

//==============================================================================
//  Declared Constants
//==============================================================================

// We only fill out values that are non-zero. Errors are treated as zero transition
const transition_count_table_entry_t transitionCountLookupTable[] = {
    {   PURIFIER_OFF,  PURIFIER_HI,    1    },
    {   PURIFIER_OFF,  PURIFIER_LO,    2    },
    {   PURIFIER_HI,   PURIFIER_LO,    1    },
    {   PURIFIER_HI,   PURIFIER_OFF,   2    },
    {   PURIFIER_LO,   PURIFIER_OFF,   1    },
    {   PURIFIER_LO,   PURIFIER_HI,    2    }, 
};

const int transitionCountLookupTableSize = (sizeof(transitionCountLookupTable) / sizeof(transition_count_table_entry_t));

//==============================================================================
//  Private Function Prototypes
//==============================================================================

void fauxmo_callback(uint8_t id, const char * name, bool state, uint8_t value);
unsigned long blinkDurationByState(purifier_state_t state);
void transitionStatePulse();
int numberOfPulses(purifier_state_t currentState, purifier_state_t targetState);

//==============================================================================
//  MAIN
//==============================================================================

void setup() {
    Serial.begin(9600);
    Serial.flush();
  
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CTRL_BTN_PIN, OUTPUT);
    pinMode(LED_HI_STATE_PIN, INPUT);
    pinMode(LED_LO_STATE_PIN, INPUT);

    digitalWrite(CTRL_BTN_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);

    wifiManager.autoConnect("AP-ESP8266-Air-Purifier");

    fauxmo.addDevice("air purifier");
    fauxmo.setPort(80);
    fauxmo.enable(true);

    fauxmo.onSetState(fauxmo_callback);
  
    cooldownTime = blinkTime = resetTime = millis();
    prevBtnState = readResetBtnState();
    currentState = readPurifierState();
}

void loop() {
    // We do most things against soft timers in the main loop
    unsigned long currentTime = millis();

    // Blink the LED at a certain rate dependent on state of purifier
    //  Useful for debugging
    if ((currentTime - blinkTime) > blinkDurationByState(hwState)) {
        blinkTime = currentTime;
        digitalWrite(LED_BUILTIN, blinkState);
        blinkState = !blinkState;
    }

    // State input is either HI LED or LO LED not both. But it could be neither = OFF
    // pin reads are shifted to produce state
    // If both pins are high then we are in a hardware fault state
    currentState = readPurifierState();
  
    // Serial debugging interface provides full state control
    while (Serial.available() > 0) {
        if (rxBufferIdx >= (sizeof(rxBuffer) - 1)) {
            Serial.println(F("Error - Buffer Overflow ~ Clearing"));
            rxBufferIdx = 0;
        }

        char c = Serial.read();
        if (c != '\n') {
            rxBuffer[rxBufferIdx] = c;
            rxBufferIdx++;
        } else {
            rxBuffer[rxBufferIdx] = '\0';
            rxBufferIdx = 0;
      
            String str = String(rxBuffer);
      
            if (str == "next") {
                queuedPulses += 1;
            } else if (str == "state?") {
                Serial.print(F("STATE = "));
                Serial.println(currentState);  
            } else if (str == "reset") {
                flag_reset = true;
            } else if (str == "low") {
                queuedPulses += numberOfPulses(currentState, PURIFIER_LO);
            } else if (str == "high") {
                queuedPulses += numberOfPulses(currentState, PURIFIER_HI);
            } else if (str == "off") {
                queuedPulses += numberOfPulses(currentState, PURIFIER_OFF);
            } else {
                Serial.print(F("Error - Invalid Command ~ `"));
                Serial.print(str);
                Serial.println(F("`"));
            }
        }
    }

    // Wifi Light service
    // NOTE: Its not great that Alexa thinks its a light but it does work
    // The callback will update queued pulses
    fauxmo.handle();

    // Handle Reset button
    bool btnState = readResetBtnState();
    if (!btnState && prevBtnState) {
        resetTime = currentTime;
    } else if (btnState && !prevBtnState) {
        if ((currentTime - resetTime) >= RESET_BTN_TIME_THRESHOLD) {
            flag_reset = true;
        }
        // Otherwise button press was debounced.
    }
    prevBtnState = btnState;

    // Reset State Machine
    if (flag_reset) {
        flag_reset = false;
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wifiManager.resetSettings();
        ESP.restart();
    }

    // Handle our queued pulses. This will block for one pulse at a time!
    if (queuedPulses > 0 && (currentTime - cooldownTime) > COOLDOWN_THRESHOLD) {
        queuedPulses -= 1;
        cooldownTime = currentTime;
        transitionStatePulse();
    }
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

void fauxmo_callback(uint8_t id, const char * name, bool state, uint8_t value) {
    if (state && currentState == PURIFIER_OFF) {
        queuedPulses += numberOfPulses(currentState, PURIFIER_HI);
    } else if (!state && currentState != PURIFIER_OFF) {
        queuedPulses += numberOfPulses(currentState, PURIFIER_OFF);
    }
}

unsigned long blinkDurationByState(purifier_state_t state) {
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

void transitionStatePulse() {
    Serial.println("Pulse");
    digitalWrite(CTRL_BTN_PIN, HIGH);
    delay(PULSE_TIME);
    digitalWrite(CTRL_BTN_PIN, LOW);
}

int numberOfPulses(purifier_state_t currentState, purifier_state_t targetState) {
    for (int i=0; i<transitionCountLookupTableSize; i++) {
        if (transitionCountLookupTable[i].current == currentState && transitionCountLookupTable[i].target == targetState) {
            return transitionCountLookupTable[i].transitionCount;
        }
    }

    return 0; // If no match in table, default to zero
}
