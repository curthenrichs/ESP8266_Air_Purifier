/**
 * ESP8266 Air Purifier Control
 * @author Curt Henrichs
 * @date 5-9-2022
 * 
 * We want to control an air purifier
 * 
 * Blue Wire = LO = 1.8V on, 0V off
 *  - Indicates state
 * White Wire = Hi = 2.3V on, 0V off
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





 */

#include <WiFiManager.h>
#include <fauxmoESP.h>

#define CTRL_BTN_PIN        D5
#define LED_HI_STATE_PIN    D6
#define LED_LO_STATE_PIN    D7


typedef enum purifier_state {
  PURIFIER_OFF = 0,
  PURIFIER_HI = 1,
  PURIFIER_LO = 2,
  PURIFIER_FAULT = 3    //! Shouldn't happen during normal operation
} purifier_state_t;


static WiFiManager wifiManager;
static fauxmoESP fauxmo;

static bool blinkState = false;
static unsigned long blinkTime;
static purifier_state_t hwState;
static char rxBuffer[50];
static int rxBufferIdx = 0;


void setup() {
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CTRL_BTN_PIN, OUTPUT);
  pinMode(LED_HI_STATE_PIN, INPUT);
  pinMode(LED_LO_STATE_PIN, INPUT);

  digitalWrite(CTRL_BTN_PIN, LOW);

  wifiManager.autoConnect("AP-ESP8266-Air-Purifier");

  //TODO should this only be enabled when wifimanager successfully connects?
  fauxmo.addDevice("air purifier");
  fauxmo.setPort(80);
  fauxmo.enable(true);

  fauxmo.onSetState(fauxmo_callback);
  
  blinkTime = millis();

  Serial.flush();
}

void fauxmo_callback(unsigned char device_id, const char * device_name, bool state, unsigned char value) {
  Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
}


void loop() {
  unsigned long currentTime = millis();

  // Blink the LED at a certain rate dependent on state of purifier
  //  Useful for debugging
  if (currentTime - blinkTime > blinkDurationByState(hwState)) {
    blinkTime = currentTime;
    digitalWrite(LED_BUILTIN, blinkState);
    blinkState = !blinkState;
  }
  
  // State input is either HI LED or LO LED not both. But it could be neither = OFF
  // pin reads are shifted to produce state
  // If both pins are high then we are in a hardware fault state
  hwState = (purifier_state_t)(digitalRead(LED_LO_STATE_PIN) << 1 | digitalRead(LED_HI_STATE_PIN) << 0);
  

  // Control - We would trigger this via some wifi interface in the future
  // For now I will just use a serial interface
  while (Serial.available() > 0) {
    if (rxBufferIdx >= (sizeof(rxBuffer) - 1)) {
      Serial.println(F("Error - Buffer Overflow ~ Clearing"));
      rxBufferIdx = 0;
    }

    char c = Serial.read();
    if (c == '\n') {
      rxBuffer[rxBufferIdx] = '\0';
      
      String str = String(rxBuffer);
      if (str == "next") {
        transitionState();
      } else if (str == "state?") {
        Serial.print(F("HW STATE = "));
        Serial.println(hwState);
      } else {
        Serial.print(F("Error - Invalid Command ~ `"));
        Serial.print(str);
        Serial.println(F("`"));
      }

      rxBufferIdx = 0;
    } else {
      rxBuffer[rxBufferIdx] = c;
      rxBufferIdx++;
    }
  }

  fauxmo.handle();
}

unsigned long blinkDurationByState(purifier_state_t st) {
  switch (st) {
    case PURIFIER_OFF:
      return 2000;
    case PURIFIER_HI:
      return 500;
    case PURIFIER_LO:
      return 500;
    case PURIFIER_FAULT:
      return 250;
    default:
      return 250; 
  }
}

void transitionState() {
  digitalWrite(CTRL_BTN_PIN, HIGH);
  delay(500);
  digitalWrite(CTRL_BTN_PIN, LOW);
}
