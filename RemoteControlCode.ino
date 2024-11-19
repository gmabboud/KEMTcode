//Dependencies
#include <Wire.h>         
#include <string>
#include <sstream>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"

//For LED
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

//Constants
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 14 // dBm
#define LORA_BANDWIDTH 0 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 // Define the payload size here

#define JOYSTICK1_NEUTRAL 2650  // Center of the throttle
#define JOYSTICK2_NEUTRAL 2860  // Center of the steering

#define JOYSTICK1_DEADZONE 350  // Deadzone for the throttle
#define JOYSTICK2_DEADZONE 350  // Deadzone for the steering

//Global Variables
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr, freq, SDA, SCL, resolution, rst

// These messages will prevent bogus messages from being interpretted
const uint8_t CONTROLLERMSG_CODE[8] = "UK RC  ";
const uint8_t BOATMSG_CODE[8] = "UK BOAT";
// This struct contains commands for the boat
union controllerMsg {
  struct {
    uint8_t secretCode[8];
    bool isGoingForward;
    bool isGoingBackward;
    bool isSteeringLeft;
    bool isSteeringRight;
    bool killswitch;
    //bool automation;
  } status;
  uint8_t str[13];
};

union boatMsg {
  struct {
    uint8_t secretCode[8];
    int16_t lastRSSI;
    int16_t steeringPosition;
    float latitude;
    float longitude;
    float speed;
  } status;
  uint8_t str[24];
} inboundMsg;

void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );

int signalStrength = 0;
uint32_t signalTime = 0;

int displaySelector = 48;
char displayString[32];

// Functions declaration
typedef void (*DisplayFunc)(void);

//Event handlers
static RadioEvents_t RadioEvents = {};
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

controllerMsg updateStatusVals() {
  controllerMsg outboundMsg = {0};
  memcpy(outboundMsg.status.secretCode, CONTROLLERMSG_CODE, sizeof(CONTROLLERMSG_CODE));  
  
  // Update throttle commands
  int reading1 = analogRead(ADC1);
    if (reading1 > JOYSTICK1_NEUTRAL + JOYSTICK1_DEADZONE || reading1 < JOYSTICK1_NEUTRAL - JOYSTICK1_DEADZONE) {
      // Joystick is outside the deadzone
      if (reading1 > JOYSTICK1_NEUTRAL + JOYSTICK1_DEADZONE) {
        outboundMsg.status.isGoingForward = true;
        outboundMsg.status.isGoingBackward = false;
      } else {
        outboundMsg.status.isGoingForward = false;
        outboundMsg.status.isGoingBackward = true;
      }
    } else {
      // Joystick is within the deadzone
      outboundMsg.status.isGoingForward = false;
      outboundMsg.status.isGoingBackward = false;
    }

    // Update steering commands with deadzone for Joystick 2
    int reading2 = analogRead(ADC2);
    if (reading2 > JOYSTICK2_NEUTRAL + JOYSTICK2_DEADZONE || reading2 < JOYSTICK2_NEUTRAL - JOYSTICK2_DEADZONE) {
      // Joystick is outside the deadzone
      if (reading2 > JOYSTICK2_NEUTRAL + JOYSTICK2_DEADZONE) {
        outboundMsg.status.isSteeringRight = true;
        outboundMsg.status.isSteeringLeft = false;
      } else {
        outboundMsg.status.isSteeringRight = false;
        outboundMsg.status.isSteeringLeft = true;
      }
    } else {
      // Joystick is within the deadzone
      outboundMsg.status.isSteeringRight = false;
      outboundMsg.status.isSteeringLeft = false;
    }

  // Update killswitch
  outboundMsg.status.killswitch = digitalRead(GPIO3);
  
  // Update automation
  //outboundMsg.status.automation = digitalRead(GPIO7);

  return outboundMsg;
}


// ===== SCREENDISPLAY.INO =====

void displayTeamName() {
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawStringMaxWidth(0, 0, 128, "Team 01: KEMT");
}

void displayThrottle() {
  int throtReading = analogRead(ADC1);
  
  if (throtReading < JOYSTICK1_NEUTRAL + JOYSTICK1_DEADZONE && throtReading > JOYSTICK1_NEUTRAL - JOYSTICK1_DEADZONE) {
    snprintf(displayString, sizeof(displayString), "Idle:  %d", throtReading);
  } else if (throtReading > JOYSTICK1_NEUTRAL + JOYSTICK1_DEADZONE) {
    snprintf(displayString, sizeof(displayString), "Forward: %d", throtReading);
  } else {
    snprintf(displayString, sizeof(displayString), "Reverse:  %d", throtReading);
  }

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Throttle");
  display.drawString(0, 20, displayString);
}

void displaySteering() {
  int steerReading = analogRead(ADC2);
  
  if (steerReading < JOYSTICK2_NEUTRAL + JOYSTICK2_DEADZONE && steerReading > JOYSTICK2_NEUTRAL - JOYSTICK2_DEADZONE) {
    snprintf(displayString, sizeof(displayString), "Center:   %d", steerReading);
  } else if (steerReading > JOYSTICK2_NEUTRAL + JOYSTICK2_DEADZONE) {
    snprintf(displayString, sizeof(displayString), "Left: %d", steerReading);
  } else {
    snprintf(displayString, sizeof(displayString), "Right:  %d", steerReading);
  }

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Steering");
  display.drawString(0, 20, displayString);
}

void displayKillSwitch() {
  if (digitalRead(GPIO3)) {
    snprintf(displayString, sizeof(displayString), "Killswitch ON");
  } else {
    snprintf(displayString, sizeof(displayString), "Killswitch OFF");
  }

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, displayString);
}

// void displayAutomation() {
//   if (digitalRead(GPIO7)) {
//     snprintf(displayString, sizeof(displayString), "Automation ON");
//   } else {
//     snprintf(displayString, sizeof(displayString), "Automation OFF");
//   }

//   display.setTextAlignment(TEXT_ALIGN_LEFT);
//   display.setFont(ArialMT_Plain_16);
//   display.drawString(0, 0, displayString);
// }

void displayConnectionStrength() {
  if (millis() - signalTime < 1000) {
    snprintf(displayString, sizeof(displayString), "boat RSSI: %d", inboundMsg.status.lastRSSI);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, displayString);
    snprintf(displayString, sizeof(displayString), "controller RSSI: %d", signalStrength);
    display.drawString(0, 20, displayString);
  } else {
    snprintf(displayString, sizeof(displayString), "Timed out: %d", millis() - signalTime);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, displayString);
  }
}

void displayPosition() {
  snprintf(displayString, sizeof(displayString), "lat: %f.3", inboundMsg.status.latitude);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, displayString);
  snprintf(displayString, sizeof(displayString), "lng: %f.3", inboundMsg.status.latitude);
  display.drawString(0, 20, displayString);
}

void displaySpeed() {
  snprintf(displayString, sizeof(displayString), "speed (mph): %f.3", inboundMsg.status.speed);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, displayString);
}

void displaySteeringPosition() {
  snprintf(displayString, sizeof(displayString), "steerPos: %d", inboundMsg.status.steeringPosition);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, displayString);
}

void VextON(void) {
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {//Vext default OFF
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void writeDisplay(int select) {
  select %= 8;
  // Removed displayAutomation, from the outputs list
  DisplayFunc outputs[] = {displayTeamName, displayThrottle, displaySteering, displayKillSwitch, displayConnectionStrength, displayPosition, displaySpeed, displaySteeringPosition};

  // Clear the display
  display.clear();

  // Draw the current display method
  outputs[select]();

  display.display();  
}

// ===== LORAHANDLING.INO =====

void LoRaSetup() {
  RadioEvents.RxTimeout = onRxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true );


}

void onRxTimeout() {
  Serial.println("Rx Timeout");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  signalStrength = rssi;
  signalTime = millis();
  Serial.println("Received");
  if (size == sizeof(boatMsg)) {
    boatMsg temp;
    memcpy(temp.str, payload, sizeof(boatMsg));
    if (memcmp(temp.status.secretCode, BOATMSG_CODE, sizeof(BOATMSG_CODE)) == 0) {
      memcpy(inboundMsg.str, payload, sizeof(inboundMsg));
      Serial.printf("Copied %s, rssi: %d, pos: %d, lat: %.2f, lon: %.2f, speed: %.2f \n", 
        inboundMsg.status.secretCode, inboundMsg.status.lastRSSI, inboundMsg.status.steeringPosition,
        inboundMsg.status.latitude, inboundMsg.status.longitude, inboundMsg.status.speed);
      // Serial.println("Copied.");
    }

  }
}

void OnTxDone() {
  // Serial.printf("Copied %s, fwd: %d, rev: %d, left: %d, right: %d, kill: %d \n",
  //   outboundMsg.status.secretCode, outboundMsg.status.isGoingForward, outboundMsg.status.isGoingBackward,
  //   outboundMsg.status.isSteeringLeft, outboundMsg.status.isSteeringRight, outboundMsg.status.killswitch);
  Serial.println("Tx Done");
}

void OnTxTimeout() {
  Serial.println("Tx Timeout");
}

//Setup function
void setup() {
  Serial.begin(115200);

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  //Serial.println();
  //VextON();
  //delay(100);

  // Initialize the display
  display.init();
  display.setFont(ArialMT_Plain_10);
  
  // Read button press
  pinMode(GPIO1, INPUT_PULLDOWN);
  pinMode(GPIO2, INPUT_PULLDOWN);
  pinMode(GPIO3, INPUT_PULLDOWN);
  pinMode(GPIO7, INPUT_PULLDOWN);
  
}

void loop() {
  // Serial.printf("Status %s, fwd: %d, rev: %d, left: %d, right: %d, kill: %d \n",
  //   outboundMsg.status.secretCode, outboundMsg.status.isGoingForward, outboundMsg.status.isGoingBackward,
  //   outboundMsg.status.isSteeringLeft, outboundMsg.status.isSteeringRight, outboundMsg.status.killswitch);

  static uint32_t debounceTimer = 0;

  // If button pressed, switch display.
  if (digitalRead(GPIO1) && (millis() - debounceTimer > 500)) {
    debounceTimer = millis();
    displaySelector++;
  }

  // If button pressed, switch display.
  if (digitalRead(GPIO2) && (millis() - debounceTimer > 500)) {
    debounceTimer = millis();
    displaySelector--;
  }

  // Loops through the displays
  writeDisplay(displaySelector);
  
  // Listen for boat signals
  Radio.Rx(100);
  delay(100);
  Radio.IrqProcess();
  
  // Package up myRC data and send it to the other module
  Radio.Send(updateStatusVals().str, sizeof(controllerMsg)); 
  delay(100);
  Radio.IrqProcess();

}