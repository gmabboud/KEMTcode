#include <Wire.h>         
#include <string>
#include <sstream>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"


// LoRa Constants
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 21 // dBm  // 22 is the max value
#define LORA_BANDWIDTH 0 // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 2 // Define the payload size here
#define TIMEOUT 5 // May need to adjust this

Air530ZClass GPS;

// Control code messages; this message will prevent bogus messages from giving incorrect signals to the boat
const uint8_t CONTROLLERMSG_CODE[8] = "UK RC  ";
const uint8_t BOATMSG_CODE[8] = "UK BOAT";

// This struct contains commands for the boat
// TODO: Change these to be ints to reflect new boat and controller code
union controllerMsg {
    struct {
      uint8_t secretCode[8];
      bool isGoingForward;
      bool isGoingBackward;
      //uint8_t throttlePercentage;
      bool isSteeringLeft;
      bool isSteeringRight;
      bool killswitch;
      //bool automation;
    } status;
    uint8_t str[13];
  } inboundMsg;
  
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
};

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
uint32_t signalTime = 0;
int16_t lastRSSI = 0;

// UART Communication Variables
uint8_t serialBuffer[BUFFER_SIZE];
int bufferSize;
bool automationMode = false; // Flag for automation mode

typedef void (*DisplayFunc)(void);
DisplayFunc outputs;

// Display Message
String serialMessage = "";
static SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr, freq, SDA, SCL, resolution, rst


void setup() {
    Serial.begin(9600); // Initialize UART for communication with other Heltec
    Serial1.begin(9600); // Initialize UART for communication with Raspberry Pi

    // // LoRa Setup
    // RadioEvents.RxDone = OnRxDone;
    // RadioEvents.RxTimeout = OnRxTimeout;
    // RadioEvents.TxDone = OnTxDone;
    // RadioEvents.TxTimeout = OnTxTimeout;

    // Radio.Init( &RadioEvents );
    // Radio.SetChannel( RF_FREQUENCY );
    // Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    //                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    //                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    //                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    // Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    //                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
    //                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    //                     0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
                    
    GPS.begin();

    // Serial Debug Display
    outputs = displaySerial;
    // Initialize the display
    display.init();
    display.setFont(ArialMT_Plain_10);
}

void loop() {
    // Read UART messages from Raspberry Pi
    bufferSize = Serial1.read(serialBuffer, TIMEOUT);
    
    if (bufferSize) {  // If a valid message is received
        // Process the message
        processUARTMessage(serialBuffer, bufferSize);
    }

    // Debug Displays
    // Clear the display
    display.clear();
    // Draw the current display method
    outputs();
    display.display();  


    // Read LoRa serial messages from remote controller
    //Radio.Rx(500);
    //delay(100);
    //Radio.IrqProcess();

    // Perform actions based on automation or remote control
    doActions();

    delay(100);  // Reduce CPU usage?
    // // IDK if this is necessary either
    //Radio.IrqProcess();
}


void doActions() {
    
}


// Serial Debug Display function
void displaySerial() {
    char messageBuffer[20];  // Small buffer for formatted output
    //char secondMessageBuffer[20];

    // Format: "T:XX S:XX" (Throttle and Steering values)
    snprintf(messageBuffer, sizeof(messageBuffer), "T:%d S:%d", serialBuffer[0], serialBuffer[1]);
    //snprintf(secondMessageBuffer, sizeof(secondMessageBuffer))

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Serial Message");
    display.drawString(0, 20, messageBuffer);  // Display formatted throttle & steering
}

// Function to extract throttle and steering from received message
void processUARTMessage(uint8_t *message, int length) {
    if (length == 2) {  // Expect exactly 2 bytes (throttle and steering)
        // uint8_t throttle = message[0];  // First byte = throttle (0-100)
        // uint8_t steering = message[1];  // Second byte = steering (0-100)
        //automationThrottle = static_cast<int>(message[0]);
        //automationSteering = static_cast<int>(message[1]);

        // Serial.print("Received Throttle: ");
        // Serial.println(throttle);
        // Serial.print("Received Steering: ");
        // Serial.println(steering);

        
    } else {
        //Serial.println("Invalid UART message length!");
        //automationThrottle = static_cast<int>(message[0]);
        //automationSteering = static_cast<int>(message[1]);
    }
}

//TX and RX functions
void OnTxDone() {
    //Serial1.println("Sent msg");
}
  
void OnTxTimeout() {
    //Serial1.println("Tx timeout");
}
  
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    signalTime = millis();
    lastRSSI = rssi;
    //Serial.println("Received");
    if (size == sizeof(controllerMsg)) {
        controllerMsg temp;
        memcpy(temp.str, payload, sizeof(controllerMsg));
        if (memcmp(temp.status.secretCode, CONTROLLERMSG_CODE, sizeof(CONTROLLERMSG_CODE)) == 0) {
        // Debug printing on the screen
        //Serial1.printf("Copied %s, fwd: %d, rev: %d, left: %d, right: %d, kill: %d \n",
            //inboundMsg.status.secretCode, inboundMsg.status.isGoingForward, inboundMsg.status.isGoingBackward,
            //inboundMsg.status.isSteeringLeft, inboundMsg.status.isSteeringRight, inboundMsg.status.killswitch);
        memcpy(inboundMsg.str, payload, sizeof(controllerMsg));
        }
    }
}
  
void OnRxTimeout() {
    //Serial.println("Rx timeout");
}