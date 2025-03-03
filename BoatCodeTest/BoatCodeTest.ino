#include <Wire.h>           
#include "GPS_Air530Z.h"
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

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
#define BUFFER_SIZE 30 // Define the payload size here

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
      bool isSteeringLeft;
      bool isSteeringRight;
      bool killswitch;
      //bool automation;
    } status;
    uint8_t str[13];
  } inboundMsg;
  
// union boatMsg {
//     struct {
//       uint8_t secretCode[8];
//       int16_t lastRSSI;
//       int16_t steeringPosition;
//       float latitude;
//       float longitude;
//       float speed;
//     } status;
//     uint8_t str[24];
// };

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
uint32_t signalTime = 0;
int16_t lastRSSI = 0;

// UART Communication Variables
String receivedData = "";  // Buffer to store UART messages
bool automationMode = false; // Flag for automation mode

// Pin Definitions
#define THROTTLE_PIN GPIO5
#define STEERING_LEFT_PIN GPIO7
#define STEERING_RIGHT_PIN GPIO6
#define MOTOR_CONTROLLER_ENABLE GPIO4

// Controller and Automation Steering and Throttle Values
int controllerThrottle = 0;
int controllerSteering = 50; // 50 is centered
int automationThrottle = 0;
int automationSteering = 0;

// Legacy throttle and throttle state variables
int throttle = 0;
int throttleState = 0;

void setup() {
    Serial1.begin(115200); // Initialize UART for communication with Raspberry Pi

    // LoRa Setup in controller this is a separate function so this might not be necessary?
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    //RadioEvents.TxDone = OnTxDone;
    //RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                        0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
                    
    GPS.begin();

    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(STEERING_LEFT_PIN, OUTPUT);
    pinMode(STEERING_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_CONTROLLER_ENABLE, OUTPUT);
    // Enabling the motor controller?
    digitalWrite(GPIO4, HIGH);
}

void loop() {
    // Read UART messages from Raspberry Pi
    // If this doesnt work then switch to using Serial1.read(buffer, size)
    // I am thinking that this Serial1.available is not going to work 
    // Potential solution is use the buffer and make sure that the message is always an exact length
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {  
            processUARTMessage(receivedData);  // Process complete message
            receivedData = ""; // Clear buffer
        } else {
            receivedData += c; // Append character to buffer
        }
    }
    // NOTE: I'm not sure this code is even correct or what we want to do but it is what the previous code did to work
    Radio.Rx(500);
    delay(100);
    Radio.IrqProcess();

    // Perform actions based on automation or remote control
    doActions();

    delay(100);  // Reduce CPU usage?
    // IDK if this is necessary either
    Radio.IrqProcess();
}

void doActions() {
    // Kill switch that is usually here does nothing?
    
    if (automationMode) {
        // Use automation values from Raspberry Pi
        controlBoat(automationThrottle, automationSteering);
    } else {
        //controlBoat(controllerThrottle, controllerSteering);
        // Use remote control values
        if (inboundMsg.status.isGoingForward) {
            if (throttleState != 1) {
              throttleState = 1;
              throttle = 1000;
            } else {
              throttle += 2500;
              //throttle = min(throttle, (3*UINT16_MAX)/4);
              throttle = min(throttle, (19*UINT16_MAX)/20);
            }
            digitalWrite(GPIO5, LOW);
            analogWrite(PWM1, throttle);
        } else if (inboundMsg.status.isGoingBackward){
            if (throttleState != -1) {
              throttleState = -1;
              throttle = 1000;
            } else {
              throttle += 2500;
              //throttle = min(throttle, (3*UINT16_MAX)/4);
              throttle = min(throttle, (19*UINT16_MAX)/20);
            }
            digitalWrite(GPIO5, HIGH);
            analogWrite(PWM1, throttle);
        } else {
            throttleState = 0;
            analogWrite(PWM1, 0);
        }

        if (inboundMsg.status.isSteeringLeft) {
            digitalWrite(STEERING_LEFT_PIN, HIGH);
            digitalWrite(STEERING_RIGHT_PIN, LOW);
        } else if (inboundMsg.status.isSteeringRight) {
            digitalWrite(STEERING_LEFT_PIN, LOW);
            digitalWrite(STEERING_RIGHT_PIN, HIGH);
        } else {
            digitalWrite(STEERING_LEFT_PIN, LOW);
            digitalWrite(STEERING_RIGHT_PIN, LOW);
        }
    }
}

// Function to control the boat given throttle and steering values
//CHANGE THIS WHEN WE HAVE CONFIRMED THE MESSAGES GET SENT CORRECTLY WE DONT WANT DIGITAL LOGIC ON THE CONTROLS
void controlBoat(int throttleValue, int steeringValue) {
    // TODO: Add new logic to put the throttle at a certain percentage out of 100%.
    if (throttleValue == 1) {
        digitalWrite(THROTTLE_PIN, LOW);
        analogWrite(PWM1, 1000);
    } else {
        analogWrite(PWM1, 0);
    }

    // TODO: Add new logic to steer to a given position
    if (steeringValue == 1) {
        digitalWrite(STEERING_LEFT_PIN, HIGH);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    } else if (steeringValue == -1) {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, HIGH);
    } else {
        digitalWrite(STEERING_LEFT_PIN, LOW);
        digitalWrite(STEERING_RIGHT_PIN, LOW);
    }
}

// Function to process incoming UART messages and update automation throttle and steering values
void processUARTMessage(String message) {
    // Look for correctly prefixed message
    if (message.startsWith("MAV") || message.startsWith("CD")) { 

        int tIndex = message.indexOf("T:");
        int sIndex = message.indexOf("S:");
        if (tIndex != -1 && sIndex != -1) {
            automationThrottle = message.substring(tIndex + 2, sIndex).toInt();
            automationSteering = message.substring(sIndex + 2).toInt();
        }
        Serial1.println("Received automation values - Throttle: " + String(automationThrottle) + ", Steering: " + String(automationSteering));
    }
}

// TX and RX functions
// void OnTxDone() {
//     Serial1.println("Sent msg");
// }
  
// void OnTxTimeout() {
//     Serial1.println("Tx timeout");
// }
  
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    signalTime = millis();
    lastRSSI = rssi;
    Serial1.println("Received");
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
    Serial1.println("Rx timeout");
}