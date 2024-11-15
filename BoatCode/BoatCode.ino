// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>           
#include "GPS_Air530Z.h"
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

// LoRa Configuration Constants
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

Air530ZClass GPS;

// This message will prevent bogus messages from giving incorrect signals to the boat
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


// Declare radio events
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
uint32_t signalTime = 0;
int16_t lastRSSI = 0;

void setup() {
  Serial.begin(115200);

  // LoRa Setup
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

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

  // Steering Left
  pinMode(GPIO7, OUTPUT);
  // Steering Right
  pinMode(GPIO6, OUTPUT);
  // Forward or Reverse
  pinMode(GPIO5, OUTPUT);
  // This is the PWM pin 2 that is not used?
  pinMode(GPIO4, OUTPUT);
  digitalWrite(GPIO4, HIGH);

}

// Initialize throttle variables
int throttle = 0;
int throttleState = 0;

void doActions() {

  if ((millis() - signalTime) > 2000) {
    analogWrite(PWM1, 0);
    digitalWrite(GPIO6, LOW);
    digitalWrite(GPIO7, LOW);
    return;
  }


  if (inboundMsg.status.killswitch) {
    killswitchLock();
  }

  if (inboundMsg.status.isSteeringRight) {
    //Right
    digitalWrite(GPIO6, HIGH);
    digitalWrite(GPIO7, LOW);

  } else if (inboundMsg.status.isSteeringLeft) {
    //Left
    digitalWrite(GPIO6, LOW);
    digitalWrite(GPIO7, HIGH);
  } else {

    // Static
    int steerPos = analogRead(ADC3);
    if (steerPos > 3900) {
      digitalWrite(GPIO6, LOW);
      digitalWrite(GPIO7, HIGH);
    } else if (steerPos < 3700) {
      digitalWrite(GPIO6, HIGH);
      digitalWrite(GPIO7, LOW);      
    } else {
      digitalWrite(GPIO6, LOW);
      digitalWrite(GPIO7, LOW);
    }

    // digitalWrite(GPIO6, LOW);
    // digitalWrite(GPIO7, LOW);

  }

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
}

boatMsg updateStatusVals() {
  boatMsg outboundMsg = {0};
  memcpy(outboundMsg.status.secretCode, BOATMSG_CODE, sizeof(BOATMSG_CODE));

  outboundMsg.status.lastRSSI = lastRSSI;
  outboundMsg.status.steeringPosition = analogRead(ADC3);

  while (GPS.available() > 0) {
    GPS.encode(GPS.read());
  }
  outboundMsg.status.latitude = (float)GPS.location.lat();
  outboundMsg.status.longitude = (float)GPS.location.lng();
  outboundMsg.status.speed = (float)GPS.speed.mph();

  return outboundMsg;
}

void loop() {

  Serial.println(analogRead(ADC3));

  Radio.Rx(500);
  delay(100);

  doActions();
  Radio.Send(updateStatusVals().str, sizeof(boatMsg));

  delay(100);
  Radio.IrqProcess();

}

void killswitchLock() {
  analogWrite(PWM1, 0);
  digitalWrite(GPIO6, LOW);
  digitalWrite(GPIO7, LOW);
  while(true) {
    Serial.println(analogRead(ADC3));
    Radio.Send(updateStatusVals().str, sizeof(boatMsg));
    delay(100);
  }
}

void OnTxDone() {
  Serial.println("Sent msg");
}

void OnTxTimeout() {
  Serial.println("Tx timeout");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  signalTime = millis();
  lastRSSI = rssi;
  Serial.println("Received");
  if (size == sizeof(controllerMsg)) {
    controllerMsg temp;
    memcpy(temp.str, payload, sizeof(controllerMsg));
    if (memcmp(temp.status.secretCode, CONTROLLERMSG_CODE, sizeof(CONTROLLERMSG_CODE)) == 0) {
      Serial.printf("Copied %s, fwd: %d, rev: %d, left: %d, right: %d, kill: %d \n",
        inboundMsg.status.secretCode, inboundMsg.status.isGoingForward, inboundMsg.status.isGoingBackward,
        inboundMsg.status.isSteeringLeft, inboundMsg.status.isSteeringRight, inboundMsg.status.killswitch);
      memcpy(inboundMsg.str, payload, sizeof(controllerMsg));
    }

  }

}

void OnRxTimeout() {
  Serial.println("Rx timeout");
}


