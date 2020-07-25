#include "LoRaWan_APP.h"
#include <Region.h>
#include "Arduino.h"
#include "innerWdt.h"
#include "lora_commissioning.h"

// The interrupt pin is attached to D?/GPIO5
#define INT_PIN GPIO5

bool accelWoke = false;

DeviceClass_t  CLASS = LORAWAN_CLASS;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/* LoRaWAN Adaptive Data Rate */
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/* Application port */
#define DEVPORT 2
#define APPPORT 1
uint8_t appPort = 1;

/*the application data transmission duty cycle.  value in [ms].*/
//uint32_t appTxDutyCycle = (24 * 60 * 60 * 1000); // 24h
uint32_t appTxDutyCycle = (20 * 60 * 1000); // 20min

uint16_t rain_total = 0;

uint32_t h_cnt = 0;

void increment_rain_meter() {
  rain_total++;
}


//downlink data handle function
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (mcpsIndication->BufferSize == 1) {
    Serial.println("Check 1 Byte Message");
    if (mcpsIndication->Buffer[0] == 0x01) {
      Serial.println("Reset Total Rain");
      rain_total = 0;
    }
  }
 
}


/* Prepares the payload of the frame */
static bool prepareTxFrame( uint8_t port, uint16_t voltage, uint8_t restart)
{
  int head;
  int offset = 0;

  appPort = port;
  switch (port) {
    case 1: // woke up from interrupt
      Serial.println("Sending data packet");
      appDataSize = 1 + sizeof(voltage) + sizeof(rain_total);
      memcpy(&appData[offset], &voltage, sizeof(voltage));
      offset += sizeof(voltage);
      memcpy(&appData[offset], &rain_total, sizeof(rain_total));
      offset += sizeof(rain_total);
      appData[offset] = restart;
      offset += 1;
      appDataSize = offset;
      break;
    case 2: // daily wake up
      Serial.println("Sending dev status packet");
      memcpy(&appData[offset], &voltage, sizeof(voltage));
      offset += sizeof(voltage);
      memcpy(&appData[offset], &rain_total, sizeof(rain_total));
      offset += sizeof(rain_total);
      appData[offset] = restart;
      offset += 1;
      appDataSize = offset;
      break;
  }
  return true;
}

// for OTAA

extern bool IsLoRaMacNetworkJoined;


void accelWakeup()
{
  accelWoke = true;
}


uint16_t read_batt_voltage() {
  static uint16_t voltage;
  static uint8_t cnt = 99;

  cnt++;
  if (cnt >= 1) {
    cnt = 0;
  } else {
    return voltage;
  }

  pinMode(VBAT_ADC_CTL,OUTPUT);
  digitalWrite(VBAT_ADC_CTL,LOW);
  voltage = analogRead(ADC) * 2;
  digitalWrite(VBAT_ADC_CTL,HIGH);

  return voltage;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Beginn Setup");

  delay(200); // wait for stable
  accelWoke = false;

  boardInitMcu();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  pinMode(INT_PIN, OUTPUT_PULLUP);
  digitalWrite(INT_PIN, HIGH);
  attachInterrupt(INT_PIN, accelWakeup, FALLING);
  Serial.println("Interrupts attached");

  Serial.println("Starting watchdog");
  /* Enable the WDT, autofeed */
  innerWdtEnable(true);
}

void loop()
{
  int voltage;
  static uint8_t restart = 1;

  if (accelWoke) {
    uint32_t now = TimerGetCurrentTime();
    Serial.print(now); Serial.println("accel woke");
  }

  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
        Serial.printf("LoRaWan Class%X test start! \r\n", CLASS + 10);
#if(AT_SUPPORT)
        Enable_AT();
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.init(loraWanClass,loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        Serial.println("INIT done");
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND: // a send is scheduled to occur, usu. daily status
      {
        Serial.println("send");
        voltage =  read_batt_voltage();
        prepareTxFrame( DEVPORT, voltage, restart); // Timer
        restart = 0;
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        Serial.println("cycle");
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle; // + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        //Serial.println("sleep");
        if (accelWoke) {
          increment_rain_meter();
          accelWoke = false;
          //deviceState = DEVICE_STATE_SEND;
          //break;
        }
        LoRaWAN.sleep();

        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
