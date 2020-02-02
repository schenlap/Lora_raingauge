#include "LoRaWan_APP.h"
#include <Region.h>
#include "Arduino.h"
#include "lora_commissioning.h"



// The interrupt pin is attached to D4/GPIO1
//#define INT_PIN GPIO1

// The interrupt pin is attached to D?/GPIO5
#define INT_PIN GPIO5

bool accelWoke = false;

DeviceClass_t  CLASS = LORAWAN_CLASS;

/*OTAA or ABP*/
bool OVER_THE_AIR_ACTIVATION = LORAWAN_NETMODE;

/* LoRaWAN Adaptive Data Rate */
bool LORAWAN_ADR_ON = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool KeepNet = LORAWAN_Net_Reserve;

/*LoraWan REGION*/
LoRaMacRegion_t REGION = ACTIVE_REGION;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool IsTxConfirmed = 0; //LORAWAN_UPLINKMODE;

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
uint8_t ConfirmedNbTrials = 8;

/* Application port */
#define DEVPORT 2
#define APPPORT 1
uint8_t AppPort = 1;

/*the application data transmission duty cycle.  value in [ms].*/
//uint32_t APP_TX_DUTYCYCLE = (24 * 60 * 60 * 1000); // 24h
uint32_t APP_TX_DUTYCYCLE = (60 * 60 * 1000); // 1h

uint32_t rain_total = 0;
uint32_t rain_today = 0;

uint32_t h_cnt = 0;


void increment_rain_meter() {
  rain_total++;
  rain_today++;
}


/* Prepares the payload of the frame */
static bool prepareTxFrame( uint8_t port, uint16_t voltage )
{
  int head;
  int offset = 0;
  
  AppPort = port;
  switch (port) {
    case 1: // woke up from interrupt
      Serial.println("Sending data packet");
      AppDataSize = 1 + sizeof(voltage) + sizeof(rain_total);
      memcpy(&AppData[offset], &voltage, sizeof(voltage));
      offset += sizeof(voltage);
      memcpy(&AppData[offset], &rain_total, sizeof(rain_total));
      offset += sizeof(rain_total);
      memcpy(&AppData[offset], rain_today, sizeof(rain_today));
      offset += sizeof(rain_today);
      AppDataSize = offset;
      break;
    case 2: // daily wake up
      Serial.println("Sending dev status packet");
      memcpy(&AppData[offset], &voltage, sizeof(voltage));
      offset += sizeof(voltage);
      memcpy(&AppData[offset], &rain_total, sizeof(rain_total));
      offset += sizeof(rain_total);
      memcpy(&AppData[offset], rain_today, sizeof(rain_today));
      offset += sizeof(rain_today);
      AppDataSize = offset;
      break;
  }
  return true;
}

// for OTAA
extern uint8_t DevEui[];
extern uint8_t AppEui[];
extern uint8_t AppKey[];
// for ABP
extern uint32_t DevAddr;
extern uint8_t NwkSKey[];
extern uint8_t AppSKey[];
extern bool IsLoRaMacNetworkJoined;

void accelWakeup()
{
  accelWoke = true;
}

uint16_t read_batt_voltage() {
  uint16_t voltage;
  pinMode(ADC_CTL,OUTPUT);
  digitalWrite(ADC_CTL,LOW);
  voltage = analogRead(ADC) * 2;
  digitalWrite(ADC_CTL,HIGH);
  Serial.print("ADC_battery: ");
  Serial.print(voltage, DEC);
  Serial.println("");

  return voltage;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Beginn Setup");

  delay(200); // wait for stable
  accelWoke = false;

  // for OTAA
  memcpy(DevEui, myDevEui, sizeof(myDevEui));
  memcpy(AppEui, myAppEui, sizeof(myAppEui));
  memcpy(AppKey, myAppKey, sizeof(myAppKey));
  // for ABP
  memcpy(&DevAddr, &myDevAddr, sizeof(myDevAddr));
  memcpy(NwkSKey, myNwkSKey, sizeof(myNwkSKey));
  memcpy(AppSKey, myAppSKey, sizeof(myAppSKey));
  
  BoardInitMcu();
  
  DeviceState = DEVICE_STATE_INIT;
  LoRaWAN.Ifskipjoin();

  pinMode(INT_PIN, OUTPUT_PULLUP);
  digitalWrite(INT_PIN, HIGH);
  attachInterrupt(INT_PIN, accelWakeup, FALLING);
  Serial.println("Interrupts attached");
}

void loop()
{
  int voltage;

  if (accelWoke) {
    uint32_t now = TimerGetCurrentTime();
    Serial.print(now); Serial.println("accel woke");
  }

  switch ( DeviceState )
  {
    case DEVICE_STATE_INIT:
      {
        Serial.printf("LoRaWan Class%X test start! \r\n", CLASS + 10);
#if(AT_SUPPORT)
        Enable_AT();
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.Init(CLASS, REGION);
        DeviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.Join();
        break;
      }
    case DEVICE_STATE_SEND: // a send is scheduled to occur, usu. daily status
      {
        h_cnt++;
        if (h_cnt >= 24) {
          h_cnt = 0;
          for (int i = RAIN_HISTORY_LENGTH - 1; i >= 1; i++)
            rain_today[i] = rain_today[i - 1];
          rain_today[0] = 0;
        }
        voltage =  read_batt_voltage();
        prepareTxFrame( DEVPORT, voltage); // Timer
        LoRaWAN.Send();
        DeviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.Cycle(TxDutyCycleTime);
        DeviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          increment_rain_meter();
          Serial.println("+");
          voltage = read_batt_voltage();
          if (IsLoRaMacNetworkJoined) {
            if(prepareTxFrame(APPPORT, voltage)) { // ext. interrupt
              LoRaWAN.Send();
            }
          } else {
            Serial.println("not joined, no not send event");
            //if(prepareTxFrame(APPPORT, voltage)) { // ext. interrupt
            //  LoRaWAN.Send();
          }
          accelWoke = false;
        }
        LoRaWAN.Sleep();
        //Serial.println("LoRaWAN.Sleep() finished");
        break;
      }
    default:
      {
        DeviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
