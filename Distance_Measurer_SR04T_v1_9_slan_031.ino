#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
//#include <BMP180.h>
#include "credentials.h" // The Things Network OTAA credentials




//#define trigPin GPIO5
//#define echoPin GPIO3
uint16_t  mmDistance;
float     time;

#define trigPin GPIO1
#define echoPin GPIO2

float pulse_width;

bool sensorDebug = true;

uint16_t getDistance()
{
  float distance;
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  delay(2000);// Wait 2 secs before next ranging
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(35);
  digitalWrite(trigPin,LOW);
  time=pulseIn(echoPin,HIGH);
  distance=time*2.54/147;
  //unsigned char *puc;
  //puc = (unsigned char *)(&distance);
  
  return (uint16_t)(distance * 10.0);
}

// https://github.com/HelTecAutomation/ASR650x-Arduino/blob/master/libraries/Sensor_ThirdParty/examples/MB1040/MB1040.ino
static void prepareTxFrame( uint8_t port )
{
  //if (input.fPort == 11) {
  //  data.battery = ((input.bytes[0] << 8) + input.bytes[1]) ; // in mV
  //  data.distance = (input.bytes[2] << 8) + input.bytes[3]; // in mm
  //}
  
  appDataSize = 4;

  uint16_t batteryVoltage = getBatteryVoltage();
  appData[0] = (uint8_t)(batteryVoltage>>8);
  appData[1] = (uint8_t)(batteryVoltage & 0xFF);

  bool badReading = true;
  uint16_t lastReading = 0;
  
  while (badReading) {
    for (int i=0; i<10; i++) {
      mmDistance = getDistance(); // convert from float tpo unsigned 16-bit int for transmission
      if (sensorDebug) printf( "i = %d, mmDistance = %d\n", i, mmDistance);
    }
    int diff = abs(mmDistance - lastReading);
    if (sensorDebug) printf( "diff = %d, lastReading = %d, mmDistance = %d\n", diff, lastReading, mmDistance);
    lastReading = mmDistance;
    if ((mmDistance > 0) && (diff < 5)) {
      badReading = false;
    }
  }
  appData[2] = (uint8_t)(mmDistance >> 8);
  appData[3] = (uint8_t)(mmDistance & 0xFF);
  
  Serial.printf("battery: %d mV, distance: %d mm\n", batteryVoltage, mmDistance );
}





/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */


/* ABP para - not used */
uint8_t nwkSKey[] = {  };
uint8_t appSKey[] = {  };

uint32_t devAddr =  ( uint32_t )0; // x260B54C4;  // Not sure if used!

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = (15 * 60 * 1000);

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 11;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;








// Blast! Sensor needs 5v (VIN) not 3.3V (VDD) so external power pack needed!

#define SKETCH_NAME   "Distance_Measurer_SR04T_v1_9_slan_031.ino"
#define CREATED       "Mon 9th Sept 2024 16:45"
#define BOARD         "CubeCell Dev-Board Plus (HTCC-AB02)" // https://heltec.org/project/htcc-ab02/


// v1.6 kept sending join messages and no up
// v1.4 failed field trial as kept returning 0 mm


bool debug = true;



void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
	Serial.begin(115200);
  Serial.printf("\n\nSketch:  %s\n", SKETCH_NAME);
  Serial.printf("Created: %s\n", CREATED);
  Serial.printf("Board:   %s\n", BOARD);
  Serial.printf("Region:  REGION_EU868\n");
  Serial.printf("Info:    https://heltec.org/project/htcc-ab02/\n\n");
#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}



void loop()
{
 
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
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
