#define DUET_RX 1   
//edit above line for switch Tx or Rx Mode
//0 = Tx, 1 = Rx 

#if DUET_RX == 1    //do not edit this line

#include <Arduino.h>
#include <SPI.h>
#include <Ra01S.h>

//#define RF_FREQUENCY                                433000000 // Hz  center frequency
//#define RF_FREQUENCY                                866000000 // Hz  center frequency
#define RF_FREQUENCY                                920000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             10        // dBm tx output power
#define LORA_BANDWIDTH                              4         // bandwidth
                                                              // 2: 31.25Khz
                                                              // 3: 62.5Khz
                                                              // 4: 125Khz
                                                              // 5: 250KHZ
                                                              // 6: 500Khz 
#define LORA_SPREADING_FACTOR                       7         // spreading factor [SF5..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_PAYLOADLENGTH                          0         // 0: Variable length packet (explicit header)
                                                              // 1..255  Fixed length packet (implicit header)

//#define USE_EBYTE
                                                             
#ifdef USE_EBYTE
SX126x  lora(5,               //Port-Pin Output: SPI select
             6,               //Port-Pin Output: Reset 
             7,               //Port-Pin Input:  Busy
             8,               //Port-Pin Output: TXEN
             9                //Port-Pin Output: RXEN
             );

#else
SX126x  lora(10,               //Port-Pin Output: SPI select
             14,               //Port-Pin Output: Reset 
             21                //Port-Pin Input:  Busy
             );
#endif // USE_EBYTE

void setup() 
{
  delay(1000);
  Serial.begin(115200);
  SPI.begin(12,11,13,10); //sck,miso,mosi,ssl

  //lora.DebugPrint(true);

#ifdef USE_EBYTE
  Serial.println("Enable TCXO");
  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER,           //tx power in dBm
                           3.3,                       //use TCXO
                           true);                     //use TCXO
  if (ret != ERR_NONE) while(1) {delay(1);}
#else
  Serial.println("Disable TCXO");
  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER,           //tx power in dBm
                           0.0,                       //TCXO voltage
                           true);                     //use useRegulatorLDO
  if (ret != ERR_NONE) {
    Serial.println("#Error !!, halt.");
    while(1) {delay(1);}
  }
#endif // USE_EBYTE

  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                  LORA_BANDWIDTH, 
                  LORA_CODINGRATE, 
                  LORA_PREAMBLE_LENGTH, 
                  LORA_PAYLOADLENGTH, 
                  true,               //crcOn  
                  false);             //invertIrq

}

void loop() 
{
  uint8_t rxData[255];
  uint8_t rxLen = lora.Receive(rxData, 255);
  if ( rxLen > 0 )
  { 
    Serial.print("Receive rxLen:");
    Serial.println(rxLen);
    for(int i=0;i< rxLen;i++) {
      Serial.print(rxData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    for(int i=0;i< rxLen;i++) {
      if (rxData[i] > 0x19 && rxData[i] < 0x7F) {
        char myChar = rxData[i];
        Serial.print(myChar);
      } else {
        Serial.print("?");
      }
    }
    Serial.println();

    int8_t rssi, snr;
    lora.GetPacketStatus(&rssi, &snr);
    Serial.print("rssi: ");
    Serial.print(rssi, DEC);
    Serial.println(" dBm");
    Serial.print("snr: ");
    Serial.print(snr, DEC);
    Serial.println(" dB");
    Serial.println();
  }
  delay(1);
}

#else

#include <Arduino.h>
#include <SPI.h>
#include <Ra01S.h>

//#define RF_FREQUENCY                                433000000 // Hz  center frequency
//#define RF_FREQUENCY                                866000000 // Hz  center frequency
#define RF_FREQUENCY                                920000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             10        // dBm tx output power
#define LORA_BANDWIDTH                              4         // bandwidth
                                                              // 2: 31.25Khz
                                                              // 3: 62.5Khz
                                                              // 4: 125Khz
                                                              // 5: 250KHZ
                                                              // 6: 500Khz                                                               
#define LORA_SPREADING_FACTOR                       7         // spreading factor [SF5..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_PAYLOADLENGTH                          0         // 0: Variable length packet (explicit header)
                                                              // 1..255  Fixed length packet (implicit header)

//#define USE_EBYTE

#ifdef USE_EBYTE
SX126x  lora(5,               //Port-Pin Output: SPI select
             6,               //Port-Pin Output: Reset 
             7,               //Port-Pin Input:  Busy
             8,               //Port-Pin Output: TXEN
             9                //Port-Pin Output: RXEN
             );

#else
SX126x  lora(10,               //Port-Pin Output: SPI select
             14,               //Port-Pin Output: Reset 
             21                //Port-Pin Input:  Busy
             );
#endif // USE_EBYTE

void setup() 
{
  delay(1000);
  Serial.begin(115200);
  //SPI.setFrequency(2000000);
  SPI.begin(12,11,13,10); //sck,miso,mosi,ssl

  //lora.DebugPrint(true);

#ifdef USE_EBYTE
  Serial.println("Enable TCXO");
  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER,           //tx power in dBm
                           3.3,                       //use TCXO
                           true);                     //use TCXO
  if (ret != ERR_NONE) while(1) {delay(1);}
#else
  Serial.println("Disable TCXO");
  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER,           //tx power in dBm
                           0.0,                       //TCXO voltage
                           true);                     //use useRegulatorLDO
  if (ret != ERR_NONE) {
    Serial.println("#Error !!, halt.");
    while(1) {delay(1);}
  }
#endif // USE_EBYTE

  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                  LORA_BANDWIDTH, 
                  LORA_CODINGRATE, 
                  LORA_PREAMBLE_LENGTH, 
                  LORA_PAYLOADLENGTH, 
                  true,               //crcOn  
                  false);             //invertIrq

}

void loop() 
{
  uint8_t txData[255];
  sprintf((char *)txData, "Hello World %lu", millis());
  uint8_t len = strlen((char *)txData);

  // Wait for transmission to complete
  if (lora.Send(txData, len, SX126x_TXMODE_SYNC)) {
    Serial.println("Send success");
  } else {
    Serial.println("Send fail");
  }

  // Do not wait for the transmission to be completed
  //lora.Send(txData, len, SX126x_TXMODE_ASYNC );

  delay(1000);
}

#endif
