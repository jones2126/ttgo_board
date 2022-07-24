/*
   RadioLib SX127x, (specifically SX1276) Receive Example LoRa OLED TTGO V1 board

   This example listens for LoRa transmissions using SX127x Lora modules.
   To successfully receive data, the following settings have to be the same
   on both transmitter and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word
    - preamble length

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages: https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void InitLoRa();

/*
 * Begin method:
    Carrier frequency: 915.0 MHz
    Bit rate: 4.8 kbps
    Frequency deviation: 5.0 kHz (single-sideband)
    Receiver bandwidth: 156.2 kHz
    Output power: 10 dBm
    Preamble length: 16 bits
    TCXO reference voltage: 1.6 V (SX126x module with TCXO)
    LDO regulator mode: disabled (SX126x module with DC-DC power supply)

#define SS 18 // GPIO18 -- SX1278's CS
#define DI0 26 // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define RST 14 // GPIO14 -- SX1278's RESET

#define SCK 5 // GPIO5 -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define DI1 33 // GPIO33

 */

//float FREQUENCY = 433.5;  // MHz - EU 433.5; US 915.0
float FREQUENCY = 915.0;  // MHz - EU 433.5; US 915.0
float F_OFFSET = 1250 / 1e6;  // Hz - optional if you want to offset the frequency
int8_t POWER = 10;  // 2 - 20dBm
uint8_t SPREADING_FACTOR = 8;  // 6 - 12; higher is slower
float BANDWIDTH = 125;  // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t CODING_RATE = 7;  // 5 - 8; high data rate / low range -> low data rate / high range

SX1276 radio = new Module(18, 26, 14, 33);  // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);


void setup() {
  startSerial();
  InitLoRa();
}

void loop() {
  Serial.print(F("[SX1278] Waiting for incoming transmission ... "));

  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  String str;
  int state = radio.receive(str);

  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("packet successfully received!"));

    // print the data of the packet
    Serial.print(F("[SX1278] Data:\t\t\t"));
    Serial.println(str);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1278] RSSI:\t\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1278] SNR:\t\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    // of the last received packet
    Serial.print(F("[SX1278] Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
          // timeout occurred while waiting for a packet
          Serial.println(F("nothing received, waiting!"));
          } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                // packet was received, but is malformed
                Serial.println(F("nothing received, no timeout, but CRC error!"));
                } else {
                      // some other error occurred
                      Serial.print(F("nothing received, no timeout, printing failed code "));
                      Serial.println(state);
                      }

/*
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true);
    }
*/
  delay(1000);
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: radio_control_tractor_v1");
}
void InitLoRa(){

// initialize SX1276 with default settings
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {  
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  if (radio.setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
    }
  Serial.println("Selected frequency is: "); Serial.println(FREQUENCY);
/*
  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
    }
  Serial.print("Selected bandwidth is: "); Serial.println(BANDWIDTH);

  // set spreading factor to 7
  if (radio.setSpreadingFactor(7) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
    }
  Serial.println("Selected spreading factor is 7");

  // set coding rate to 5
  if (radio.setCodingRate(5) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
    }
  Serial.println("Selected coding rate is 5");
  // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved for LoRaWAN networks and should not be used
  if (radio.setSyncWord(0x12) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    while (true);
    }
  Serial.println("Selected sync word is 0x12");

  Serial.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(10, true);
*/
  delay(10000); 
}