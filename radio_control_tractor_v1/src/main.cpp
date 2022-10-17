/*
This is meant to run on a TTGO ESP32 LoRa OLED V1 board to control a tractor.  This device interacts with another board
to primarily receive the throttle and steering settings and and to apply those settings to the steering and throttle
controller connected to this TTGO board.

There are other features also configured in this program.  For example if this program receives an e-stop notification
this board should trigger a relay to stops the motor on the this tractor. 

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/

10/8/22 - 
- Test receiving throttle and steering settings
- Test receiving the e-stop notification
- Test getting heading and speed from ROS and transmitting to the radio control device
*/

// include the library
#include <RadioLib.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void InitLoRa();
void getTractorData();
void sendOutgoingMsg();
void handleIncomingMsg();
void print_Info_messages();

// radio related
float FREQUENCY = 915.0;  // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;  // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;  // 6 - 12; higher is slower; started at 7
uint8_t CODING_RATE = 7;  // 5 - 8; high data rate / low range -> low data rate / high range
byte SYNC_WORD = 0x12; // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
float F_OFFSET = 1250 / 1e6;  // Hz - optional if you want to offset the frequency
int8_t POWER = 15;  // 2 - 20dBm
SX1276 radio = new Module(18, 26, 14, 33);  // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);

int led = 2;

struct RadioControlStruct{
  float steering_val;
  float  throttle_val;
  float press_norm ; 
  float press_hg;
  float temp;
  unsigned long counter;
  }RadioControlData;

// tractorData 4 bytes/float = 5X4=20+ 4 bytes/unsigned long = 4 so 24 bytes; 3x/second or 72 bps

uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

struct TractorDataStruct{
  float speed;
  float heading; 
  float voltage;
  unsigned long counter;
  }TractorData;

// tractorData 4 bytes/float = 3X4=12 + 4 bytes/unsigned long = 4 so 16 bytes; 2x/second or 32 bps

uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

/////////////////////Loop Timing variables///////////////////////
const long readingInterval = 100;
const long transmitInterval = 500;
const long infoInterval = 2000;
unsigned long prev_time_reading = 0;
unsigned long prev_time_xmit = 0;
unsigned long prev_time_printinfo = 0;
/////////////////////////////////////////////////////////////////

void setup() {
  startSerial();
  InitLoRa();
}
void loop() {
    unsigned long currentMillis = millis();
    handleIncomingMsg();
    if ((currentMillis - prev_time_reading) >= readingInterval)   {getTractorData();}
    if ((currentMillis - prev_time_xmit)    >= transmitInterval)  {sendOutgoingMsg();}
    if ((currentMillis - prev_time_printinfo)  >= infoInterval)   {print_Info_messages();}       
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
  Serial.print("Selected frequency is: "); Serial.println(FREQUENCY);

  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
    }
  Serial.print("Selected bandwidth is: "); Serial.println(BANDWIDTH);

  if (radio.setSpreadingFactor(SPREADING_FACTOR) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
    }
  Serial.print("Selected spreading factor is: "); Serial.println(SPREADING_FACTOR);

  if (radio.setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
    }
  Serial.print("Selected coding rate is: ");  Serial.println(CODING_RATE);

  if (radio.setSyncWord(SYNC_WORD) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    while (true);
    }
  Serial.print("Selected sync word is: "); Serial.println(SYNC_WORD, HEX);

  if (radio.setOutputPower(POWER, true) == RADIOLIB_ERR_NONE) {
      Serial.print("Selected Power set at: ");  Serial.println(POWER);
  } else {
      Serial.println(F("Unable to set power level!"));
      Serial.print(F("failed, code "));
      Serial.println(state);      
      while (true);
      }

  delay(10000); 
}
void getTractorData(){
  TractorData.speed = 255;
  TractorData.heading = 359.9;
  TractorData.voltage = 12.8;
  prev_time_reading = millis();
}
void sendOutgoingMsg(){
    digitalWrite(led, HIGH);
    Serial.print(F(", xmit"));
    // if estop = 1, set flag to 1 otherwise set flag to 0;  estop is on pin 25

    memcpy(tx_TractorData_buf, &TractorData, TractorData_message_len);
    int state = radio.transmit(tx_TractorData_buf, TractorData_message_len);
    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted

        } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
              // the supplied packet was longer than 256 bytes
              Serial.println(F("too long!"));
              } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
                    // timeout occurred while transmitting packet
                    Serial.println(F("timeout!"));
                    } else {
                        // some other error occurred
                        Serial.print(F("failed, code "));
                        Serial.println(state);
                        }
    TractorData.counter++;
    digitalWrite(led, LOW);
}
void handleIncomingMsg(){
    int state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);
    //Serial.print(F("state (")); Serial.print(state); Serial.println(F(")"));
    if (state == RADIOLIB_ERR_NONE) {        // packet was successfully received
      memcpy(&RadioControlData, tx_RadioControlData_buf, RadioControlData_message_len);
      digitalWrite(led, HIGH);
      } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {   // timeout occurred while waiting for a packet
            Serial.print(F("waiting..."));
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {  // packet was received, but is malformed
                  Serial.println(F("nothing received, no timeout, but CRC error!"));
                  } else {   // some other error occurred
                        Serial.print(F("nothing received, no timeout, printing failed code "));
                        Serial.println(state);
                        }
}
void print_Info_messages(){
    printf("\n"); 
    //Serial.println(F(" success!, sent the following data..."));
    //Serial.print("speed: "); Serial.print(TractorData.speed);
    //Serial.print("heading: "); Serial.print(TractorData.heading);
    //Serial.print("voltage: "); Serial.print(TractorData.voltage);
    Serial.print("Tractr ctr: "); Serial.print(TractorData.counter);
    Serial.print(", RC ctr: "); Serial.print(RadioControlData.counter);    
    // print measured data rate
    Serial.print(F(", BPS "));
    Serial.print(radio.getDataRate());
    //Serial.print(F(" bps"));
    //Serial.println(F("packet received!"));
    // print the RSSI (Received Signal Strength Indicator) of the last received packet
    Serial.print(F(", RSSI: "));  Serial.print(radio.getRSSI());  
    //Serial.print(F(", SNR: "));  Serial.print(radio.getSNR());  
    //Serial.print(F(", dB"));
    //Serial.print(F(", Freq error: ")); Serial.print(radio.getFrequencyError());  
    //Serial.print(F(", Hz"));
    Serial.print(", steering: "); Serial.print(RadioControlData.steering_val);
    Serial.print(", throttle: "); Serial.print(RadioControlData.throttle_val);
    //Serial.print(", press_norm: "); Serial.print(RadioControlData.press_norm);
    //Serial.print(", press_hg: "); Serial.print(RadioControlData.press_hg);
    //Serial.print(", temp: "); Serial.print(RadioControlData.temp);
    printf("\n"); 
}