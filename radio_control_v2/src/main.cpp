/*
   RadioLib SX127x, (specifically SX1276) Transmit Example Using a LoRa OLED TTGO V1 board

   This example transmits packets using SX1278 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

#define SS 18 // GPIO18 -- SX1278's CS
#define DI0 26 // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define RST 14 // GPIO14 -- SX1278's RESET

#define SCK 5 // GPIO5 -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define DI1 33 // GPIO33

*/

// include the library
#include <RadioLib.h>
#include <Adafruit_SSD1306.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void InitLoRa();
void getControlReadings();
void getWeatherReadings();
void handleIncomingMsg();
void sendOutgoingMsg();
void startOLED();
void displayOLED();

float FREQUENCY = 915.0;        // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;          // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;  // 6 - 12; higher is slower; started with 7, 8 (barely successful)
uint8_t CODING_RATE = 7;        // 5 - 8; high data rate / low range -> low data rate / high range; the example started with 5
byte SYNC_WORD = 0x12;          // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
int8_t POWER = 10;              // 2 - 20dBm
float F_OFFSET = 1250 / 1e6;    // Hz - optional if you want to offset the frequency


SX1276 radio = new Module(18, 26, 14, 33);  // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);

//Sensor Pin definitions
#define POT_X 36
#define POT_Y 37
int steering_val = 0;
int throttle_val = 0;
int switch_mode;

int led = 2;

struct RadioControlStruct{
  int steering_val;
  int throttle_val;
  float press_norm ; 
  float press_hg;
  float temp;
  unsigned long counter;
  }RadioControlData;

uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

struct TractorDataStruct{
  float speed;
  float heading; 
  float voltage;
  unsigned long counter;
  }TractorData;

uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

//OLED definitions
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

void setup() {
  pinMode(led, OUTPUT);
  startSerial();
  InitLoRa();
  startOLED();
}

void loop() {
  getControlReadings();
  getWeatherReadings();
  sendOutgoingMsg();
  handleIncomingMsg();
  displayOLED();
  delay(333); // wait before transmitting again
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: radio_control_slave_v1");
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
void getControlReadings(){
  //steering_val = analogRead(POT_X);
  //throttle_val = analogRead(POT_Y);
  RadioControlData.steering_val = 255;
  RadioControlData.throttle_val = 4096;
}
void getWeatherReadings(){
  //light_val = analogRead(light_sensor);  
 // temperature = bme.readTemperature();
 // TempF = (temperature*1.8)+32;
 // humidity = bme.readHumidity();
 // pressure = bme.readPressure() / 100.0F;

// set initial values for tranmitting data

  RadioControlData.press_norm=1000.11; 
  RadioControlData.press_hg=0.59; 
  RadioControlData.temp=22.394;
}
void handleIncomingMsg(){
    int state = radio.receive(tx_TractorData_buf, TractorData_message_len);
    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("packet successfully received!"));
      // print the RSSI (Received Signal Strength Indicator) of the last received packet
      Serial.print(F("[SX1278] RSSI:\t\t\t"));  Serial.print(radio.getRSSI());  Serial.println(F(" dBm"));
      // print the SNR (Signal-to-Noise Ratio) of the last received packet
      Serial.print(F("[SX1278] SNR:\t\t\t"));  Serial.print(radio.getSNR());  Serial.println(F(" dB"));
      // print frequency error of the last received packet
      Serial.print(F("[SX1278] Frequency error:\t"));  Serial.print(radio.getFrequencyError());  Serial.println(F(" Hz"));
      // print the data from the received message 
      memcpy(&TractorData, tx_TractorData_buf, TractorData_message_len);
      Serial.print(F("Incoming message:\t\t\t"));
      Serial.print("speed: "); Serial.print(TractorData.speed);
      Serial.print("heading: "); Serial.print(TractorData.heading);
      Serial.print("voltage: "); Serial.print(TractorData.voltage);
      Serial.print("counter: "); Serial.print(TractorData.counter);
      TractorData.counter++;
      printf("\n"); 
      digitalWrite(led, HIGH);
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
}
void sendOutgoingMsg(){
    digitalWrite(led, HIGH);
    Serial.print(F("[SX1278] Transmitting steering, throttle, pressure, etc. ... "));
    memcpy(tx_RadioControlData_buf, &RadioControlData, RadioControlData_message_len);
    int state = radio.transmit(tx_RadioControlData_buf, RadioControlData_message_len);
    //Serial.println("Sent a reply");
    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        //Serial.println(F(" success!, sent the following data..."));
        //Serial.print("steering: "); Serial.print(RadioControlData.steering_val);
        //Serial.print(", throttle: "); Serial.print(RadioControlData.throttle_val);
        //Serial.print(", press_norm: "); Serial.print(RadioControlData.press_norm);
        //Serial.print(", press_hg: "); Serial.print(RadioControlData.press_hg);
        //Serial.print(", temp: "); Serial.print(RadioControlData.temp);
        //Serial.print(", counter: "); Serial.println(RadioControlData.counter);
        // print measured data rate
        Serial.print(F("[SX1278] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));
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
    /*
    Serial.print(F("Outgoing message (hex):\t\t\t"));
    int i=0;
    for (i=0; i<RadioControlData_message_len; i++) {
        printf("%02x ", tx_RadioControlData_buf[i]);
        }
    printf("\n");
    */
   RadioControlData.counter++;
    digitalWrite(led, LOW);
}
void startOLED(){
  Serial.println("In startOLED");
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while(1) delay(1000);   // loop forever and don't continue
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("start OLED");
  Serial.println("startOLED - exit");   
}
void displayOLED(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("Control Readings");
  display.setCursor(0,30);
  display.print("Pot 1:");
  display.setCursor(54,30);
  display.print(throttle_val);
  display.setCursor(0,40);
  display.print("Pot 2:");
  display.setCursor(54,40);
  display.print(steering_val);
  display.setCursor(0,50);
  display.print("Mode SW");
  display.setCursor(66,50);
  display.print(switch_mode);  
  display.display();
}