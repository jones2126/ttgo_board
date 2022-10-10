/*
This is meant to run on a TTGO ESP32 LoRa OLED V1 board to be a hand-held radio control device which interacts with another board
to take primarily the throttle and steering settings and for those settings to be processed by the other, companion board.  There
are other features also configured in this program.  For example there is a physical e-stop switch that if enabled would direct
the companion board to execute an e-stop protocol. 

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/

10/8/22 - 
- Implement timers in main loop; Get all the print statements into one function
- change "float setThrottle(int x){" to the generic bucketing routine
- Test sending throttle and steering settings
    - convert analog steering to -45 to 45
- move the check RSSI function to inside the incoming message handler; Set a timer and flag if the timer expires
- Test sending e-stop


*/

// include the library
#include <RadioLib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FastLED.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void initLEDs();
void InitLoRa();
void startOLED();
void startBME();
void getControlReadings();
void getWeatherReadings();
void handleIncomingMsg();
void sendOutgoingMsg();
void displayOLED();
void displayLEDstatus();
int classifyRange(int a[], int);
void print_Info_messages();

// radio related
float FREQUENCY = 915.0;        // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;          // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;  // 6 - 12; higher is slower; started with 7, 8 (barely successful)
uint8_t CODING_RATE = 7;        // 5 - 8; high data rate / low range -> low data rate / high range; the example started with 5
byte SYNC_WORD = 0x12;          // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
int8_t POWER = 15;              // 2 - 20dBm
float F_OFFSET = 1250 / 1e6;    // Hz - optional if you want to offset the frequency
float RSSI = 0;
SX1276 radio = new Module(18, 26, 14, 33);  // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);

///////////////////////Inputs/outputs///////////////////////
//Sensor Pin definitions
#define POT_X 36
#define POT_Y 37
#define voltage_pin 34
int led = 2;

//OLED definitions
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//BME280 definition
#define BME280_SDA 13
#define BME280_SCL 21
#define SEALEVELPRESSURE_HPA (1013.25)
float temperature = 0;
float TempF = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;

// LED bar status related
#define NUM_LEDS 4
#define LED_PIN 12
#define BRIGHTNESS 30  // 0 off, 255 highest

// LoRa and RSSI 
int return_test = 0;
int RSSI_test = 9;
int sentStatus = 0;

float steering_val = 0;
float steering_val_ROS = 0;
float throttle_val = 0;
char *throttle_val_ROS;
//char throttle_val_ROS;
int switch_mode;
int voltage_val = 0;


// used classifying results
#define arraySize 10 // size of array a
int SteeeringPts[arraySize] = {0, 130, 298, 451, 1233, 2351, 3468, 4094, 4096, 4097}; 
float SteeeringValues[] = {-45, -30, -15, 0, 15, 30, 45, 45, 45, 99};
// although RSSI is presented as a negative, in order to use this array we will pass the ABS of RSSI ref: https://www.studocu.com/row/document/institute-of-space-technology/calculus/why-rssi-is-in-negative/3653793
int RSSIPts[arraySize] = {0, 70, 90, 120, 124, 128, 132, 136, 140, 160}; 
CRGB RSSIPtsValues[arraySize] = {CRGB::Green, CRGB::Green, CRGB::Yellow, CRGB::Yellow, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::White, CRGB::White};
String ledcolors[arraySize] = { "green", "green", "yellow", "yellow","red", "red","red","red","white","white"};

int ThrottlePts[arraySize] = {0, 780, 1490, 2480, 3275, 4000, 4001, 4002, 4096, 4097}; 
//char ThrottleValues[arraySize][3] = {"-2", "-1", "0", "1", "2", "3", "3", "3", "3", "99"};
int ThrottleValues[arraySize] = {-2, -1, 0, 1, 2, 3, 3, 3, 3, 99};

///////////////////////////////////////////////////////////

/////////////////////Loop Timing variables///////////////////////
const long readingInterval = 500;
const long weatherInterval = 5000;
const long transmitInterval = 333;
const long OLEDInterval = 500;
const long infoInterval = 3000;  // 100 = 1/10 of a second (i.e. 10 Hz) 3000 = 3 seconds
const long radioSignalInterval = 200;  // 100 = 1/10 of a second (i.e. 10 Hz) 3000 = 3 seconds

//const long steerInterval = 50;  // 100 10 HZ, 50 20Hz, 20 = 50 Hz
//const long safetyInterval = 1000; 
//unsigned long prev_safety_time = 0; 
//unsigned long prev_time_steer = 0;


unsigned long prev_time_reading = 0;
unsigned long prev_time_weather = 0;
unsigned long prev_time_xmit = 0;
unsigned long prev_time_OLED = 0;
unsigned long prev_time_printinfo = 0;
unsigned long prev_time_radio_signal = 0;
////////////////////////////////////////////////////////////////




/////////////////////// data structures ///////////////////////
struct RadioControlStruct{
  float steering_val;
  float throttle_val;
  float press_norm ; 
  float humidity;
  float TempF;
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

///////////////////////////////////////////////////////////

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;
CRGB leds[NUM_LEDS];



// int sensor_value = -99;
void setup(){
  pinMode(led, OUTPUT);
  startSerial();
  initLEDs();
  InitLoRa();
  startOLED();
  startBME();
}
void loop(){
  unsigned long currentMillis = millis();
  handleIncomingMsg();
  if ((currentMillis - prev_time_reading)   >= readingInterval)   {getControlReadings();}
  if ((currentMillis - prev_time_weather)    >= weatherInterval)  {getWeatherReadings();}
  if ((currentMillis - prev_time_xmit)       >= transmitInterval) {sendOutgoingMsg();}
  if ((currentMillis - prev_time_OLED)       >= OLEDInterval)     {displayOLED();}
  if ((currentMillis - prev_time_printinfo)  >= infoInterval)     {print_Info_messages();}     
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: radio_control_v2.1");
}
void initLEDs(){
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);  
    //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show(); 
}
void InitLoRa(){
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
  delay(2000); 
}
void getControlReadings(){
  throttle_val = analogRead(POT_X);
  return_test = classifyRange(ThrottlePts, throttle_val); 
  RadioControlData.throttle_val = ThrottleValues[return_test];
  steering_val = analogRead(POT_Y);
  return_test = classifyRange(SteeeringPts, steering_val); 
  RadioControlData.steering_val = SteeeringValues[return_test];
  //RadioControlData.steering_val = steering_val_ROS; 
  voltage_val = analogRead(voltage_pin);  
}
void getWeatherReadings(){
    //light_val = analogRead(light_sensor);  // currently do not have one installed
    temperature = bme.readTemperature();
    TempF = (temperature*1.8)+32; // Convert temperature to Fahrenheit
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // set values for transmitting data
    RadioControlData.press_norm=pressure; 
    RadioControlData.humidity=humidity; 
    RadioControlData.TempF=TempF;
}
void handleIncomingMsg(){
  // tractor receive statement
  //int state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);
    int state = radio.receive(tx_TractorData_buf, TractorData_message_len);
    //Serial.print(F("state (")); Serial.print(state); Serial.println(F(")"));
    if (state == RADIOLIB_ERR_NONE) {    // packet was successfully received
      prev_time_radio_signal = millis();  
      RSSI = abs(radio.getRSSI());
      displayLEDstatus();      
      memcpy(&TractorData, tx_TractorData_buf, TractorData_message_len);
      //TractorData.counter++;
      digitalWrite(led, HIGH);
      } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // timeout occurred while waiting for a packet
            Serial.print(F("waiting..."));
            RSSI = 999;
            displayLEDstatus();            
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                  // packet was received, but is malformed
                  Serial.println(F("nothing received, no timeout, but CRC error!"));
                  RSSI = 999;
                  displayLEDstatus();                   
                  } else {
                        // some other error occurred
                        Serial.print(F("nothing received, no timeout, printing failed code "));
                        Serial.println(state);
                        }
}
void sendOutgoingMsg(){
    digitalWrite(led, HIGH);
    //Serial.print(F("[SX1278] Transmitting steering, throttle, pressure, etc. ... "));
    memcpy(tx_RadioControlData_buf, &RadioControlData, RadioControlData_message_len);
    int state = radio.transmit(tx_RadioControlData_buf, RadioControlData_message_len);
    //Serial.println("Sent a reply");
    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        sentStatus = 1;
        
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
  display.setCursor(0,17);  display.print("RC Volt:");  display.setCursor(58,17); display.print(voltage_val);  
  //display.setCursor(0,27);  display.print("RSSI:");     display.setCursor(58,27); display.print(radio.getRSSI());
  display.setCursor(0,27);  display.print("RSSI:");     display.setCursor(58,27); display.print(RSSI);
  display.setCursor(0,37);  display.print("Throttle:"); display.setCursor(58,37); display.print(throttle_val_ROS);
  display.setCursor(0,47);  display.print("Steering:"); display.setCursor(58,47); display.print(steering_val_ROS);
  //display.setCursor(0,57);  display.print("Mode SW:");  display.setCursor(58,57); display.print(switch_mode);
  display.setCursor(0,57);  display.print("T cntr:");  display.setCursor(58,57); display.print(TractorData.counter);   
  display.display();
  //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
}
void startBME(){
  Serial.println("In startBME function");
  I2Cone.begin(BME280_SDA, BME280_SCL, 100000ul); 
  bool status1 = bme.begin(0x77, &I2Cone);  //default 0x77; jump SDO to GND to change address to 0x76
  if (!status1) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("ID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(1000);    
  }
}
void displayLEDstatus(){
    RSSI_test = classifyRange(RSSIPts, RSSI); 
    leds[0] = RSSIPtsValues[RSSI_test];
    leds[1] = CRGB::Blue;
    leds[2] = CRGB::Blue;
    leds[3] = CRGB::Blue;
    FastLED.show();
}
int classifyRange(int a[], int x){ 
    int classification_ptr;  
    if (x >=a[0] && x <=(a[1]-1)){
        classification_ptr = 0;
        } else if (x >=a[1] && x <=(a[2]-1)){
            classification_ptr = 1;
            } else if (x >=a[2] && x <=(a[3]-1)){
                  classification_ptr = 2;
                  } else if (x >=a[3] && x <=(a[4]-1)){
                        classification_ptr = 3;
                        } else if (x >=a[4] && x <=(a[5]-1)){
                              classification_ptr = 4;
                              } else if (x >=a[5] && x <=(a[6]-1)){
                                    classification_ptr = 5;
                                    } else if (x >=a[6] && x <=(a[7]-1)){
                                          classification_ptr = 6;
                                          } else if (x >=a[7] && x <=(a[8]-1)){
                                                classification_ptr = 7;
                                                } else if (x >=a[8] && x <=a[9]){
                                                      classification_ptr = 8;
                                                      }
                                                      else {
                                                        classification_ptr = 9;
                                                      }
    return classification_ptr;        
}
void print_Info_messages(){
    printf("\n");  
    printf("\n");   
    Serial.print(F("last tractor data: "));
    //Serial.print(" speed: "); Serial.print(TractorData.speed);
    //Serial.print(", heading: "); Serial.print(TractorData.heading);
    //Serial.print(", voltage: "); Serial.print(TractorData.voltage);
    Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter); 
    Serial.print(F(" RC data sent: "));
    //Serial.print(F("Datarate: "));  Serial.print(radio.getDataRate());  Serial.print(F(" bps "));
    //Serial.print(", RSSI: "); Serial.print(RSSI);  //radio.getRSSI()
    Serial.print(", RSSI: "); Serial.print(radio.getRSSI()); 
    //Serial.print(F(", RSSI color: "));  Serial.print(ledcolors[RSSI_test]);    
    //Serial.print(F(", throttle: "));  Serial.print(RadioControlData.throttle_val);
    //Serial.print(F(", POT X: "));  Serial.print(throttle_val);
    //Serial.print(F(", steering: "));  Serial.print(RadioControlData.steering_val);     
    //Serial.print(F(", POT Y: "));  Serial.print(steering_val);
    // Serial.print(F(", steering_val_ROS: "));  Serial.print(steering_val_ROS);
    //Serial.print("Temp *C = "); Serial.print(temperature);
    //Serial.print(", Temp *F = "); Serial.print(TempF);  // Convert temperature to Fahrenheit
    //Serial.print("Pressure (hPa) = "); Serial.print(pressure);
    //Serial.print("Approx. Altitude (m) = "); Serial.print(altitude);
    //Serial.print(", Humidity = "); Serial.print(humidity); Serial.println(" % ");
    Serial.print(", RadioControlData.counter: "); Serial.print(RadioControlData.counter);

    //printf("\n");   



    printf("\n");
    printf("\n");     
    //Serial.print(F("[SX1278] RSSI:\t\t\t"));  Serial.print(RSSI);  Serial.println(F(" dBm"));
    // print the SNR (Signal-to-Noise Ratio) of the last received packet
    //Serial.print(F("[SX1278] SNR:\t\t\t"));  Serial.print(radio.getSNR());  Serial.println(F(" dB"));
    // print frequency error of the last received packet
    //Serial.print(F("[SX1278] Frequency error:\t"));  Serial.print(radio.getFrequencyError());  Serial.println(F(" Hz"));
    // print the data from the received message   
    //Serial.println();
    prev_time_printinfo = millis();  // reset the timer
}

/*
check tractor.counter
- if the counter is not in line with the expected result then you are not receiving good data
- assume the tractor is sending data a 1 packet per second (i.e. 1:1000 milliseconds)
- ( current_tractor_counter - last_tractor_counter)/millis = age of counter
- (1003 - 999) / (current_millis - last_received_millis)
- 4 / 10 (seconds)
*/