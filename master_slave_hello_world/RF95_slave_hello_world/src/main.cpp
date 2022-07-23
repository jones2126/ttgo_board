/*
Example sketch showing how to create a simple messaging client (transmitter)
It is designed to work with a "master" (RF95_master_hello_world) that receives the "hello world" message

Based on: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-rfm69hcw-module/using-the-rfm69-radio
But this example started with Feather9x_TX which uses the RH_RF95.h library.  When using the Arduino IDE I used the 
Adafruit RadioHead library.

I'm using a LoRa TTGO Semtech SX1276 board which has an OLED.  If I started to use the OLED I'm assuming the SPI pins would 
get confused.

*/

#include <SPI.h>
#include <RadioHead.h>  // I had to add this when using VSCode/PlaformIO as the IDE.  Not required when using Arduino IDE
#include <RH_RF95.h>

// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void startLoRA();

// LoRa pins for TTGO Semtech SX1276 ref: https://microcontrollerslab.com/ttgo-lora32-sx1276-oled-board-pinout-getting-started-with-arduino-ide/ 
#define RFM95_CS  18   
#define RFM95_RST 14   
#define RFM95_INT 26   
#define RF95_FREQ 915.0
#define RFM95_TxPower 5 

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() {
  startSerial();
  startLoRA();
}

void loop()
{
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }

}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: RF95_slave_hello_world");
}
void startLoRA(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);    // manual reset of LoRa pins
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    delay(5000);
  }
  Serial.println("LoRa radio init OK!");
  while (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    delay(5000);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(RFM95_TxPower, false);  
}