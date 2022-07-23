/*
Example sketch showing how to create a simple messaging master (aka receiver)
It is designed to work with "slave" (RF95_slave_hello_world) that is transmitting a "hello world" message


Based on: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-rfm69hcw-module/using-the-rfm69-radio
This example started with Feather9x_RX which uses the RH_RF95.h library.  When using the Arduino IDE I used the 
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

#define RFM95_CS  18   
#define RFM95_RST 14   
#define RFM95_INT 26   
#define RF95_FREQ 915.0
#define RFM95_TxPower 5 
#define LED 3

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
  startSerial();
  startLoRA();
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
void startSerial(){
  Serial.begin(115200);
  while (!Serial) {
      delay(1000);   // loop forever and don't continue
  }
  Serial.println("starting: RF95_master_hello_world");
}
void startLoRA(){
  pinMode(LED, OUTPUT);
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