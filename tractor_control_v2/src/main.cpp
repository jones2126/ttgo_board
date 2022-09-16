#include <Arduino.h>
/*
Functions: 
get radio transmission from lora, decode speed and steering instruction
read actual tractor speed and steering angle
instruct the transmission controller and steering controller to move 

testing:
- read the lo-resolution angle sensor

code from original
  if (millis() - prev_analog_pot_read >= read_analog_pot_Interval) {  
    current_position_low_res = analogRead(A1); //pin 15 on Teensy 3.2
    front_angle_low_res.data = current_position_low_res;    // publish ROS topic
    vishaypot_front_angle_low_res.publish(&front_angle_low_res);    
    prev_analog_pot_read = millis();
  }
*/
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - prev_analog_pot_read >= read_analog_pot_Interval) {  
    current_position_low_res = analogRead(A1); //pin 15 on Teensy 3.2
    front_angle_low_res.data = current_position_low_res;    // publish ROS topic
    vishaypot_front_angle_low_res.publish(&front_angle_low_res);    
    prev_analog_pot_read = millis();
  }



}