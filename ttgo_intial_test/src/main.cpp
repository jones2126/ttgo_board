#include <Arduino.h>

void setup(){
byte x=50;
Serial.begin(9600);
if (x >=10 && x <=20){
Serial.println("statement 1");
}
if (x >=10 || x <=20) {
Serial.println("statement 2");
}
if (x >=10 && !(x<=20)) {
Serial.println("statement 3");
}
}
void loop(){
} 