// /*
//  * Main: Purely for testing encoder ticks
//  * Credit to http://bildr.org/2012/08/rotary-encoder-arduino/
//  * 
//  */


// #include <Arduino.h>
// #include <Wire.h>

// int encoderPin1_1 = PB12;
// int encoderPin1_2 = PB13;

// volatile int lastEncoded_1 = 0;
// volatile long encoderValue_1 = 0;

// long lastencoderValue_1 = 0;

// int lastMSB = 0;
// int lastLSB = 0;


// void updateEncoder(){
//   int MSB = digitalRead(encoderPin1_1); //MSB = most significant bit
//   int LSB = digitalRead(encoderPin1_2); //LSB = least significant bit

//   int encoded = (MSB << 1) | LSB; 
//   int sum = (lastEncoded_1 << 2) | encoded;
//   if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_1++; 
//   if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_1--; 
//   lastEncoded_1 = encoded;
// }

// void setup() {
//   Serial.begin (115200);

//   pinMode(encoderPin1_1, INPUT_PULLUP);
//   pinMode(encoderPin1_2, INPUT_PULLUP);

//   //call updateEncoder() when any high/low changed seen
//   //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
//   attachInterrupt(encoderPin1_1, updateEncoder, CHANGE);
//   attachInterrupt(encoderPin1_2, updateEncoder, CHANGE);
// }

// void loop(){
// //Do stuff here
//   Serial.println(encoderValue_1);
//   delay(100); //just here to slow down the output, and show it will work even during a delay
// }

