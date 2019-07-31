#include <Arduino.h>
#include <Wire.h>
#include "Encoder.h"

// great implementation thanks to: https://bildr.org/2012/08/rotary-encoder-arduino/

// class Encoder 
// {
//     int pin1, pin2;

//     volatile int lastEncoded;
//     volatile long encoderValue;
//     long lastencoderValue;

// public:
//     Encoder(int pin1, int pin2);
//     int getPosition();
//     static void setupEncoder(Encoder encoder);
//     static void updateEncoder(Encoder encoder);
//     static void updateEncoder();

// };

Encoder::Encoder(int pin1, int pin2) {
    this->pin1=pin1;
    this->pin2=pin2;
    this->lastEncoded=0;
    this->encoderValue=0;
    this->lastencoderValue=0;
}

int Encoder::getPosition() {
    return this->lastEncoded;
}

int Encoder::getPin1() {
    return this->pin1;
}

int Encoder::getPin2() {
    return this->pin2;
}

void Encoder::setupEncoder(Encoder encoder) {
    pinMode(encoder.pin1, INPUT_PULLUP);
    pinMode(encoder.pin2, INPUT_PULLUP);
    // attachInterrupt(encoder.pin1, Encoder::updateEncoder(), CHANGE);
    // attachInterrupt(encoder.pin2, updateEncoder(), CHANGE);
}


void Encoder::updateEncoder(Encoder& encoder) {
  int MSB = digitalRead(encoder.pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder.pin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; 
  int sum = (encoder.lastEncoded << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder.encoderValue++; 
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder.encoderValue--; 
  encoder.lastEncoded = encoded;
}

void Encoder::updateEncoder() {
  int MSB = digitalRead(this->pin1); //MSB = most significant bit
  int LSB = digitalRead(this->pin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; 
  int sum = (this->lastEncoded << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) this->encoderValue++; 
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) this->encoderValue--; 
  this->lastEncoded = encoded;
}


int tur() {
    Encoder encoder(1,2);
    Encoder::updateEncoder(encoder);
    //attachInterrupt(2, Encoder::updateEncoder(encoder), CHANGE);
    return 0;
}

