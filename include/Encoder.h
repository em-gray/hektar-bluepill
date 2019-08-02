#ifndef ENCODER_H
#define ENCODER_H

class Encoder
{
    private:
        int pin1, pin2;

        volatile long lastEncoded;
        volatile long encoderValue;
        long lastencoderValue;
        
    public:
        Encoder(int pin1, int pin2);
        long getPosition();
        int getPin1();
        int getPin2();

        static void setupEncoder(Encoder encoder);
        static void updateEncoder(Encoder& encoder);
        void updateEncoder();

};

#endif