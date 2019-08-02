/*
 * Main
 */

#include <Arduino.h>
//#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>
#include <ros.h>
#include <hektar/armCtrl.h>
#include <hektar/wheelVelocity.h>
#include <hektar/Claw.h>
#include <hektar/IRarray.h>
#include <hektar/armPos.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// Define claw servo output pins
#define CLAW_L PA_6
#define CLAW_R PA_0

// PWM output pins
#define BASE_PWM PB_9
#define SHOULDER_PWM PB_8
#define ELBOW_PWM PA_8
#define wheelR_PWM PB_7
#define wheelL_PWM PB_6

// Max duty cycle for PWM outputs
#define PWM_MAX_DUTY 2000 //500

// Toggle pins for PWM output
#define toggleShoulder PA11
#define toggleElbow PA15
#define toggleWheelR PB4
#define toggleWheelL PB5

// Arm potentiometer input pins
#define SHOULDER_POT PB0
#define ELBOW_POT PA7

// IR array input pins
#define IR0 PA_1
#define IR1 PA_2
#define IR2 PA_3
#define IR3 PA_4
#define IR4 PA_5

// Encoder input pins
#define ENCODER_L_1 PB_12
#define ENCODER_L_2 PB_13
#define ENCODER_R_1 PB_14
#define ENCODER_R_2 PB_3

// Define
#define LEFT_PIN PB11

int basePulse = 22;
std_msgs::Float64 debug;

/* Function: averageAnalog
*  Params: analog input pin number
*  Returns: analog value representing rolling average
*/
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

/* Function: lineralize
*  Params: int8 from -127 to 127
*  Returns: PWM duty cycle length, up to PWM_MAX_DUTY
*/
int linearize(int pwmPercent) { 
  if (pwmPercent > 0) {
    return pwmPercent*PWM_MAX_DUTY/127;
  } else {
    return -pwmPercent*PWM_MAX_DUTY/127;
  }
}

/* Function: getClawServoPulse
*  Params: angle, int from 0-180
*  Returns: pulse length that moves claw servo to desired location
*/
float getClawServoPulse(int angle){
  return ((49.0 * angle)/180.0 + 1.0);
}

/* Function: getBaseServoPulse
*  Params: angle, int from -90 to 90
*  Returns: pulse length that moves base servo to desired location
*/
float getBaseServoPulse(int angle){
   float val = ((22.0 * angle)/180.0 + 16.0);
   if (val > 27.0){
     return 27.0;
   } else if (val < 5){
     return 5.0;
   }
   return val;

}

/*  Theoretically, 1ms pulse moves it to 0 degree state, 2ms pulse moves it to 180 degree state,
 and everything in between is linear. */
void claw_callback(const hektar::Claw &claw_cmd_msg) {
  int leftPulse = 50.0 - getClawServoPulse(claw_cmd_msg.posL);
  int rightPulse = getClawServoPulse(claw_cmd_msg.posR);
  pwm_start(CLAW_L, 10000, 200, leftPulse, 0);
  pwm_start(CLAW_R, 10000, 200, rightPulse, 0);
}

void arm_callback(const hektar::armCtrl &arm_cmd_msg) {

  if (arm_cmd_msg.shoulderVel > 0) {
      digitalWrite(toggleShoulder, 1);
      pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
  } else {
      digitalWrite(toggleShoulder, 0);
      pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
  }
  
  if (arm_cmd_msg.elbowVel > 0) {
    digitalWrite(toggleElbow, 1);
    pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  } else {
    digitalWrite(toggleElbow, 0);
    pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  }

  float basePulse = getBaseServoPulse
  (arm_cmd_msg.baseVel);
  debug.data = basePulse;
  pwm_start(BASE_PWM, 100000, PWM_MAX_DUTY, 10*basePulse, 0);
  

}

void wheelVel_callback(const hektar::wheelVelocity  &wheel_cmd_msg) {



  if (wheel_cmd_msg.wheelL > 0) {
      digitalWrite(toggleWheelL, 1);
      pwm_start(wheelL_PWM, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelL), 0);
  } else {
      digitalWrite(toggleWheelL, 0);
      pwm_start(wheelL_PWM, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelL), 0);
  }
  
  if (wheel_cmd_msg.wheelR > 0) {
      digitalWrite(toggleWheelR, 0);
      pwm_start(wheelR_PWM, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelR), 0);
  } else {
      digitalWrite(toggleWheelR, 1);
      pwm_start(wheelR_PWM, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelR), 0);
  }

}


hektar::IRarray ir_msg; 
ros::NodeHandle nh;
hektar::armPos armpos_msg;
std_msgs::Bool left;

ros::Publisher armpub("arm_positions", &armpos_msg);
ros::Publisher irpub("ir_array", &ir_msg);
ros::Publisher debugger("debug", &debug);
ros::Publisher leftpub("left", &left);

ros::Subscriber<hektar::armCtrl> armSub("arm_commands", arm_callback);
ros::Subscriber<hektar::wheelVelocity> wheelSub("wheel_output", wheelVel_callback);
ros::Subscriber<hektar::Claw> clawSub("grabber", claw_callback);

Encoder encoderL(ENCODER_L_1, ENCODER_L_2);
Encoder encoderR(ENCODER_R_1, ENCODER_R_2);

void updateEncoderL() {
  encoderL.updateEncoder();
}

void updateEncoderR() {
  encoderR.updateEncoder();
}

void leftBool() {
  left.data = 
  leftpub.publish();
}

void setup() {
  //ros stuff
  nh.initNode();
  nh.advertise(armpub);
  nh.advertise(irpub);
  nh.advertise(leftpub);
  nh.advertise(debugger);
  nh.subscribe(armSub);
  nh.subscribe(clawSub);
  nh.subscribe(wheelSub);

  // //setup of pins 
  pinMode(ENCODER_L_1, INPUT_PULLUP);
  pinMode(ENCODER_L_2, INPUT_PULLUP);
  pinMode(ENCODER_R_1, INPUT_PULLUP);
  pinMode(ENCODER_R_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_1), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_2), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_1), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_2), updateEncoderR, CHANGE);

  pinMode(CLAW_L, OUTPUT);
  pinMode(CLAW_R, OUTPUT);

  pinMode(SHOULDER_PWM, OUTPUT); 
  pinMode(ELBOW_PWM, OUTPUT); 
  pinMode(BASE_PWM, OUTPUT);

  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  pinMode(ELBOW_POT, INPUT);
  pinMode(SHOULDER_POT, INPUT);

  pinMode(toggleWheelL, OUTPUT);
  pinMode(toggleWheelR, OUTPUT);

  pinMode(toggleShoulder, OUTPUT);
  pinMode(toggleElbow, OUTPUT);

  pwm_start(wheelL_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(wheelR_PWM, 100000, PWM_MAX_DUTY, 0, 1);

  pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(BASE_PWM, 100000, PWM_MAX_DUTY, 225, 1);

  // // Values courtesy of Mo
  pwm_start(CLAW_L, 10000, 200, 0, 1);
  pwm_start(CLAW_R, 10000, 200, 0, 1);

}

void loop() {
  // IR Array Publication
  ir_msg.ir_0 = averageAnalog(PA1);
  ir_msg.ir_1 = averageAnalog(PA2);
  ir_msg.ir_2 = averageAnalog(PA3);
  ir_msg.ir_3 = averageAnalog(PA4);
  ir_msg.ir_4 = averageAnalog(PA5);

  // ARM Publication: 
  //reading data to publish
  armpos_msg.basePos = 0;
  armpos_msg.shoulderPos = analogRead(SHOULDER_POT);
  armpos_msg.elbowPos = analogRead(ELBOW_POT);

  //ros stuff

  irpub.publish(&ir_msg);
  armpub.publish(&armpos_msg);
  debugger.publish(&debug);

  nh.spinOnce();

  delay(20);
}