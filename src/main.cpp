/*
 * Main
 * 
 * 
 */

#include <Arduino.h>
//#include <Wire.h>
#include <Servo.h>

#include <ros.h>
#include <hektar/armCtrl.h>
#include <hektar/wheelVelocity.h>
#include <hektar/IRarray.h>
#include <hektar/armPos.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// Define claw servo output pins
#define CLAW_L PA6
#define CLAW_R PA0

// Define PWM output pins
#define BASE_PWM PB_9
#define SHOULDER_PWM PB_8
#define ELBOW_PWM PA_8
#define wheelR_PWM PB_7
#define wheelL_PWM PB_6

// Define toggle pins for PWM output
#define toggleBase PA12
#define toggleShoulder PA11
#define toggleElbow PA15
#define toggleWheelR PB4
#define toggleWheelL PB5

// Define arm potentiometer input pins
#define BASE_POT PB1
#define SHOULDER_POT PB0
#define ELBOW_POT PA7

// Define IR array input pins
#define IR0 PA_1
#define IR1 PA_2
#define IR2 PA_3
#define IR3 PA_4
#define IR4 PA_5

#define PWM_MAX_DUTY 500

int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

/* Function: lineralize
 * Params: int8 from -127 to 127
 * Returns: PWM duty cycle length, up to PWM_MAX_DUTY
 */
int linearize(int pwmPercent) { 
  if (pwmPercent > 0) {
    return pwmPercent*PWM_MAX_DUTY/127;
  } else {
    return -pwmPercent*PWM_MAX_DUTY/127;
  }
}

float get_servo_pulse(int angle){
  return (angle/180.0 + 1);
}

// Theoretically, 1ms pulse moves it to 0 degree state, 2ms pulse moves it to 180 degree state,
// and everything in between is linear.
void claw_callback(const hektar::Claw &claw_cmd_msg) {
  int leftPulse = 10 * get_servo_pulse(claw_cmd_msg.posL);
  int rightPulse = 10 * get_servo_pulse(claw_cmd_msg.posR)
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
    digitalWrite(toggleELbow, 1);
    pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  } else {
    digitalWrite(toggleELbow, 0);
    pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  }

  if (arm_cmd_msg.shoulderVel > 0) {
      digitalWrite(toggleShoulder, 1);
      pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
  } else {
      digitalWrite(toggleShoulder, 0);
      pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
  }
  
  if (arm_cmd_msg.baseVel > 0) {
    digitalWrite(toggleBase, 1);
    pwm_start(BASE_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  } else {
    digitalWrite(toggleBase, 0);
    pwm_start(BASE_PWM, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
  }

}

hektar::IRarray ir_msg; 

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

ros::NodeHandle nh;
hektar::armPos armpos_msg;


ros::Publisher armpub("arm_positions", &armpos_msg);
ros::Publisher irpub("ir_array", &ir_msg);

ros::Subscriber<hektar::armCtrl> armSub("arm_commands", arm_callback);
ros::Subscriber<hektar::wheelVelocity> wheelSub("wheel_output", wheelVel_callback);
ros::Subscriber<hektar::Claw> clawSub("grabber", claw_callback);


void setup() {
  //ros stuff
  nh.initNode();
  nh.advertise(armpub);
  nh.advertise(irpub);
  nh.subscribe(armSub);
  nh.subscribe(wheelSub);

  // //setup of pins 
  pinMode(CLAW_L, OUTPUT);
  pinMode(CLAW_R, OUTPUT);

  pinMode(SHOULDER_PWM, OUTPUT); 
  pinMode(ELBOW_PWM, OUTPUT); 

  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(BASE_POT, INPUT);
  pinMode(ELBOW_POT, INPUT);
  pinMode(SHOULDER_POT, INPUT);

  pinMode(toggleWheelL, OUTPUT);
  pinMode(toggleWheelR, OUTPUT);
  pinMode(toggleBase, OUTPUT);
  pinMode(toggleShoulder, OUTPUT);
  pinMode(toggleElbow, OUTPUT);

  pwm_start(wheelL_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(wheelR_PWM, 100000, PWM_MAX_DUTY, 0, 1);

  pwm_start(SHOULDER_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(ELBOW_PWM, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(BASE_PWM, 100000, PWM_MAX_DUTY, 0, 1);

  // Values courtesy of Mo
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

  //armpub.publish(&armpos_msg);

  // ARM Publication: 
  //reading data to publish
  armpos_msg.basePos = analogRead(BASE_POT);
  armpos_msg.shoulderPos = analogRead(SHOULDER_POT);
  armpos_msg.elbowPos = analogRead(ELBOW_POT);

  //ros stuff
  irpub.publish(&ir_msg);
  armpub.publish(&armpos_msg);

  nh.spinOnce();

  delay(20);
}

/********** ADC STUFF **********/
  //rosserial_arduino::Adc adc_msg;
  //ros::Publisher p("adc", &adc_msg);

  // adc_msg.adc0 = averageAnalog(PA0);
  // adc_msg.adc1 = averageAnalog(PA1);
  // adc_msg.adc2 = averageAnalog(PA2);
  // adc_msg.adc3 = averageAnalog(PA3);
  // adc_msg.adc4 = averageAnalog(PA4);
  // p.publish(&adc_msg);


/********** SIMPLE BLINK **********/
  // void setup()
  // {
  //   pinMode(PC13, OUTPUT);
  //   digitalWrite(PC13, HIGH);
  // }

  // void loop()
  // {
  //   digitalWrite(PC13, HIGH);
  //   delay(1000);
  //   digitalWrite(PC13, LOW);
  //   delay(1000);
  // }




/********** SERVO SWEEP **********/
  // Servo myservo;  // create servo object to control a servo
  // // twelve servo objects can be created on most boards

  // int pos = 0;    // variable to store the servo position

  // void setup() {
  //   myservo.attach(PB7);  
  //   pinMode(PC13, OUTPUT);
  //   digitalWrite(PC13, HIGH);
  // }

  // void loop() {
  //   myservo.write(0);
  //   digitalWrite(PC13, HIGH);
  //   delay(2000);

  //   myservo.write(180);
  //   digitalWrite(PC13, LOW);
  //   delay(2000);
  // }