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

// #define CLAW_L PA6
// #define CLAW_R PA0

// #define SHOULDER THINGS NOW THAT WE'RE USING A LOGIC SETUP THIS CODE NEEDS TO CHANGE
// Shoulder toggle will be PA4 and shoulder PWM is going to be PA10

// #define ELBOW_F PA_8
// #define ELBOW_B PA_9

// #define BASE_CW PB_8
// #define BASE_CCW PB_9

#define wheelControlR PB_7
#define toggleWheelR PB4

#define wheelControlL PB_6
#define toggleWheelL PB5

// #define BASE_READ PB1
// #define SHOULDER_READ PB0
// #define ELBOW_READ PA7

#define IR0 PA_1
#define IR1 PA_2
#define IR2 PA_3
#define IR3 PA_4
#define IR4 PA_5

#define PWM_MAX_DUTY 500

// Servo claw_l;
// Servo clar_r;

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

void arm_callback(const hektar::armCtrl &arm_cmd_msg) {
//   claw_l.write(arm_cmd_msg.l_claw);   
//   clar_r.write(arm_cmd_msg.r_claw);

//   if (arm_cmd_msg.shoulderVel > 0) {

//       pwm_start(SHOULDER_F, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
//       pwm_start(SHOULDER_B, 100000, PWM_MAX_DUTY, 0, 0);
//   } else {
//       pwm_start(SHOULDER_B, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.shoulderVel), 0);
//       pwm_start(SHOULDER_F, 100000, PWM_MAX_DUTY, 0, 0);
//   }
  
//   if (arm_cmd_msg.elbowVel > 0) {
//     pwm_start(ELBOW_F, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
//     pwm_start(ELBOW_B, 100000, PWM_MAX_DUTY, 0, 0);
//   } else {
//     pwm_start(ELBOW_B, 100000, PWM_MAX_DUTY, linearize(arm_cmd_msg.elbowVel), 0);
//     pwm_start(ELBOW_F, 100000, PWM_MAX_DUTY, 0, 0);
//   }

}

hektar::IRarray ir_msg; 

void wheelVel_callback(const hektar::wheelVelocity  &wheel_cmd_msg) {



  if (wheel_cmd_msg.wheelL > 0) {
      digitalWrite(toggleWheelL, 1);
      pwm_start(wheelControlL, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelL), 0);
  } else {
      digitalWrite(toggleWheelL, 0);
      pwm_start(wheelControlL, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelL), 0);
  }
  
  if (wheel_cmd_msg.wheelR > 0) {
      digitalWrite(toggleWheelR, 0);
      pwm_start(wheelControlR, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelR), 0);
  } else {
      digitalWrite(toggleWheelR, 1);
      pwm_start(wheelControlR, 100000, PWM_MAX_DUTY, linearize(wheel_cmd_msg.wheelR), 0);
  }

}

ros::NodeHandle nh;
hektar::armPos armpos_msg;


//ros::Publisher armpub("arm_positions", &armpos_msg);
ros::Publisher irpub("ir_array", &ir_msg);

ros::Subscriber<hektar::armCtrl> armSub("arm_commands", arm_callback);
ros::Subscriber<hektar::wheelVelocity> wheelSub("wheel_output", wheelVel_callback);


void setup() {
  //ros stuff
  nh.initNode();
  //nh.advertise(armpub);
  nh.advertise(irpub);
  nh.subscribe(armSub);
  nh.subscribe(wheelSub);

  // //setup of pins 
  // claw_l.attach(CLAW_L);
  // clar_r.attach(CLAW_R);

  // pinMode(SHOULDER_F, OUTPUT); 
  // pinMode(SHOULDER_B, OUTPUT); 
  // pinMode(ELBOW_F, OUTPUT); 
  // pinMode(ELBOW_B, OUTPUT); 

  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  // pinMode(BASE_READ, INPUT);
  // pinMode(ELBOW_READ, INPUT);
  // pinMode(SHOULDER_READ, INPUT);

  pinMode(toggleWheelL, OUTPUT);
  pinMode(toggleWheelR, OUTPUT);
  pinMode(wheelControlL, OUTPUT);
  pinMode(wheelControlR, OUTPUT);

  pwm_start(wheelControlL, 100000, PWM_MAX_DUTY, 0, 1);
  pwm_start(wheelControlR, 100000, PWM_MAX_DUTY, 0, 1);

  // pwm_start(SHOULDER_F, 100000, PWM_MAX_DUTY, 0, 1);
  // pwm_start(SHOULDER_B, 100000, PWM_MAX_DUTY, 0, 1);
  // pwm_start(ELBOW_F, 100000, PWM_MAX_DUTY, 0, 1);
  // pwm_start(ELBOW_B, 100000, PWM_MAX_DUTY, 0, 1);

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
  // armpos_msg.basePos = analogRead(BASE_READ);
  // armpos_msg.shoulderPos = analogRead(SHOULDER_READ);
  // armpos_msg.elbowPos = analogRead(ELBOW_READ);
  //ros stuff
  irpub.publish(&ir_msg);

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