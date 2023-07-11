#include <PID_v1.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
ros::NodeHandle nh;
//pid for motor 1
double Pk1 = 2;
double Ik1 = 0;
double Dk1 = 0.015;

double Setpoint1, Input1, Output1, Output1a;  // PID variables
PID PID1(&Input1 ,&Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);  //PID setup

//pid for motor 2
double Pk2 = 2.1;
double Ik2 = 0;
double Dk2 = 0.015;

double Setpoint2, Input2, Output2, Output2a;  // PID variables
PID PID2(&Input2 ,&Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);  //PID setup

float demand1;
float demand2;

float demandx;
float demandz;

float speed_left;
float speed_right;

unsigned long currentMillis;
unsigned long previousMillis;
int looptime = 10;

// wheel encoder interrupt
#define encoder_0_PinA 2           // encoder 1
#define encoder_0_PinB 3

#define encoder_1_PinA 18          // encoder 2
#define encoder_1_PinB 19

#define motor_0_in1 8 //motor 1 input 1(pwm pin)
#define motor_0_in2 9 //motor 1 input 2(pwm pin)

#define motor_1_in1 10 //motor 2 input 1(pwm pin)
#define motor_1_in2 11 //motor 2 input 2(pwm pin)

volatile long encoder_0_Pos = 0; //position encoder 1
volatile long encoder_1_Pos = 0; //position encoder 2

float encoder_0_diff;
float encoder_1_diff;

float encoder_0_error;
float encoder_1_error;

float encoder_0_prev;
float encoder_1_prev;

// ROS callback & subscriber

void velCallBack(const geometry_msgs::Twist& vel){
  demandx = vel.linear.x;
  demandz = vel.angular.z;

  demandx = constrain(demandx,-0.25,0.25);
  demandz = constrain(demandz,-1,1);
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallBack);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed",&speed_msg);
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  
  pinMode(motor_0_in1,OUTPUT);
  pinMode(motor_0_in2,OUTPUT);
  pinMode(motor_1_in1,OUTPUT);
  pinMode(motor_1_in2,OUTPUT);

  pinMode(encoder_0_PinA,INPUT_PULLUP);
  pinMode(encoder_0_PinB,INPUT_PULLUP);
  
  pinMode(encoder_1_PinA,INPUT_PULLUP);
  pinMode(encoder_1_PinB,INPUT_PULLUP);
  
  attachInterrupt(0,doencoderA, CHANGE);
  attachInterrupt(1,doencoderB, CHANGE);
  
  attachInterrupt(4,doencoderC, CHANGE);
  attachInterrupt(5,doencoderD, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-200,200);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-200,200);
  PID2.SetSampleTime(10);

  Serial.begin(57600);
}
void loop(){
  nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= looptime) {
    previousMillis = currentMillis;

    demand1= demandx - (demandz*0.24);
    demand2= demandx + (demandz*0.24);
//    Serial.print(encoder_0_Pos);
//    Serial.print(",");
//    Serial.println(encoder_1_Pos);
    
    //work out difference in encoder counts per loop
    encoder_0_diff = encoder_0_Pos - encoder_0_prev;
    encoder_1_diff = encoder_1_Pos - encoder_1_prev;

    encoder_0_error = (demand1*640) - encoder_0_diff;
    encoder_1_error = (demand2*640) - encoder_1_diff;

    encoder_0_prev = encoder_0_Pos;
    encoder_1_prev = encoder_1_Pos;
    
    Setpoint1 = demand1*640;
    Input1 = encoder_0_diff;
    PID1.Compute();

    Setpoint2 = demand2*640;
    Input2 = encoder_1_diff;
    PID2.Compute();
    //actual speed
    speed_left = encoder_0_diff / 64;
    speed_right = encoder_1_diff / 64;
    //drive motor
    //Motor 1
    if (Output1 > 0){
      Output1a = abs (Output1);
      analogWrite(motor_0_in1,Output1a);
      analogWrite(motor_0_in2,0);
    }
    if (Output1 < 0){
      Output1a = abs (Output1);
      analogWrite(motor_0_in2,Output1a);
      analogWrite(motor_0_in1,0);
    }
    //Motor 2
    if (Output2 > 0){
      Output2a = abs (Output2);
      analogWrite(motor_1_in1,Output2a);
      analogWrite(motor_1_in2,0);
    }
    if (Output2 < 0){
      Output2a = abs (Output2);
      analogWrite(motor_1_in2,Output2a);
      analogWrite(motor_1_in1,0);
    }
    publishSpeed(looptime);
  }
}

void publishSpeed(double time){
  speed_msg.header.stamp = nh.now();//time for odometry data
  speed_msg.vector.x = speed_left;//left wheel speed
  speed_msg.vector.y = speed_right;//right wheel
  speed_msg.vector.z = time/1000;//loop time
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//encoderInterrupt
//encoder 1
void doencoderA(){
  // look for a low-to-high on channel A
  if(digitalRead(encoder_0_PinA) == HIGH){
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_0_PinB) == LOW){
      encoder_0_Pos = encoder_0_Pos + 1; //CW
    }
    else {
      encoder_0_Pos = encoder_0_Pos - 1; //CCW
    }
  }
  // looke for a high-to-low egde on channel A
  else {
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_0_PinB) == HIGH){
      encoder_0_Pos = encoder_0_Pos + 1; //CW
    }
    else {
      encoder_0_Pos = encoder_0_Pos - 1; //CCW
    }
  }
}
void doencoderB(){
  // look for a low-to-high on channel B
  if(digitalRead(encoder_0_PinB) == HIGH){
    // check channel A to see which way encoder is turning
    if(digitalRead(encoder_0_PinA) == HIGH){
      encoder_0_Pos = encoder_0_Pos + 1; //CW
    }
    else {
      encoder_0_Pos = encoder_0_Pos - 1; //CCW
    }
  }
  // looke for a high-to-low egde on channel A
  else {
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_0_PinA) == LOW){
      encoder_0_Pos = encoder_0_Pos + 1; //CW
    }
    else {
      encoder_0_Pos = encoder_0_Pos - 1; //CCW
    }
  }
}

// encoder 2
void doencoderC(){
  // look for a low-to-high on channel B
  if(digitalRead(encoder_1_PinA) == HIGH){
    // check channel A to see which way encoder is turning
    if(digitalRead(encoder_1_PinB) == LOW){
      encoder_1_Pos = encoder_1_Pos - 1; //CW
    }
    else {
      encoder_1_Pos = encoder_1_Pos + 1; //CCW
    }
  }
  // looke for a high-to-low egde on channel A
  else {
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_1_PinB) == HIGH){
      encoder_1_Pos = encoder_1_Pos - 1; //CW
    }
    else {
      encoder_1_Pos = encoder_1_Pos + 1; //CCW
    }
  }
}
void doencoderD(){
  // look for a low-to-high on channel B
  if(digitalRead(encoder_1_PinB) == HIGH){
    // check channel A to see which way encoder is turning
    if(digitalRead(encoder_1_PinA) == HIGH){
      encoder_1_Pos = encoder_1_Pos - 1; //CW
    }
    else {
      encoder_1_Pos = encoder_1_Pos + 1; //CCW
    }
  }
  // looke for a high-to-low egde on channel A
  else {
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_1_PinA) == LOW){
      encoder_1_Pos = encoder_1_Pos - 1; //CW
    }
    else {
      encoder_1_Pos = encoder_1_Pos + 1; //CCW
    }
  }
}
