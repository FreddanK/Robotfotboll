#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"


#define BALL 1
#define OPPONENT 2


void Controller::doTask() {
  uint16_t blocksCount = pixy.getBlocks();
  
  //Get the time since pixy last saw an object
  uint32_t updateTimer = millis()-pixyTimer;
  //if pixy sees an object the timer needs to be reset
  if (blocksCount>0)
    pixyTimer = millis();
  
  if(task == search) {
    //Pixys update frequency is 50Hz = 20ms
    //Assume pixy sees something if blocksCount > 0
    //or if it was less than 25ms since it last saw an object.
    //(It needs to be a little higher than 20ms, otherwise there is
    //a chance of missing an object.)
    if(blocksCount || updateTimer<25) {
      goToObject(0,BALL);
    }
    //This delay is to make the robot stop properly.
    //Without it the robot is very unstable when an object suddently
    //goes out of sight.
    else if(updateTimer>=25 && updateTimer < 1500) {
      motor.steer(stop);
    }
    else {
      //Search for objects in the direction where the last
      //seen object went out of sight.
      uint16_t xPos = pixy.blocks[0].x;
      uint16_t width = pixy.blocks[0].width;

      if(xPos<120){
        motor.steer(left,20);
      }
      else if(xPos>200){
        motor.steer(right,20);
      }
    }
  }
  else if(task == kick) {
    kickBall();
  }
  else if(task == avoid) {
    avoidObject();
  }
  else {
    motor.steer(stop);
  }
}

void Controller::goToObject(int object, int signature) {
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  if(xPos<120){
    motor.steer(left,20);
    motor.steer(forward,5);
  }
  else if(xPos>200){
    motor.steer(right,20);
    motor.steer(forward,5);
  }
  else if(width > 10 && width < 110){
    motor.steer(forward,20);
  }
  else if(width >= 110){
    if(signature == BALL){
      task = kick;
      taskTimer = millis();
    }
    else if(signature == OPPONENT){
      task=avoid;
      taskTimer = millis();
    }
  }
}

void Controller::kickBall() {
  uint32_t time_since_start = millis() - taskTimer;
  if (time_since_start<500){
    motor.steer(forward,50);
  }
  else if(time_since_start>500 && time_since_start<2000){
    motor.steer(forward,0);
  }  
  else{
    motor.steer(stop);
    task=search;
    taskTimer = millis(); //unnecessary right now, bur may be useful later
  }
}

void Controller::avoidObject() {
  int time_since_start = millis() - taskTimer;
  if (time_since_start<500){
    motor.steer(right, 50);
    motor.steer(forward,10);
  }
  
  else if(time_since_start>500 && time_since_start<2000){
    motor.steer(left,20);
    motor.steer(forward,30);
  }
  else{
    motor.steer(stop);
    task=search;
    taskTimer = millis(); //unnecessary right now, bur may be useful later
  }
}

void Controller::setupEncoderMove(float d, float r, float s) {
  startValue = motor.getWheelsPosition();
  targetDistance = d;
  radius = r;
  speed = s;
}

void Controller::setupEncoderSpin(float degrees, float s) {
  startLeftvalue = motor.readRightEncoder();
  startRightvalue = motor.readLeftEncoder();
  targetTurningDistance = (degrees/360)*2*3.141592654*10; //The distance between the wheels on the robot is 20cm, so the radius is 20/2 = 10cm
  rate = s;
}

void Controller::encoderMove(){
  float circumference = 30.9211; //the wheel's circumference in cm
  int32_t pulses = 1856; //number of pulses per revolation (928 for each wheel)
  
  float position = startValue + motor.getWheelsPosition();
  float distance = -position*circumference/(pulses*2); //position is negative when driving forward and positive when driving backwards

  if (targetDistance > 0) { //Drive forwards
    if (distance < targetDistance-speed){
      motor.steer(stop);   //This is only to make sure turningOffset is 0
      motor.steer(forward,speed);
      motor.turningRadius = radius;
    }
    else if(distance >= targetDistance-speed){
      motor.steer(stop);
    }
  }
  else if(targetDistance < 0) { //Drive backwards
    if (distance > targetDistance+speed){
      motor.steer(stop);   //This is only to make sure turningOffset is 0
      motor.steer(backward,speed);
      motor.turningRadius = radius;
    }
    else if(distance <= targetDistance+speed){
      motor.steer(stop);
    }
  }
}

void Controller::encoderSpin() {
  float circumference = 30.9211; //the wheel's circumference in cm
  int32_t pulses = 1856; //number of pulses per revolation (928 for each wheel)
  
  int32_t rightwheelpos = motor.readRightEncoder() + startLeftvalue;
  int32_t leftwheelpos = motor.readLeftEncoder() + startRightvalue;
  
  float leftdistance = -leftwheelpos*circumference/pulses; //the position is negative when driving forwards and positive when driving backwards
  float rightdistance = -rightwheelpos*circumference/pulses;

  if(targetTurningDistance > 0) { //spin right
    if(rightdistance < targetTurningDistance){
      motor.steer(right,rate);
    }
    else if(rightdistance >= targetTurningDistance) {
      motor.steer(stop);
    }
  }
  else if(targetTurningDistance < 0) { //spin left
    if(rightdistance > targetTurningDistance) {
      motor.steer(left,rate);
    }
    else if (rightdistance <= targetTurningDistance) {
      motor.steer(stop);
    }
  }
  else
    motor.steer(stop);
}

void Controller::moveBacknForth(){
  unsigned long time_since_start = millis();
  if(time_since_start>3000 && time_since_start<5000){
    motor.steer(forward, 20);
  }
  else if(time_since_start>6000 && time_since_start<8000){
    motor.steer(left, 40);
    motor.steer(forward, 10);
  }
  else if(time_since_start>9000 && time_since_start<12000){
    motor.steer(backward, 20);
  }
  else {
    motor.steer(stop);
  }
}

void Controller::findBall(){
  uint16_t blocksCount = pixy.getBlocks();
  
  int width = pixy.blocks[0].width;
  int xPos = pixy.blocks[0].x;
  
  if(xPos<120){
    motor.steer(left,20);
    motor.steer(forward,0);
    //motor.steerStop = true;
    //motor.turningOffset = -12;
  }
  else if(xPos>200){
    motor.steer(right,20);
    motor.steer(forward,0);
  }
  else if(width>=110){
    task=kick;
    taskTimer = millis();
  }
  else if(width > 10 && width < 110){
    motor.steer(forward,20); 
  }
  else {
    motor.steer(stop);
  }
}

void Controller::rotate360() {
  unsigned long time_since_start = millis();
  if(time_since_start>2000 && time_since_start<10000){
    motor.steer(left,20);  
  }
  if(time_since_start>7000){
    motor.steer(stop);
  }
}

void Controller::makeCircle() {
  unsigned long time_since_start = millis();
  if(time_since_start>2000 && time_since_start<10000){
    motor.steer(right,30);
    motor.steer(forward,20); 
  }
  else if(time_since_start>10000){
    motor.steer(stop);
  }
}

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
ServoLoop tiltLoop(500, 700);

void Controller::tiltServo() {
  uint16_t blocksCount = pixy.getBlocks();
  if(blocksCount) {
    int32_t tiltError = pixy.blocks[0].y - Y_CENTER;
    tiltLoop.update(tiltError);
    pixy.setServos(0, PIXY_RCS_MAX_POS-tiltLoop.m_pos);
  }
}


ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}
