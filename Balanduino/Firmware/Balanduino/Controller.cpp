#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"


#define BALL 1
#define OPPONENT 2


void Controller::doTask() {
  blocksCount = pixy->getBlocks();

  if(task == search) {
    int i;
    for(i=0; i<blocksCount; i++) {
      if pixy->blocks[i].signature == BALL) {
        goToObject(i,BALL);
      }
      else if(pixy->blocks[i].signature == OPPONENT) {
        goToObject(i,OPPONENT);
      }
    }
  }
  else if(task == kick) {
    kickBall();
  }
  else if(task == avoid) {
    avoidObject();
  }
}

void Controller::goToObject(int object, int signature) {
  int xPos = pixy->blocks[object].x;
  int width = pixy->blocks[object].width;

  if(xPos<120){
    motor->steer(right,20);
  }
  else if(xPos>200){
    motor->steer(left,20);
  }
  else if(width > 10 && width < 110){
    motor->(forward,30);
  }
  else if(width > 110){
    if(signature == BALL){
      motor->steer(stop);
      task = kick;
      taskTimer = millis();
    }
    else if(signature == OPPONENT){
      motor->steer(stop);
      task=avoid;
      taskTimer = millis();
    }
  }
}

void Controller::kickBall() {
  int time_since_start = millis() - taskTimer;
  if (time_since_kickstart<500){
    motor->steer(forward,50);
  }
  
  else if(time_since_start>500 && time_since_start<2000){
    motor->steer(stop);
  }  
  else{
    motor->steer(stop);
    task=search;
    taskTimer = millis(); //unnecessary right now, bur may be useful later
  }
}

void Controller::avoidObject(int object) {
  int time_since_start = millis() - taskTimer;
  if (time_since_kickstart<500){
    motor->steer(right, 50);
    motor->steer(forward,10);
  }
  
  else if(time_since_start>500 && time_since_start<2000){
    motor->steer(left,20);
    motor->steer(forward,30);
  }  
  else{
    motor->steer(stop);
    task=search;
    taskTimer = millis(); //unnecessary right now, bur may be useful later
  }
}


void Controller::moveBacknForth(){
  unsigned long time_since_start = millis();
  if(time_since_start>3000 && time_since_start<5000){
    motor->steer(forward, 20);
  }
  else if(time_since_start>6000 && time_since_start<8000){
    motor->steer(left, 40);
    motor->steer(forward, 10);
  }
  else if(time_since_start>9000 && time_since_start<12000){
    motor->steer(backward, 20);
  }
  else {
    motor->steer(stop);
  }
}

void Controller::findBall(){
  uint16_t blocksCount;
  blocksCount = pixy->getBlocks();
  
  int width = pixy->blocks[0].width;
  int xPos = pixy->blocks[0].x;
  
  if(xPos<140){
    motor->steer(right,20);
  }
  else if(xPos>170){
    motor->steer(left,20);
  }
  else if(width>80 && width < 180){
    motor->steer(backward,20);
  }
  else if(width < 80 && width > 20){
    motor->steer(forward,20);
  }
  else {
    motor->steer(stop);
  }
}

void Controller::rotate360() {
  unsigned long time_since_start = millis();
  if(time_since_start>2000 && time_since_start<10000){
    motor->steer(left,20);  
  }
  if(time_since_start>7000){
    motor->steer(stop);
  }
}

void Controller::makeCircle() {
  unsigned long time_since_start = millis();
  if(time_since_start>2000 && time_since_start<10000){
    motor->steer(right,20);
    motor->steer(forward,40);  
  }
  else if(time_since_start>10000){
    motor->steer(stop);
  }
}

