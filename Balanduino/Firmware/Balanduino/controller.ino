#include "controller.h"
#include "Balanduino.h"
#include <Arduino.h>

//Set ENABLE_AI and disable ENABLE_SPP and ENABLE_ADK in Balanduino.ino to make this work

//Makes the Balanduino to go forward for 3sek, stop, turn 90deg,go forward for 3sek and stop
void setControlOffset(){
  int time_since_start = millis();
  steerStop = false;
  if(time_since_start>2000 && time_since_start<5000){
    targetOffset = scale(30, 0, 36, 0, cfg.controlAngleLimit);
    turningOffset = scale(0, 0, 45, 0, cfg.turningLimit);
    targetPosition = getWheelsPosition();
    stopped = false;
  }
  else if(time_since_start>6000 && time_since_start<8000){
    targetOffset = scale(0, 0, 36, 0, cfg.controlAngleLimit);
    turningOffset = scale(20, 0, 45, 0, cfg.turningLimit);
  }
  else if(time_since_start>9000 && time_since_start<12000){
    targetOffset = scale(30, 0, 36, 0, cfg.controlAngleLimit);
    turningOffset = scale(0, 0, 45, 0, cfg.turningLimit);
    targetPosition = getWheelsPosition();
    stopped = false;
  }
  else {
    targetOffset = 0;
    turningOffset = 0;
    targetPosition = getWheelsPosition();
    stopped = true;
    steerStop = true;
  }
}

