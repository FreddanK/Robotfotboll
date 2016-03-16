#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"


//Makes the Balanduino to go forward for 3sek, stop, turn 90deg,go forward for 3sek and stop
void Controller::setControlOffset(Motor *motor){
  int time_since_start = millis();
  if(time_since_start>2000 && time_since_start<5000){
    motor->steer(imu, 20 ,0);
  }
  else if(time_since_start>6000 && time_since_start<8000){
    motor->steer(imu, 0, 10);
  }
  else if(time_since_start>9000 && time_since_start<12000){
    motor->steer(imu, 20, 0);
  }
  else {
    motor->steer(stop);
  }
}
