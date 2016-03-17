#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"


//Makes the Balanduino to go forward for 3sek, stop, turn 90deg,go forward for 3sek and stop
void Controller::setControlOffset(Motor *motor){
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
