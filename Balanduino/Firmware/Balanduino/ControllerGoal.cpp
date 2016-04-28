
#include "ControllerGoal.h"

#include <Arduino.h>

#include "Motor.h"

//Object, Index
#define BALL 0  //Signature 1 (Ball)
#define GOAL1 1 //Signature 45(octal) 90 < theta <= 180 and -180 <= theta <-90 (Own goal)
#define GOAL2 2 //Signature 45(octal) -90 < theta < 90 (Opponents goal, yellow(left) blue(right))
#define PLAYER1 3 //Signature 23(octal) -180 < theta < 0  (Team member)
#define PLAYER2 4 //Signature 23(octal) 0 < theta < 180 (Opponent, purple(top) green(bottom)
#define EDGE 5 //NA


void ControllerGoal::doTask() {  
  uint16_t blocksCount = pixy.getBlocks();
  
  //Get the time since pixy last saw an object
  uint32_t updateTimer = millis()-pixyTimer;
  //if pixy sees an object the timer needs to be reset

  if (blocksCount>0){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
    if(visible(BALL)){
      lastXPosBall=pixy.blocks[objectIndex[BALL]].x;
    }
  }

  if(task == search) {
    //Pixys update frequency is 50Hz = 20ms
    //Assume pixy sees something if blocksCount > 0
    //or if it was less than 25ms since it last saw an object.
    //(It needs to be a little higher than 20ms, otherwise there is
    //a chance of missing an object.)
    if(blocksCount || updateTimer<25) {
      goalKeeper(BALL);
    }
   
        //Search for objects in the direction where the last
      //seen object went out of sight.
      else {
        findBall(lastXPosBall); //Just temporary. Here should be some kind of look for ball function
      }
  }
  else if(task == kick) {
    kickBall();
  }
  else {
    motor.steer(stop);
  }
}

void ControllerGoal::goalKeeper(int object) {
  boolean hasKicked = false;
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  if(hasKicked == true && width < 50){
    goToObject(GOAL1);
    findBall(lastXPosBall);
    hasKicked = false;
    }

  if(xPos<120){
    motor.steer(left,20);
    }
  else if(xPos>200){
    motor.steer(right,20);
    }
  else if(visible(BALL)){
    if(width < 20){
      motor.steer(stop);
    }
  }
  else if(width > 100){
   if((visible(BALL)) && (distanceBetween(BALL, PLAYER2)) <= 100){      
      goToObject(BALL);
      task = kick;
      hasKicked = true;
      taskTimer = millis();
      
    }
  }
}

bool ControllerGoal::visible(int object) {
  if(objectIndex[object]!=-1){
    return true;
  }
  else return false;
}

void ControllerGoal::goToObject(int object) {
  uint16_t xPos = pixy.blocks[objectIndex[object]].x;
  uint16_t width = pixy.blocks[objectIndex[object]].width;

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
}

void ControllerGoal::kickBall() {
  uint32_t time_since_start = millis() - taskTimer;
  if (time_since_start<500){
    motor.steer(forward,50);
  }
  else if(time_since_start>500 && time_since_start<2000){
    motor.steer(forward,0);
  }  
  else{
    motor.steer(stop);
    taskTimer = millis(); //unnecessary right now, but may be useful later
  }
}


void ControllerGoal::getSignatureIndexes(uint16_t actualBlocks) {
  
  for(int i=0; i<6;i++){
    objectIndex[i] = -1;
    objectDistance[i] = -1;
  }
    
  for(int i=0; i<actualBlocks;i++) {
    uint16_t signature = pixy.blocks[i].signature;
    if(signature == 1) { //Ball
      if(objectIndex[BALL] == -1) { //only store information about the closest object
        objectIndex[BALL] = i;
        objectDistance[BALL] = distanceToObject(pixy.blocks[i].width,12,false);
      }
      //Serial.print(objectSeen[BALL]);
      //Serial.print(" Ball, ");
    }
    else if(signature == 045){ //Opponent goal
      uint16_t angle = pixy.blocks[i].angle; //TODO, angle doesen't seem to update as often as the rest of the parameters from the pixy, so it is not used right now
      if(objectIndex[GOAL2] == -1) { //only store information about the closest object
        objectIndex[GOAL2] = i;
        objectDistance[GOAL2] = distanceToObject(pixy.blocks[i].width,100,false);
      }
      /*
      if(90<angle && angle<=180 || -180<=angle && angle<-90){
        objectIndex[GOAL1] = i; 
      }
      else if(-90 < angle && angle < 90) {
        objectIndex[GOAL2] = i;
      }
      */
    }
    else if(signature = 023) {  //Opponent player
      uint16_t angle = pixy.blocks[i].angle; //TODO, angle doesen't seem to update as often as the rest of the parameters from the pixy, so it is not used right now
      if(objectIndex[PLAYER2] == -1) { //only store information about the closest object
        objectIndex[PLAYER2] = i;
        objectDistance[PLAYER2] = distanceToObject(pixy.blocks[i].height,15,true);
      }
      /*
      if(-180<angle && angle<0){
        objectSeen[PLAYER1] = i;
      }
      else if(0<angle && angle<180) {
        objectSeen[PLAYER2] = i;
      }
      */
    }
  }
}
//Takes the object's size in pixels and converts it to distance in cm.
//Arguments: the object's size (pixels), real size of the object (cm) and an extra argument
//that is true if you want to measure the distance based on the heigth instead of the width.
float ControllerGoal::distanceToObject(int object_size, float real_size, bool measure_height) {
  float focal_length = 0.28; //cm
  float image_width = 320; //pixels
  float image_height = 200; //pixels
  float sensor_width = 0.3888; //cm
  float sensor_height = 0.243; //cm
  if(measure_height)
    return (focal_length*real_size*image_height)/(object_size*sensor_height);
  else
    return (focal_length*real_size*image_width)/(object_size*sensor_width);
  
}

//Calculate the distance between two objects
//Example of use: float var = distanceBetween(BALL,PLAYER2);
float ControllerGoal::distanceBetween(int16_t object1, int16_t object2) {
  uint16_t xPos1 = pixy.blocks[objectIndex[object1]].x;
  uint16_t xPos2 = pixy.blocks[objectIndex[object2]].x;
  float d1 = objectDistance[object1];
  float d2 = objectDistance[object2];
  float image_width = 320; //pixels
  float fov = 75*3.141592654/180; //field of view, degrees

  return sqrt(sq(d1)+sq(d2)-2*d1*d2*cos((abs(xPos1-xPos2)/image_width)*fov)); 
}

//gets distance between xPosBall and xPosGoal
//to get the distance between the goal and ball in pixels
void ControllerGoal::getPixelDistance(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
  uint16_t pixelDistance = abs(xPosBall-xPosGoal);
}

float ControllerGoal::distancePixelsToCm(int object_size_pixels, float real_size_cm, bool measure_height) {
  float focal_length_cm = 0.28;
  float image_width_pixels = 320;
  float image_height_pixels = 200;
  float sensor_width_cm = 0.3888;
  float sensor_height_cm = 0.243;
  if(measure_height)
    return (focal_length_cm*real_size_cm*image_height_pixels)/(object_size_pixels*sensor_height_cm);
  else
    return (focal_length_cm*real_size_cm*image_width_pixels)/(object_size_pixels*sensor_width_cm);
  
}

void ControllerGoal::findBall(int object){
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
    motor.steer(stop);
  }
  else if(width > 10 && width < 110){
    motor.steer(forward,20); 
  }
  else {
    motor.steer(stop);
  }
}

