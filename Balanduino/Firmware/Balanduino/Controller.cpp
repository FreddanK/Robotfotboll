#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"

//Object, Index
#define BALL 0  //Signature 1 (Ball)
#define GOAL1 1 //Signature 45(octal) 90 < theta <= 180 and -180 <= theta <-90 (Own goal)
#define GOAL2 2 //Signature 45(octal) -90 < theta < 90 (Opponents goal, yellow(left) blue(right))
#define PLAYER1 3 //Signature 23(octal) -180 < theta < 0  (Team member)
#define PLAYER2 4 //Signature 23(octal) 0 < theta < 180 (Opponent, purple(top) green(bottom)
#define EDGE 5 //NA


void Controller::doTask() {
  
  uint16_t blocksCount = pixy.getBlocks();
  
  //Get the time since pixy last saw an object
  uint32_t updateTimer = millis()-pixyTimer;
  //if pixy sees an object the timer needs to be reset

  uint16_t lastXPosBall;
  
  if (blocksCount>0){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
    if(visible(BALL)){
      lastXPosBall=pixy.blocks[objectIndex[BALL]].x;
    }
  }
  //if pixy sees at least two objects, check the distance
  //between the ball and goal to determine which function to call
  
  if(task == search) {
    //Pixys update frequency is 50Hz = 20ms
    //Assume pixy sees something if blocksCount > 0
    //or if it was less than 25ms since it last saw an object.
    //(It needs to be a little higher than 20ms, otherwise there is
    //a chance of missing an object.)
    if(blocksCount || updateTimer<25) {
      if(visible(BALL)){
        goToObject(BALL);
      }
      else {
        findBall(lastXPosBall); //Just temporary. Here should be some kind of look for ball function
      }
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
  else if(task == score){
    scoreGoal();
  }
  else if(task == center){
    centerBall();
  }
  else {
    motor.steer(stop,0);
  }
}

void Controller::findBall(int ballPos){ 
 if(ballPos<120){
  motor.steer(left,20);
 }
 else if(ballPos>200){
  motor.steer(right,20);
 }
 else {
  motor.steer(stop);
 }
}

void Controller::doTaskGoalKeeper() {
  uint16_t blocksCount = pixy.getBlocks();
  
  //Get the time since pixy last saw an object
  uint32_t updateTimer = millis()-pixyTimer;
  //if pixy sees an object the timer needs to be reset
  if (blocksCount>0){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
  }
  if(task == search) {
    //Pixys update frequency is 50Hz = 20ms
    //Assume pixy sees something if blocksCount > 0
    //or if it was less than 25ms since it last saw an object.
    //(It needs to be a little higher than 20ms, otherwise there is
    //a chance of missing an object.)
    if(blocksCount || updateTimer<25) {
      goalKeeper(0);
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
  else {
    motor.steer(stop);
  }
}

void Controller::goalKeeper(int object) {
  boolean hasKicked = false;
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  if(hasKicked == true && width < 50){
    goToObjectGoalkeeper(BALL);
    findBall();
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
   if(visible(BALL)){      
      goToObjectGoalkeeper(BALL);
      task = kick;
      hasKicked = true;
      taskTimer = millis();
      
    }
  }
}

bool Controller::visible(int object) {
  if(objectIndex[object]!=-1){
    return true;
  }
  else return false;
}

void Controller::goToObject(int object) {
  uint16_t xPos = pixy.blocks[objectIndex[object]].x;
  uint16_t width = pixy.blocks[objectIndex[object]].width;

  //if both goal and ball can be seen at the same time
  //check the distance between them to determine which function to call
  /*if (objectIndex[BALL] != -1 && objectIndex[GOAL2] != -1){
    getPixelDistance();
  //if the distance between goal and ball is small
  //start function scoreGoal
  }
  if(pixelDistance<100){
    task=score;
  }
  else*/ 
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
  else if(width>110){
    task=kick;
    taskTimer = millis();
  }
}

//adjust and centers both ball and goal to get
//into "goal scoring position".
//sets task to kick when position is right
void Controller::scoreGoal(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
  uint16_t width = pixy.blocks[objectIndex[BALL]].width;
  //When the goal is far out on either of the edges in pixys field of vision
  //while the ball is not, THEN move forward
//  if((xPosGoal<50 && xPosBall>50 || xPosGoal>260 && xPosBall<260)){
//    motor.steer(forward,20);
//    Serial.println("Forward");
//  }
  //when the ball is to the right of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go right
  if(xPosBall>xPosGoal){
    motor.steer(right,20);
    Serial.println("Go right");
  }
  //when the ball is to the left of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go left
  else if(xPosBall<xPosGoal){
    motor.steer(left,20);
    Serial.println("Go left");
  }
  /*
  //if both ball and goal is far to the left or far to the right, then center ball
  else if((xPosGoal<50 && xPosBall<50) || (xPosGoal>260 && xPosBall>260)){
    task=center;
    Serial.println("Center ball");
  }
  //if ball is centered but still far. move forward
  else if(xPosBall>120 && xPosBall<200 && width > 10 && width < 110){
    motor.steer(forward,20);
    Serial.println("Forward, too far");
  }
  //if ball is centered and close, kick ball
  else if(xPosBall>120 && xPosBall<200 && width>110){
    task=kick;
    taskTimer = millis();
    Serial.println("Kick!");
  }*/
}

void Controller::goToObjectGoalkeeper(int object) {
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
    taskTimer = millis(); //unnecessary right now, but may be useful later
  }
}

//avoids an object by turning away from it 
//and towards the direction of the ball if the ball is visible
void Controller::avoidObject() {
  //gets the x-Position for the opponent to avoid and the ball
  //by first checking their signatures position integer for the blocks array
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosObject = pixy.blocks[objectIndex[PLAYER2]].x;
  //ballVisible = visible(BALL);
  if(true==visible(BALL)){
    //if the ball is to the left of the object turn left
    if(xPosBall<xPosObject){
      motor.steer(left,20);
    }
    //otherwise the ball is to the right of the object, then turn right
    else{
      motor.steer(right,20);
    }
  }
  //If ball isn't visible
  else{
    //turn right if the object to avoid is to the left
    //and left if the object to avoid is to the right
    if(xPosObject<160){
      motor.steer(right,20);
    }
    else if(xPosObject>160){
      motor.steer(left,20);
    }
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

void Controller::getSignatureIndexes(uint16_t actualBlocks) {
  
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
float Controller::distanceToObject(int object_size, float real_size, bool measure_height) {
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
float Controller::distanceBetween(int16_t object1, int16_t object2) {
  uint16_t xPos1 = pixy.blocks[objectIndex[object1]].x;
  uint16_t xPos2 = pixy.blocks[objectIndex[object2]].x;
  float d1 = objectDistance[object1];
  float d2 = objectDistance[object2];
  float image_width = 320; //pixels
  float fov = 75*3.141592654/180; //field of view, degrees

  return sqrt(sq(d1)+sq(d2)-2*d1*d2*cos((abs(xPos1-xPos2)/image_width)*fov)); 
}

//Centers the ball in pixys field of vision
void Controller::centerBall(){ 
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  if(xPosBall<150){
    motor.steer(left,20);
  }
  else if(xPosBall>170){
    motor.steer(right,20);
  }
  else{
    task=score;
  }
}

//gets distance between xPosBall and xPosGoal
//to get the distance between the goal and ball in pixels
void Controller::getPixelDistance(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
  uint16_t pixelDistance = abs(xPosBall-xPosGoal);
}

float Controller::distancePixelsToCm(int object_size_pixels, float real_size_cm, bool measure_height) {
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
    motor.steer(stop);
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
    pixy.setServos(0, tiltLoop.m_pos);
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
    if (m_pos>PIXY_RCS_MAX_POS-20) 
      m_pos = PIXY_RCS_MAX_POS-20; 
    else if (m_pos<PIXY_RCS_MIN_POS+20) 
      m_pos = PIXY_RCS_MIN_POS+20;
  }
  m_prevError = error;
}

