#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"


#define BALL 1
#define OPPONENT 2
#define GOAL 3


void Controller::doTask() {
  uint16_t blocksCount = pixy.getBlocks();
  
  //Get the time since pixy last saw an object
  uint32_t updateTimer = millis()-pixyTimer;
  //if pixy sees an object the timer needs to be reset
  if (blocksCount>0){
    pixyTimer = millis();
    actualBlocks = blocksCount;
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
  else if(task=score){
    scoreGoal();
  }
  else if(task=center){
    centerBall();
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
    actualBlocks = blocksCount;
  }
  if(task == search) {
    //Pixys update frequency is 50Hz = 20ms
    //Assume pixy sees something if blocksCount > 0
    //or if it was less than 25ms since it last saw an object.
    //(It needs to be a little higher than 20ms, otherwise there is
    //a chance of missing an object.)
    if(blocksCount || updateTimer<25) {
      goalKeeper(0,BALL);
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

void Controller::goalKeeper(int object, int signature) {
  boolean hasKicked = false;
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  if(hasKicked == true && width < 50){
    goToObjectGoalkeeper(0, GOAL);
    findBall();
    hasKicked = false;
    }

  if(xPos<120){
    motor.steer(left,20);
    }
  else if(xPos>200){
    motor.steer(right,20);
    }
  else if(width < 20){
    if(signature == BALL){
      motor.steer(stop);
    }
  }
  else if(width > 100){
   if(signature == BALL){      
      goToObjectGoalkeeper(0,BALL);
      task = kick;
      hasKicked = true;
      taskTimer = millis();
      
    }
  }
}

void Controller::goToObject(int object, int signature) {
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  if (true==checkVisibility(GOAL) && true==checkVisibility(BALL)){
    getPixelDistance();
  }
  if(pixelDistance<100){
    task=score;
  }
  else if(xPos<120){
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

void Controller::goToObjectGoalkeeper(int object, int signature) {
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
  getSigVariables();
  uint16_t xPosBall = pixy.blocks[ballVar].x;
  uint16_t xPosObject = pixy.blocks[opponentVar].x;
  //ballVisible = checkVisibility(BALL);
  if(true==checkVisibility(BALL)){
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

//This function updates the variables for which spot 
//in the blocks array the specific signatures is in
void Controller::getSigVariables(){
  //Loops through each object in blocks,
  //blocksCount is the size of the array
  for(int i=0;i<blocksCount;i++){
    //if the object in blocks equals to a specific signature
    //the designated variable for that signature is set
    if(pixy.blocks[i].signature==BALL){
      ballVar = i;
    }
    else if(pixy.blocks[i].signature==OPPONENT){
      opponentVar = i;
    }
    else if(pixy.blocks[i].signature==GOAL){
      goalVar = i;
    }
  }
}

//Goes through the blocks function to determine
//if an object is visible
boolean Controller::checkVisibility(int signature){
 for(int i=0;i<blocksCount;i++){
  //if the signature for ball is found 
  //then the ball is visible
  if(pixy.blocks[i].signature==signature){
    return true;
  }
 }
 return false;
}

//Centers the ball in pixys field of vision
void Controller::centerBall(){ 
  getSigVariables();
  uint16_t xPosBall = pixy.blocks[ballVar].x;
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

//adjust and centers both ball and goal to get
//into "goal scoring position".
//sets task to kick when position is right
void Controller::scoreGoal(){
  getSigVariables();
  uint16_t xPosBall = pixy.blocks[ballVar].x;
  uint16_t xPosGoal = pixy.blocks[goalVar].x;
  uint16_t width = pixy.blocks[ballVar].width;
  //when the ball is to the right of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go right
  if(xPosBall>xPosGoal){
    motor.steer(right,20);
  }
  //when the ball is to the left of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go left
  else if(xPosBall<xPosGoal){
    motor.steer(left,20);
  }
  //When the goal is far out on either of the edges in pixys field of vision
  //while the ball is not, THEN move forward
  else if(xPosGoal<50 && xPosGoal>260 && xPosBall>50 && xPosBall<260){
    motor.steer(forward,20);
  }
  else if(xPosGoal<50 && xPosGoal>260 && xPosBall<50 && xPosBall>260){
    task=center;
  }
  else if(xPosBall>120 && xPosBall<200 && width > 10 && width < 110){
    motor.steer(forward,20);
  }
  else if(xPosBall>120 && xPosBall<200 && width>110){
    task=kick;
    taskTimer = millis();
  }
}

//gets distance between xPosBall and xPosGoal
//to get the distance between the goal and ball in pixels
void Controller::getPixelDistance(){
  getSigVariables();
  uint16_t xPosBall = pixy.blocks[ballVar].x;
  uint16_t xPosGoal = pixy.blocks[goalVar].x;
  uint16_t pixelDistance = abs(xPosBall-xPosGoal);
}

void Controller::checkSurroundings(){
  for(int i=0;i<blocksCount;i++){
    if(pixy.blocks[i].signature==OPPONENT){
      if(pixy.blocks[i].width>80){
        avoidObject();
      }
    }
  }
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

