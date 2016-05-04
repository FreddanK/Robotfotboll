/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
*/

#include "Controller.h"

#include <Arduino.h>

#include "Motor.h"
#include "Configuration.h"


void Controller::doTask() {
  
  uint16_t blocksCount = pixy.getBlocks();

  uint32_t updateTimer = millis() - pixyTimer;
  
  if (blocksCount || updateTimer > 25){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
    if(isVisible(BALL)){
      lastXPosBall=pixy.blocks[objectIndex[BALL]].x;
    }
    if(isVisible(GOAL2)){
      lastXPosGoal=pixy.blocks[objectIndex[GOAL2]].x;
    }
    task = makeDecision(blocksCount, task);
  }
  
  if(task == search) {
    findBall();
  }
  else if(task == goToBall) {
    goToObject(BALL);
  }
  else if(task == kick) {
    kickBall();
  }
  else if(task == avoid) {
    avoidObject();
  }
  else if(task == score){
    if(blocksCount || updateTimer<25) {
      if(isVisible(BALL) && isVisible(GOAL2)){
        scoreGoal();
      }
//      else if(!isVisible(BALL)){
//        findBall();
//      }
//      else if(!isVisible(GOAL2)){
//        findGoal();
//      }
      else{
        motor.steer(stop);
      }
    }
    else{
      motor.steer(stop);
    }
  }
  else if(task == center){
    centerBall(); 
  }
  else if(task == encMove) {
    if(getNewMove == true)
      setupEncoderMove();
    else
      encoderMove();
  }
  else {
    motor.steer(stop,0);
  }
}

Task Controller::makeDecision(uint16_t actualBlocks, Task lastTask) {
  Task nextTask = search;

  //Set task depending on what pixy sees
  if(actualBlocks) {
    if(isVisible(EDGE)){
      if(objectDistance[EDGE] < 20) {
        nextTask = avoid;
      }
    }
    else if(isVisible(BALL) && isVisible(GOAL2)) {

      if(lastTask != encMove)
        calculateTrajectory();
      nextTask = encMove;
    }
    else if(isVisible(BALL)) {
      nextTask = goToBall;
    }

    if(isVisible(BALL) && objectDistance[BALL] < 30)
      nextTask = kick;
  }

  //Set task depending on lastTask and nextTask
  if(lastTask == kick || lastTask == encMove){
    if(nextTask != avoid){
      nextTask = lastTask;
    }
  }

  //Return next task, reset taskTimer if task has been changed
  if(nextTask != lastTask){
    taskTimer = millis();
  }
  return nextTask;
}

void Controller::goToObject(int object) {
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
  else if(width>110){
    task=kick;
  }
}

//adjust and centers both ball and goal to get
//into "goal scoring position".
//sets task to kick when position is right
void Controller::scoreGoal(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL2]].x;
  uint16_t width = pixy.blocks[objectIndex[BALL]].width;
  //if ball is centered but still far. move forward
  if(xPosBall>120 && xPosBall<200 && width > 10 && width < 110 && centered){
    motor.steer(forward,20);
  }
  //if ball is centered and close, kick ball
  else if(xPosBall>120 && xPosBall<200 && width>110 && centered){
    task=kick;
    taskTimer = millis();
  }
  //When the goal is far out on either of the edges in pixys field of vision
  //while the ball is not, THEN move forward
  else if((xPosGoal<70 && !centered) || (xPosGoal>240 && !centered)){
    motor.steer(forward,20);
    //if both ball and goal is far to the left or far to the right, then center ball
    if(xPosBall<10 || xPosBall>310){
      task=center;
    }
  }
  //when the ball is to the right of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go right
  else if(xPosBall>xPosGoal && !centered){
    motor.steer(right,20);
  }
  //when the ball is to the left of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go left
  else if(xPosBall<xPosGoal && !centered){
    motor.steer(left,20);
  }
  else{
    motor.steer(stop);
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
    centered=false;
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
  if(isVisible(BALL)){
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
    centered=true;
  }
}

void Controller::findBall(){ 
 if(lastXPosBall<120){
  motor.steer(left,20);
 }
 else if(lastXPosBall>200){
  motor.steer(right,20);
 }
 else {
  motor.steer(stop);
 }
}

void Controller::findGoal(){ 
 if(lastXPosGoal<120){
  motor.steer(left,20);
 }
 else if(lastXPosGoal>200){
  motor.steer(right,20);
 }
 else {
  motor.steer(stop);
 }
}

void Controller::setupEncoderMove() {
  if(!moveInstructionQueue.isEmpty()) {
    MoveInstruction moveIns = moveInstructionQueue.peek();
    startLeftvalue = motor.readRightEncoder();
    startRightvalue = motor.readLeftEncoder();
    targetTurningDistance = (moveIns.degree/360)*2*3.141592654*10;
    getNewMove = false;
  }
  else{
    motor.turningRadius = 0;
    task = search;
  }
  
}

void Controller::encoderMove(){
  MoveInstruction moveIns = moveInstructionQueue.peek();
  float circumference = 30.9211; //the wheel's circumference in cm
  int32_t pulses = 1856; //number of pulses per revolation (928 for each wheel)
  
  float startValue = startLeftvalue+startRightvalue;
  float position = motor.getWheelsPosition() - startValue;
  float distance = -position*circumference/(pulses*2); //position is negative when driving forward and positive when driving backwards
  
  bool moveFinished = false;

  if(moveIns.moveType == line) {
    if (moveIns.distance >= 0) { //Drive forwards
      if (distance < moveIns.distance){
        motor.steer(stop);   //This is only to make sure turningOffset is 0
        motor.steer(forward,moveIns.speed);
        motor.turningRadius = moveIns.radius;
      }
      else if(distance >= moveIns.distance){
        moveFinished = true;
      }
    }
    else if(moveIns.distance < 0) { //Drive backwards
      if (distance > moveIns.distance){
        motor.steer(stop);   //This is only to make sure turningOffset is 0
        motor.steer(backward,moveIns.speed);
        motor.turningRadius = moveIns.radius;
      }
      else if(distance <= moveIns.distance){
        moveFinished = true;
      }
    }
  }
  else if(moveIns.moveType == spin) {
    int32_t rightwheelpos = motor.readRightEncoder() - startLeftvalue;
    int32_t leftwheelpos = motor.readLeftEncoder() - startRightvalue;
    
    float leftdistance = -leftwheelpos*circumference/(pulses*2); //the position is negative when driving forwards and positive when driving backwards
    float rightdistance = -rightwheelpos*circumference/(pulses*2);

    float diff = leftdistance - rightdistance;

    if(targetTurningDistance > 0) { //spin right
      if(diff < targetTurningDistance){
        motor.steer(right,moveIns.speed);
      }
      else if(diff >= targetTurningDistance) {
        moveFinished = true;
      }
    }
    else if(targetTurningDistance < 0) { //spin left
      if(diff > targetTurningDistance) {
        motor.steer(left,moveIns.speed);
      }
      else if (diff <= targetTurningDistance) {
        moveFinished = true;
      }
    }
    else {
      moveFinished = true;
    }
  }
  if(moveFinished){
    motor.steer(stop);
    moveInstructionQueue.pop();
    getNewMove = true;
  }
}

float Controller::spinCheck() {
  float circumference = 30.9211; //the wheel's circumference in cm
  int32_t pulses = 1856; //number of pulses per revolation (928 for each wheel this time instead of 1826 for both)
  
  int32_t rightwheelpos = motor.readRightEncoder() - startRightvalue; // To make sure you have a start value
  int32_t leftwheelpos = motor.readLeftEncoder() - startLeftvalue;
  
  float leftdistance = -leftwheelpos*circumference/(pulses*2); //the position is negative when driving forwards and positive when driving backwards
  float rightdistance = -rightwheelpos*circumference/(pulses*2);

  float skillnadleft = rightdistance - leftdistance;
  float skillnadright = leftdistance - rightdistance;
 

  float turnedDegreeRight = skillnadleft * 18 / 3.141592654; // 18 från 180 grader delat på radien 10 cm
  //int32_t turnedDegreeRight = motor.readLeftEncoder();
  float turnedDegreeLeft = skillnadright * 18 / 3.141592654;
  
  float result = turnedDegreeRight;
  return result;
}

void Controller::clearInstructionQueue(){
  while(!moveInstructionQueue.isEmpty()) //Empty the queue before filling it
      moveInstructionQueue.pop();
  motor.turningRadius = 0;
}

void Controller::getSignatureIndexes(uint16_t actualBlocks) {
  
  for(int i=0; i<6;i++){
    objectIndex[i] = -1;
    objectDistance[i] = -1;
  }
    
  for(int i=0; i<actualBlocks;i++) {
    uint16_t signature = pixy.blocks[i].signature;
    if(signature == SIGN_BALL) { //Ball
      if(objectIndex[BALL] == -1) { //only store information about the closest object
        objectIndex[BALL] = i;
        objectDistance[BALL] = distanceToObject(pixy.blocks[i].width,REAL_WIDTH_BALL,false);
      }
    }
    else if(signature == SIGN_GOAL){ //Opponent goal
      uint16_t angle = pixy.blocks[i].angle; //TODO, angle doesen't seem to update as often as the rest of the parameters from the pixy, so it is not used right now
      if(objectIndex[GOAL2] == -1) { //only store information about the closest object
        objectIndex[GOAL2] = i;
        objectDistance[GOAL2] = distanceToObject(pixy.blocks[i].width,REAL_WIDTH_GOAL,false);
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
    else if(signature = SIGN_PLAYER) {  //Opponent player
      uint16_t angle = pixy.blocks[i].angle; //TODO, angle doesen't seem to update as often as the rest of the parameters from the pixy, so it is not used right now
      if(objectIndex[PLAYER2] == -1) { //only store information about the closest object
        objectIndex[PLAYER2] = i;
        objectDistance[PLAYER2] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_PLAYER,true);
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

bool Controller::isVisible(int object) {
  if(objectIndex[object]!=-1){
    return true;
  }
  else return false;
}

void Controller::calculateTrajectory(){
  clearInstructionQueue();

  int16_t xPosBALL= pixy.blocks[objectIndex[BALL]].x;
  int16_t xPosGOAL= pixy.blocks[objectIndex[GOAL2]].x;

  const double pi = 3.1415;

  double alpha = ((xPosBALL-xPosGOAL)/320.0)*75.0*(pi/180.0);
  double beta = ((xPosBALL-160)/320.0)*75.0*(pi/180.0);
  double d = distanceBetween(BALL, GOAL2);
  double b = objectDistance[BALL];
  double c = objectDistance[GOAL2];
  //double phi = acos((sq(d)+sq(b)-sq(c))/(2.0*d*b)); //cosinussatsen
  double phi = pi - asin(c*sin(abs(alpha))/d); //sinussatsen

  double theta = 2*pi - 2*phi;
  double r = b/(2*sin(theta/2.0));
  double l = r * theta;

  Serial.print(" Alpha:");
  Serial.print(alpha*(180.0/3.14));
  Serial.print(" Beta:");
  Serial.print(beta*(180/pi));
  Serial.print(" Phi:");
  Serial.print(phi*(180.0/3.14));
  // Serial.print(" Phi2:");
  // Serial.println(phi2*(180.0/3.14));

  Serial.print(" Theta:");
  Serial.print(theta*(180.0/3.14));
  Serial.print(" r:");
  Serial.print(r);
  Serial.print(" l:");
  Serial.println(l);

  MoveInstruction m;
  if(alpha > 0 && beta < 0) { //Ball to the right of goal and left of robot
    double angle = theta/2.0 - abs(beta);
    angle = angle*0.7;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l, -r,25);
    moveInstructionQueue.push(m);
  }
  else if(alpha < 0 && beta > 0) { //Ball to the left of goal and right of robot
    double angle = -(theta/2.0 - abs(beta));
    angle = angle*0.7;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l, r,25);
    moveInstructionQueue.push(m);
  }
  else if(alpha > 0 && beta > 0) { //Ball to the right of goal and right of robot
    double angle = theta/2.0 + abs(beta);
    angle = angle*0.7;
    m = MoveInstruction(spin,30, angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l, -r,25);
    moveInstructionQueue.push(m);
  }
  else if(alpha < 0 && beta < 0) { //Ball to the left of goal and left of robot
    double angle = -(theta/2.0 + abs(beta));
    angle = angle*0.7;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l, r,25);
    moveInstructionQueue.push(m);
  }
}

void Controller::calculateTrajectory2(){
  while(!moveInstructionQueue.isEmpty()) //Empty the queue before filling it
    moveInstructionQueue.pop();

  int16_t xPosGOAL= pixy.blocks[objectIndex[GOAL2]].x;
  int16_t xPosBALL= pixy.blocks[objectIndex[BALL]].x;

  
  float alpha = ((xPosBALL-xPosGOAL)/320.0)*75.0*(3.1415/180.0);
  float d = distanceBetween(BALL, GOAL2);
  float b = objectDistance[BALL];
  float c = objectDistance[GOAL2];
  float phi = acos((sq(d)+sq(b)-sq(c))/(2.0*d*b));
 

  MoveInstruction m;
  if(alpha > 0) {
    m = MoveInstruction(spin,30,phi*(180/3.1415));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, abs(3.1415*b*sin(phi)/2.0),-b*sin(phi)/2.0,25);
    moveInstructionQueue.push(m);
  }
  else if(alpha < 0) {
    m = MoveInstruction(spin,30,-phi*(180/3.1415));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, abs(3.1415*b*sin(phi)/2.0),b*sin(phi)/2.0,25);
    moveInstructionQueue.push(m);
  }
}


//Takes the object's size in pixels and converts it to distance in cm.
//Arguments: the object's size (pixels), real size of the object (cm) and an extra argument
//that is true if you want to measure the distance based on the heigth instead of the width.
double Controller::distanceToObject(int object_size, float real_size, bool measure_height) {
  double focal_length = 0.28; //cm
  double image_width = 320.0; //pixels
  double image_height = 200.0; //pixels
  double sensor_width = 0.3888; //cm
  double sensor_height = 0.243; //cm
  if(measure_height)
    return (focal_length*real_size*image_height)/(object_size*sensor_height);
  else
    return (focal_length*real_size*image_width)/(object_size*sensor_width);
  
}

//Calculates the distance between two objects
//Example of use: float var = distanceBetween(BALL,PLAYER2);
double Controller::distanceBetween(int16_t object1, int16_t object2) {
  int16_t xPos1 = pixy.blocks[objectIndex[object1]].x;
  int16_t xPos2 = pixy.blocks[objectIndex[object2]].x;
  double d1 = objectDistance[object1];
  double d2 = objectDistance[object2];
  double image_width = 320.0; //pixels
  double fov = 75.0*3.141592654/180.0; //field of view, degrees

  return sqrt(sq(d1)+sq(d2)-2.0*d1*d2*cos((abs(xPos1-xPos2)/image_width)*fov)); 
}

//Calculates and returns difference in x-position for two objects
int16_t Controller::getXposDiff(int16_t object1, int16_t object2){
  int16_t xPos1 = pixy.blocks[objectIndex[object1]].x;
  int16_t xPos2 = pixy.blocks[objectIndex[object2]].x;
  return abs(xPos1-xPos2);
}


//Servo fuctions below

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

