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

  //If pixy sees an object, update the stored information
  //about the objects
  if (blocksCount || updateTimer > 25){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);

    if(isVisible(BALL)){
      lastXPosBall=pixy.blocks[objectIndex[BALL]].x;
    }
    if(isVisible(GOAL2)){
      lastXPosGoal=pixy.blocks[objectIndex[GOAL2]].x;
    }
    if(!GOALKEEPER){
      task = makeDecision(blocksCount, task);
    }
    else{
      task = makeDecisionGoalkeeper(blocksCount, task);
    }
  }
  
  //Call different functions depending on current task
  if(task == search) {
    findBall();
  }
  else if(task == goToBall) {
    goToObject(BALL);
  }
  else if(task == goToGoal){
    goToObject(GOAL1);
  }
  else if(task == kick) {
    kickBall();
  }
  else if(task == avoid) {
    avoidObject();
  }
  else if(task == encMove) {
    if(getNewMove == true)
      setupEncoderMove();
    else
      encoderMove();
  }
  else if(task == stay){
    motor.steer(stop,0);
  }
  else {
    motor.steer(stop,0);
  }
}

//Decide what the next task should be (offensive player)
Task Controller::makeDecision(uint16_t actualBlocks, Task lastTask) {
  Task nextTask = search;

  //Set task depending on what pixy sees
  if(actualBlocks) {
    if(isVisible(BALL)) {
      if(isVisible(GOAL2)){
        int16_t diff = getXposDiff(BALL, GOAL2);
        if(diff < 30){
          nextTask = goToBall;
        }
        else{
          if(lastTask != encMove){
            calculateTrajectory();
          }
          nextTask = encMove;
        }
      }
      else if(isVisible(GOAL1)) {
        if(lastTask != encMove){
            calculateGoHome();
          }
        nextTask = encMove;
      }
      else{
        nextTask = goToBall;
      }
    }
    else if(!isVisible(BALL)) {
      if(isVisible(GOAL2) && objectDistance[GOAL2] < 80){
        if(lastTask != encMove){
            calculateGoHome();
          }
        nextTask = encMove;
      }
    }

    if(isVisible(PLAYER1)){
      if(objectDistance[PLAYER1] < 40) {
        nextTask = avoid;
      }
      else if(isVisible(BALL)){
        if(distanceBetween(BALL,PLAYER1) < 100 && objectDistance[BALL] < 200){
            nextTask = stay;
        } 
      }
    }
    if(isVisible(PLAYER2) && objectDistance[PLAYER2] < 40){
      nextTask = avoid;
    }
    if(isVisible(GOAL1) && objectDistance[GOAL1] < 80) {
      nextTask = avoid;
    }
    if(isVisible(EDGE) && objectDistance[EDGE] < 30){
        nextTask = avoid;
    }
  }

  //Set task depending on lastTask and nextTask

  //kick and encMove can only be interrupted by avoid
  //or by the functions themselves
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


//Decide what the next task should be (goalkeeper)
Task Controller::makeDecisionGoalkeeper(uint16_t actualBlocks, Task lastTask) {
  Task nextTask = search;

  //Set task depending on what pixy sees
  if(actualBlocks) {
    // if(isVisible(GOAL1) && objectDistance[GOAL1] < 30){
    //   nextTask = search;
    // }
    if(isVisible(GOAL1) && objectDistance[GOAL1] < 100 && lastTask == goToGoal){
      clearInstructionQueue();
      MoveInstruction m = MoveInstruction(line,70,0,20);
      moveInstructionQueue.push(m);
      nextTask = encMove;
    }

    if(isVisible(BALL)) {
      if(objectDistance[BALL] < 80){
        nextTask = goToBall;
      }
      else{
        nextTask = stay;
      }
    }
  }

  //Set task depending on lastTask

  if(kicked || lastTask == goToGoal){
    nextTask = goToGoal;
    kicked = false;
  }

  //kick and encMove can only be interrupted by avoid
  //or by the functions themselves
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
  
  if(xPos<130){
    motor.steer(left,30);
    motor.steer(forward,5);
  }
  else if(xPos>190){
    motor.steer(right,30);
    motor.steer(forward,5);
  }
  else if(objectDistance[object] >= 30 && objectDistance[object] < 600){
    motor.steer(forward,30);
  }
  else if(objectDistance[object] < 30){
    if(object == BALL) {
      task=kick;
      taskTimer = millis(); //do not remove
    }
    else {
      task = stay;
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
    task = search;
    kicked = true;
  }
}


void Controller::avoidObject() {
  //Check which object is closest. Disregard ball.
  int closestObject = 1;

  for(int i=1;i<6;i++){
    if(objectDistance[closestObject] == -1){
      closestObject = i;
    }
    else if(objectDistance[i] != -1){
      if(objectDistance[i] <= objectDistance[closestObject]){
        closestObject = i;
      }
    }
  }
  
  uint16_t xBall = lastXPosBall;
  const double pi = 3.1415;
  
  int16_t xObject = pixy.blocks[objectIndex[closestObject]].x;
  int16_t xDiffObj = xObject - 160;
  int16_t xDiffBallandObj = xBall - xObject;

  double angle = 75; //(degrees)
  double radius = 30;
  double turnAngle = 30;

  if(isVisible(BALL)) {
    if(xDiffBallandObj >= 0) { //Ball to the right of object
      angle = angle + (xDiffObj/320.0)*75.0;
      radius = -radius; //Turn to the left
    }
    else if(xDiffBallandObj < 0) { //Ball to the left of object
      angle = -angle + (xDiffObj/320.0)*75.0;
    }
  }
  else {
    if(xBall >= 160) { //Ball was last seen to the right
      angle = 120;
      radius = -radius;
    }
    else if(xBall < 160) { //Ball was last seen to the left
      angle = - 120;
    }
  }

  double length = abs(radius)*(turnAngle*pi/180.0);

  clearInstructionQueue();

  MoveInstruction m = MoveInstruction(spin,angle,25);
  moveInstructionQueue.push(m);
  m = MoveInstruction(line,length,radius,20);
  moveInstructionQueue.push(m);
  task = encMove;
}

//Turn towards where the ball is
void Controller::findBall(){ 
 if(lastXPosBall<120){
  motor.steer(left,30);
 }
 else if(lastXPosBall>200){
  motor.steer(right,30);
 }
 else {
  motor.steer(stop);
 }
}

void Controller::findGoal(){ 
 if(lastXPosGoal<120){
  motor.steer(left,30);
 }
 else if(lastXPosGoal>200){
  motor.steer(right,30);
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
  bool moveFinished = true;

  if(!moveInstructionQueue.isEmpty()){
    moveFinished = false;

    MoveInstruction moveIns = moveInstructionQueue.peek();
    float circumference = 30.9211; //the wheel's circumference in cm
    int32_t pulses = 1856; //number of pulses per revolation (928 for each wheel)
    
    float startValue = startLeftvalue+startRightvalue;
    float position = motor.getWheelsPosition() - startValue;
    float distance = -position*circumference/(pulses*2); //position is negative when driving forward and positive when driving backwards


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
        if(pixy.blocks[i].width >= pixy.blocks[i].height){
          objectDistance[BALL] = distanceToObject(pixy.blocks[i].width,REAL_WIDTH_BALL,false);
        }
        else {
          objectDistance[BALL] = distanceToObject(pixy.blocks[i].height,REAL_WIDTH_BALL,true);
        }
      }
    }
    else if(signature == SIGN_GOAL){ //Opponent goal
      int16_t angle = pixy.blocks[i].angle;
      if(-180<angle && angle<0){
        if(objectIndex[GOAL1] == -1){
          objectIndex[GOAL1] = i;
          objectDistance[GOAL1] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_GOAL,true);
        }
      }
      else if(0<angle && angle<180) {
        if(objectIndex[GOAL2] == -1){
          objectIndex[GOAL2] = i;
          objectDistance[GOAL2] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_GOAL,true);
        }
      }
    }
    else if(signature = SIGN_PLAYER) {  //Opponent player
      int16_t angle = pixy.blocks[i].angle;
      if(-180<angle && angle<0){
        if(objectIndex[PLAYER1] == -1){
          objectIndex[PLAYER1] = i;
          objectDistance[PLAYER1] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_PLAYER,true);
        }
      }
      else if(0<angle && angle<180) {
        if(objectIndex[PLAYER2] == -1){
          objectIndex[PLAYER2] = i;
          objectDistance[PLAYER2] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_PLAYER,true);
        }
      }
    }
    else if(signature == SIGN_EDGE) {
      if(objectIndex[EDGE] == -1){
        objectIndex[EDGE] = i;
        objectDistance[EDGE] = distanceToObject(pixy.blocks[i].height,REAL_HEIGHT_EDGE,true);
      }
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
  double phi = pi - asin(c*sin(abs(alpha))/d); //sinussatsen, går att välja alltid trubbig vinkel

  double theta = 2.0*pi - 2.0*phi;
  double r = b/(2.0*sin(theta/2.0));
  double l = r * theta;

  double e = 30.0; // how far away from the ball the robot should be aligned shoting at the goal
  double b2 = sqrt(sq(b) + sq(e) + 2.0*b*e*cos(phi)); // cos(pi-phi) = -cos(phi)
  double phi2 = pi - asin(b*sin(phi)/b2); // sin(pi-phi) = sin(phi)
  double r2 = b2 / (2.0*sin(phi2));
  double l2 = r2 * 2.0 * (pi - phi2)*0.7;
  double gamma = phi - phi2;
  double theta2 = 2.0*pi - 2.0*phi2; 

  if (alpha > 0) 
    beta = beta + gamma;
  else if (alpha < 0)
    beta = beta - gamma;

  l2 = constrain(l2,10,200);
  r2 = constrain(r2,15,400);

  MoveInstruction m;
  if(alpha > 0 && beta < 0) { //Ball to the right of goal and left of robot
    double angle = theta2/2.0 - abs(beta);
    angle = angle*0.8;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l2, -r2,30);
    moveInstructionQueue.push(m);
  }
  else if(alpha < 0 && beta > 0) { //Ball to the left of goal and right of robot
    double angle = -(theta2/2.0 - abs(beta));
    angle = angle*0.8;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l2, r2,30);
    moveInstructionQueue.push(m);
  }
  else if(alpha > 0 && beta > 0) { //Ball to the right of goal and right of robot
    double angle = theta2/2.0 + abs(beta);
    angle = angle*0.8;
    m = MoveInstruction(spin,30, angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l2, -r2,30);
    moveInstructionQueue.push(m);
  }
  else if(alpha < 0 && beta < 0) { //Ball to the left of goal and left of robot
    double angle = -(theta2/2.0 + abs(beta));
    angle = angle*0.8;
    m = MoveInstruction(spin,30,angle*(180/pi));
    moveInstructionQueue.push(m); 
    
    m = MoveInstruction(line, l2, r2,30);
    moveInstructionQueue.push(m);
  }
}

void Controller::calculateGoHome(){
  clearInstructionQueue();

  double angleDiff = 180;
  float distance = 150;

  if(isVisible(GOAL1)) {
    int16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
    angleDiff = ((xPosGoal-160)/320.0)*75.0*(3.1415/180.0);
  }
  if(isVisible(BALL)){
    distance = objectDistance[BALL] + 100;
  }

  distance = constrain(distance,100,300);

  MoveInstruction m = MoveInstruction(spin,angleDiff*0.8,25);
  moveInstructionQueue.push(m);

  m = MoveInstruction(line,distance,0,25);
  moveInstructionQueue.push(m);
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

void Controller::resetValues(){
  task = search;
  centered = false;
  kicked = true;
  getNewMove = true;
  clearInstructionQueue();
  startLeftvalue = 0;
  startRightvalue = 0;
  targetTurningDistance = 0;
  taskTimer = 0;
  pixyTimer = 0;
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

