#include <Pixy.h>
#include <stdint.h>

const bool GOALKEEPER = true;

//These constants holds the indexes for each object in
//the arrays 'objectIndex' and 'objectDistance'.
const int BALL = 0;
const int GOAL1 = 1;
const int GOAL2 = 2; 
const int PLAYER1 = 3; 
const int PLAYER2 = 4; 
const int EDGE = 5;

//These constants holds the actual signatures that Pixy was taught.
const int SIGN_BALL = 1;
const int SIGN_GOAL = 045;
const int SIGN_PLAYER = 023;
const int SIGN_EDGE = 067;

//These constants holds the real size in cm for all objects.
const int REAL_WIDTH_BALL = 14;
const int REAL_HEIGHT_GOAL = 47;
const int REAL_HEIGHT_PLAYER = 15;
const int REAL_HEIGHT_EDGE = 3;

Pixy pixy{};

enum Task {
  search,
  kick,
  avoid,
  goTo,
  wait,
  score,
  center,
};

uint32_t pixyTimer = millis();

Task task = search;

int16_t objectIndex[6];
float objectDistance[6];

void setup() {
  Serial.begin(115200);
  Serial.println("Strategy test started");
  pixy.init();
  
}

void loop() {
  doTask();

}
uint16_t actualBlocks =0;

void doTask() {
  uint16_t blocksCount = pixy.getBlocks();

  uint32_t updateTimer = millis() - pixyTimer;
  
  if (blocksCount || updateTimer > 25){
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
    actualBlocks = blocksCount;
  }
  if(actualBlocks){
    if(isVisible(BALL)) {
      Serial.print("Ball ");
      Serial.print(objectDistance[BALL]);
    } 
    if(isVisible(GOAL1)){
      Serial.print(", Goal 1 ");
      Serial.print(objectDistance[GOAL1]);
    }
    if(isVisible(GOAL2)){
      Serial.print(", Goal 2 ");
      Serial.print(objectDistance[GOAL2]);
    }
    if(isVisible(PLAYER1)){
      Serial.print(", Player 1 ");
      Serial.print(objectDistance[PLAYER1]);
    }
    if(isVisible(PLAYER2)){
      Serial.print(", Player 2 ");
      Serial.print(objectDistance[PLAYER2]);
    }
    if(isVisible(EDGE)){
      Serial.print(", Edge ");
      Serial.print(objectDistance[EDGE]);
    } 
    Serial.println();
  }
  else {
      Serial.println("No object");
   }
 }

void getSignatureIndexes(uint16_t actualBlocks) {
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

bool isVisible(int object) {
  if(objectIndex[object]!=-1){
    return true;
  }
  else return false;
}
//Takes the object's size in pixels and converts it to distance in cm.
//Arguments: the object's size (pixels), real size of the object (cm) and an extra argument
//that is true if you want to measure the distance based on the heigth instead of the width.
float distanceToObject(int object_size, float real_size, bool measure_height) {
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

//Calculate the distance between two objects
//Example of use: float var = distanceBetween(BALL,PLAYER2);
float distanceBetween(int16_t object1, int16_t object2) {
  int16_t xPos1 = pixy.blocks[objectIndex[object1]].x;
  int16_t xPos2 = pixy.blocks[objectIndex[object2]].x;
  double d1 = objectDistance[object1];
  double d2 = objectDistance[object2];
  double image_width = 320.0; //pixels
  double fov = 75.0*3.141592654/180.0; //field of view, degrees

  return sqrt(sq(d1)+sq(d2)-2.0*d1*d2*cos((abs(xPos1-xPos2)/image_width)*fov)); 
}

