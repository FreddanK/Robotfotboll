#include <Pixy.h>
#include <stdint.h>

//Object, Index
#define BALL 0  //Signature 1 (Ball)
#define GOAL1 1 //Signature 45(octal) 90 < theta <= 180 and -180 <= theta <-90 (Own goal)
#define GOAL2 2 //Signature 45(octal) -90 < theta < 90 (Opponents goal, yellow(left) blue(right))
#define PLAYER1 3 //Signature 23(octal) -180 < theta < 0  (Team member)
#define PLAYER2 4 //Signature 23(octal) 0 < theta < 180 (Opponent, purple(top) green(bottom)
#define EDGE 5 //NA

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

uint32_t pixelDistance=100;
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

void doTask() {
  uint16_t blocksCount = pixy.getBlocks();

  uint32_t updateTimer = millis()-pixyTimer;

  if(blocksCount) {
    pixyTimer = millis();
    getSignatureIndexes(blocksCount);
  }
  
  if(task == search) {
    if(blocksCount || updateTimer<25) { 
      //Serial.print("Object count: ");
      //Serial.println(actualBlocks);

      //makeDecision(); //TODO implement this

      if(blocksCount || updateTimer<25) {
        goToObject(0,1);
      }
      /*
      if(objectIndex[BALL] > -1) {
        Serial.print("Ball ");
        Serial.print(objectDistance[BALL]);
      } 
      if(objectIndex[GOAL1] > -1){
        Serial.print(", Goal 1 ");
        Serial.print(objectDistance[GOAL1]);
      }
      if(objectIndex[GOAL2] > -1){
        Serial.print(", Goal 2 ");
        Serial.print(objectDistance[GOAL2]);
      }
      if(objectIndex[PLAYER1] > -1){
        Serial.print(", Player 1 ");
        Serial.print(objectDistance[PLAYER1]);
      }
      if(objectIndex[PLAYER2] > -1){
        Serial.print(", Player 2 ");
        Serial.print(objectDistance[PLAYER2]);
      }
      if(objectIndex[EDGE] > -1){
        Serial.print(", Edge ");
        Serial.print(objectDistance[EDGE]);
      } 
      Serial.println();
    }
   else {
      Serial.println("No object");
   }*/
    }
  }
  else if(task == kick) {
    Serial.println("kick");
  }
  else if(task=score){
    scoreGoal();
  }
  else if(task=center){
    Serial.println("center");
  }
}

void goToObject(int object, int signature) {
  uint16_t xPos = pixy.blocks[object].x;
  uint16_t width = pixy.blocks[object].width;

  //if both goal and ball can be seen at the same time
  //check the distance between them to determine which function to call
  if(checkVisibility(3)){
    Serial.println("Goal!");
  }
  if(checkVisibility(1)){
    Serial.println("Ball!");
  }
  if (checkVisibility(3) && checkVisibility(1)){
    Serial.println("Ball and goal");
    getPixelDistance();
    Serial.println(pixelDistance);
  //if the distance between goal and ball is small
  //start function scoreGoal
  }
  if(pixelDistance<100){
    task=score;
  }
}

//adjust and centers both ball and goal to get
//into "goal scoring position".
//sets task to kick when position is right
void scoreGoal(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
  uint16_t width = pixy.blocks[objectIndex[BALL]].width;
  //When the goal is far out on either of the edges in pixys field of vision
  //while the ball is not, THEN move forward
  if(xPosGoal<50 && xPosGoal>260 && xPosBall>50 && xPosBall<260){
    Serial.println("forward");
  }
  //when the ball is to the right of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go right
  else if(xPosBall>xPosGoal){
    Serial.println("right");
  }
  //when the ball is to the left of the goal but the goal is not dissapearing from either side
  //of the field of vision. THEN go left
  else if(xPosBall<xPosGoal){
    Serial.println("left");
  }
  //if both ball and goal is far to the left or far to the right, then center ball
  else if((xPosGoal<50 && xPosBall<50) || (xPosGoal>260 && xPosBall>260)){
    Serial.println("center");
  }
  //if ball is centered but still far. move forward
  else if(xPosBall>120 && xPosBall<200 && width > 10 && width < 110){
    Serial.println("forward2");
  }
  //if ball is centered and close, kick ball
  else if(xPosBall>120 && xPosBall<200 && width>110){
    Serial.println("kick");
  }
}

bool checkVisibility(int signature){
  uint16_t sig[7];
  uint16_t blocksCount = pixy.getBlocks();
  for(int j=0;j<blocksCount;j++){
    sig[j] = pixy.blocks[j].signature;

    //if the signature for ball is found 
    //then the ball is visible
    if(pixy.blocks[j].signature=signature){
      return true;
    }
  }
 return false;
}

void getPixelDistance(){
  uint16_t xPosBall = pixy.blocks[objectIndex[BALL]].x;
  uint16_t xPosGoal = pixy.blocks[objectIndex[GOAL1]].x;
  pixelDistance = abs(xPosBall-xPosGoal);
}

void getSignatureIndexes(uint16_t actualBlocks) {
  
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
float distanceToObject(int object_size, float real_size, bool measure_height) {
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

