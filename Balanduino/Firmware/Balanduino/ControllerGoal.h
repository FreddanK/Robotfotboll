#ifndef _controllerGoal_h_
#define _controllerGoal_h_

#include <Pixy.h>
#include <stdint.h>

#include "Motor.h";

enum Task {
  search,
  kick,
  avoid,
  findgoal,
  gotogoal,
  gotoball,
  wait,
  center,
};

class ControllerGoal {
private:
  Motor& motor;
  Pixy& pixy;
  
  uint16_t lastXPosBall;
  uint16_t lastXPosGoal;
  int16_t objectIndex[6];
  float objectDistance[6];

  uint16_t blocksCount = 0;

  Task task = search;
  uint32_t taskTimer = 0;
  uint32_t pixyTimer = 0;

  int32_t startLeftvalue = 0;
  int32_t startRightvalue = 0;
  int32_t startValue = 0;

  float targetDistance = 0;
  float radius = 0;
  float speed = 0;
  float targetTurningDistance = 0;
  float rate = 0;


public:
  ControllerGoal(Motor& m, Pixy& p) : motor(m), pixy(p) {}

  float distanceToObject(int object_size, float real_size, bool measure_height);
  float distanceBetween(int16_t object1, int16_t object2); 
  void getSignatureIndexes(uint16_t actualBlocks);

  void doTask();
  void goToObject(int object);
  void goalKeeper(int object);
  void kickBall();
  void findObject(int object);
  void findBall();
  void findGoal();
  
  void setupEncoderMove(float d, float r, float s);
  void setupEncoderSpin(float degrees, float s);
  void encoderMove();
  void encoderSpin();

  bool isVisible(int object);

  void getPixelDistance();

  float distancePixelsToCm(int object_size_pixels, float real_size_cm, bool measure_height);

  void tiltServo();

};

class ServoLoop {
public:
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;

  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
};

#endif
