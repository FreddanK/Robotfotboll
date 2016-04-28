#ifndef _controller_h_
#define _controller_h_

#include <Pixy.h>
#include <stdint.h>
#include <QueueList.h>

#include "Motor.h";

enum Task {
	search,
	kick,
	avoid,
	goTo,
	wait,
  score,
  center,
};

enum MoveType {
  line, spin,
};

struct MoveInstruction {
    MoveType moveType;
    float distance;
    float radius;
    float speed;
    float degree;
    MoveInstruction() : moveType(line), distance(0), radius(0), speed(0), degree(0) {}
    MoveInstruction(MoveType m, float s, float deg) {MoveInstruction(m,0,0,s,deg);}
    MoveInstruction(MoveType m, float d, float r, float s) {MoveInstruction(m,d,r,s,0);}
    MoveInstruction(MoveType m, float d, float r, float s, float deg) : moveType(m), distance(d), radius(r), speed(s), degree(deg) {}
};

class Controller {
private:
	Motor& motor;
	Pixy& pixy;

  int16_t objectIndex[6];
  float objectDistance[6];

	Task task = search;
  bool centered=false;
	uint32_t taskTimer = 0;
  uint32_t pixyTimer = 0;
  uint16_t lastXPosBall;

  QueueList <MoveInstruction> moveInstructionQueue;

  int32_t startLeftvalue = 0;
  int32_t startRightvalue = 0;
  int32_t startValue = 0;

  float targetDistance = 0;
  float radius = 0;
  float speed = 0;
  float targetTurningDistance = 0;
  float rate = 0;


public:
	Controller(Motor& m, Pixy& p) : motor(m), pixy(p) {}

  void doTask();
  
  void goToObject(int object);
  void scoreGoal();
  void kickBall();
  void avoidObject();
  void centerBall();
  void findBall();

  void setupEncoderMove(float d, float r, float s);
  void setupEncoderSpin(float degrees, float s);
  void encoderMove();
  void encoderSpin();

  void getSignatureIndexes(uint16_t actualBlocks);
  boolean isVisible(int object);
  float distanceToObject(int object_size, float real_size, bool measure_height);
  float distanceBetween(int16_t object1, int16_t object2);
  int16_t getXposDiff(int16_t object1, int16_t object2);

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
