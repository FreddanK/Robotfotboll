/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This file contains the 'brain' of the automated robot and all the function that
calculates the control signal that is sent to the regulator.
This is for a player with an offensive role.
*/

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
	goToBall,
  score,
  center,
  encMove,
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
    MoveInstruction(MoveType m, float s, float deg) : moveType(m), distance(0), radius(0), speed(s), degree(deg) {}
    MoveInstruction(MoveType m, float d, float r, float s) : moveType(m), distance(d), radius(r), speed(s), degree(0) {}
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
  uint16_t lastXPosGoal;

  

  int32_t startLeftvalue = 0;
  int32_t startRightvalue = 0;
  //int32_t startValue = 0;

  //float targetDistance = 0;
  //float radius = 0;
  //float speed = 0;
  float targetTurningDistance = 0;
  


public:
  bool getNewMove = true;
  QueueList <MoveInstruction> moveInstructionQueue;
	Controller(Motor& m, Pixy& p) : motor(m), pixy(p) {}

  void doTask();
  Task makeDecision(uint16_t actualBlocks, Task lastTask);
  
  void goToObject(int object);
  void scoreGoal();
  void kickBall();
  void avoidObject();
  void centerBall();
  void findBall();
  void findGoal();

  void setupEncoderMove();
  void encoderMove();
  float spinCheck();
  void clearInstructionQueue();

  void getSignatureIndexes(uint16_t actualBlocks);
  bool isVisible(int object);
  void calculateTrajectory();
  void calculateTrajectory2();
  double distanceToObject(int object_size, float real_size, bool measure_height);
  double distanceBetween(int16_t object1, int16_t object2);
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
