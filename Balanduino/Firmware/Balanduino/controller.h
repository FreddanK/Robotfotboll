#ifndef _controller_h_
#define _controller_h_

#include <Pixy.h>
#include <stdint.h>

#include "Motor.h";

enum Task {
	search,
	kick,
	avoid,
	goTo,
	wait,
};

class Controller {
private:
	Motor& motor;
	Pixy& pixy;

  uint16_t blocksCount = 0;
  uint16_t actualBlocks;
	Task task = search;
	uint32_t taskTimer = 0;
  uint32_t pixyTimer = 0;
  uint32_t ballVar;
  uint32_t opponentVar;
  boolean ballVisible;
public:
	Controller(Motor& m, Pixy& p) : motor(m), pixy(p) {}

	void doTask();
	void goToObject(int object, int signature);
	void kickBall();
	void avoidObject();
  void getSigVariables();
  void checkSurroundings();
  void checkIfBallSeen();

	//Test functions
	void moveBacknForth();
	void findBall();
	void rotate360();
	void makeCircle();

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
