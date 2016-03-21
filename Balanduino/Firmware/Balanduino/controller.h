#ifndef _controller_h_
#define _controller_h_

#include <Pixy.h>

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

	Task task = search;
	unsigned long taskTimer = 0;
public:
	Controller(Motor& m, Pixy& p) : motor(m), pixy(p) {}

	void doTask();
	void goToObject(int object, int signature);
	void kickBall();
	void avoidObject();

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
