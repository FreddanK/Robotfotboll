#ifndef _controller_h_
#define _controller_h_

#include <Pixy.h>

class Motor;

enum Task {
	search,
	kick,
	avoid,
	goTo,
	wait,
};

class Controller {
private:
	Motor * motor;
	Pixy * pixy;

	Task task = search;
	unsigned long taskTimer = 0;
public:
	Controller(Motor * m, Pixy * p) { motor = m; pixy = p;}

	void doTask();
	void goToObject(int object, int signature);
	void kickBall();
	void avoidObject(int object);


	//Test functions
	void moveBacknForth();
	void findBall();
	void rotate360();
	void makeCircle();

};


#endif
