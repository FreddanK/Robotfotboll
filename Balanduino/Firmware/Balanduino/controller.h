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
  score,
  center,
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

  int32_t startLeftvalue = 0;
  int32_t startRightvalue = 0;
  int32_t startValue = 0;

  float targetDistance = 0;
  float radius = 0;
  float speed = 0;
  float targetTurningDistance = 0;
  float rate = 0;

  uint32_t ballVar;
  uint32_t goalVar;
  uint32_t opponentVar;
  uint16_t pixelDistance=100;
  boolean ballVisible;

public:
	Controller(Motor& m, Pixy& p) : motor(m), pixy(p) {}

	void doTask();
  void doTaskGoalKeeper();
	void goToObject(int object, int signature);
  void goToObjectGoalkeeper(int object, int signature);
  void goalKeeper(int object, int signature);
	void kickBall();
	void avoidObject();

  void setupEncoderMove(float d, float r, float s);
  void setupEncoderSpin(float degrees, float s);
  void encoderMove();
  void encoderSpin();

  void getSigVariables();
  void checkSurroundings();
  boolean checkVisibility(int signature);

  void centerBall();
  void scoreGoal();

  void getPixelDistance();

  float distancePixelsToCm(int object_size_pixels, float real_size_cm, bool measure_height);

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
