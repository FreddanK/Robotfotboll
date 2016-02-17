#ifndef _bluetooth_h_
#define _bluetooth_h_

/* Used to make commands more readable */
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

extern Command lastCommand; // This is used set a new targetPosition

void readSPPData();
void readUsb();
void steer(Command command);
float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);

#endif
