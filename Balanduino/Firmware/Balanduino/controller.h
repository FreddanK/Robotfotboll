#ifndef _controller_h_
#define _controller_h_
//Set ENABLE_AI and disable ENABLE_SPP and ENABLE_ADK in Balanduino.ino to make this work

class Motor;

class Controller {
public:
  void setControlOffset(Motor * motor);
};


#endif
