#ifndef _motor_h_
#define _motor_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <Usb.h>

#include "Bluetooth.h"

// These pins macros are defined in avrpins.h in the USB Host library. This allows to read and write directly to the port registers instead of using Arduino's slow digitalRead()/digitalWrite() functions
// The source is available here: https://github.com/felis/USB_Host_Shield_2.0/blob/master/avrpins.h
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
// Also see the Arduino port manipulation guide: http://www.arduino.cc/en/Reference/PortManipulation

/* Left motor */
#define leftA P23
#define leftB P24
#define leftPWM P18

/* Right motor */
#if BALANDUINO_REVISION < 13
  #define rightA P25
  #define rightB P26
#else
  #define rightA P15
  #define rightB P16
#endif
#define rightPWM P17

/* Pins connected to the motor drivers diagnostic pins */
#define leftDiag P21
#define rightDiag P22


void updatePID(float restAngle, float offset, float turning, float dt);
void moveMotor(Command motor, Command direction, float speedRaw);
void stopMotor(Command motor);
void setPWM(Command motor, uint16_t dutyCycle);
void stopAndReset();
// On newer versions of the PCB these two functions are only used in one place, so they will be inlined by the compiler.
void leftEncoder();
void rightEncoder();
int32_t readLeftEncoder();
int32_t readRightEncoder();
int32_t getWheelsPosition();

#endif
