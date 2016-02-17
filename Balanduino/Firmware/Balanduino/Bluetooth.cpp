/* Copyright (C) 2013-2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/
#include "bluetooth.h"
#include "Balanduino.h"
#include "Motor.h"

Command lastCommand;

#if defined(ENABLE_USB)


#ifdef ENABLE_SPP
void readSPPData() {
  if (SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if (SerialBT.available()) {
      uint8_t i = 0;
      while (1) {
        dataInput[i] = SerialBT.read();
        if (dataInput[i] == -1) // Error while reading the string
          return;
        if (dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
          return;
      }
      bluetoothData = true;
      setValues(dataInput);
    }
  }
}
#endif // ENABLE_SPP

void readUsb() {
#ifdef ENABLE_USB
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (Usb.getUsbTaskState() == USB_STATE_ERROR && layingDown) { // Check if the USB state machine is in an error state, but also make sure the robot is laying down
    Serial.println(F("USB fail"));
    Usb.vbusPower(vbus_off);
    delay(1000);
    Usb.vbusPower(vbus_on);
    Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE); // Reset state machine
  }
#endif // ENABLE_USB

#ifdef ENABLE_SPP
  readSPPData();
#endif // ENABLE_SPP

  if (millis() > (receiveControlTimer + receiveControlTimeout)) {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers

    if (!commandSent) // If there hasn't been send a command by now, then send stop
      steer(stop);
  }
}


#endif // defined(ENABLE_USB)

#if defined(ENABLE_SPP) || defined(ENABLE_TOOLS)
void steer(Command command) {
  commandSent = true; // Used to see if there has already been send a command or not

  steerStop = false;
  targetOffset = turningOffset = 0; // Set both to 0

#if defined(ENABLE_SPP) || defined(ENABLE_TOOLS)
  if (command == joystick) {
    if (sppData2 > 0) // Forward
      targetOffset = scale(sppData2, 0, 1, 0, cfg.controlAngleLimit);
    else if (sppData2 < 0) // Backward
      targetOffset = -scale(sppData2, 0, -1, 0, cfg.controlAngleLimit);
    if (sppData1 > 0) // Right
      turningOffset = scale(sppData1, 0, 1, 0, cfg.turningLimit);
    else if (sppData1 < 0) // Left
      turningOffset = -scale(sppData1, 0, -1, 0, cfg.turningLimit);
  } else if (command == imu) {
    if (sppData1 > 0) // Forward
      targetOffset = scale(sppData1, 0, 36, 0, cfg.controlAngleLimit);
    else if (sppData1 < 0) // Backward
      targetOffset = -scale(sppData1, 0, -36, 0, cfg.controlAngleLimit);
    if (sppData2 > 0) // Right
      turningOffset = scale(sppData2, 0, 45, 0, cfg.turningLimit);
    else if (sppData2 < 0) // Left
      turningOffset = -scale(sppData2, 0, -45, 0, cfg.turningLimit);
  }
#endif // ENABLE_SPP or ENABLE_TOOLS

  if (command == stop) {
    steerStop = true;
    if (lastCommand != stop) { // Set new stop position
      targetPosition = getWheelsPosition();
      stopped = false;
    }
  }
  lastCommand = command;
}

float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { // Like map() just returns a float
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}
#endif // defined(ENABLE_SPP) || defined(ENABLE_TOOLS)
