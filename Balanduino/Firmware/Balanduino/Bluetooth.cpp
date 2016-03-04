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
#include "Bluetooth.h"

#include "Motor.h"
#include "Tools.h"


#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

uint16_t Bluetooth2::receiveControlTimeout = 500;

void Bluetooth2::readSPPData() {
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


void Bluetooth2::readUsb() {
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (Usb.getUsbTaskState() == USB_STATE_ERROR && layingDown) { // Check if the USB state machine is in an error state, but also make sure the robot is laying down
    Serial.println(F("USB fail"));
    Usb.vbusPower(vbus_off);
    delay(1000);
    Usb.vbusPower(vbus_on);
    Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE); // Reset state machine
  }

  readSPPData();

  if (millis() > (receiveControlTimer + receiveControlTimeout)) {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers

    if (!commandSent) // If there hasn't been send a command by now, then send stop
      steer(stop);
  }
}

int Bluetooth2::USBInit() {
  return USB.init();
}

void Bluetooth2::blinkLed() {
  if (Btd.isReady()) {
    timer = millis();
    if ((Btd.watingForConnection && timer - blinkTimer > 1000) || (!Btd.watingForConnection && timer - blinkTimer > 100)) {
      blinkTimer = timer;
      LED::Toggle(); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } 
  else
    LED::Clear(); // This will turn it off
}




