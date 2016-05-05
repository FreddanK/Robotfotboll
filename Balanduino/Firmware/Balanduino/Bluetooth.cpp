/* 
Original code by 2013-2015 Kristian Lauszus, TKJ Electronics.

Modified by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
*/

#include "Bluetooth.h"

#include "Motor.h"
#include "Tools.h"


#define MAKE_PIN(pin) _MAKE_PIN(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define _MAKE_PIN(pin) P ## pin
#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

const uint16_t Bluetooth::receiveControlTimeout = 500;

void Bluetooth::init() {
  tools->addBluetooth(this);
}

void Bluetooth::readSPPData() {
  if (SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if (SerialBT.available()) {
      uint8_t i = 0;
      while (1) {
        tools->dataInput[i] = SerialBT.read();
        if (tools->dataInput[i] == -1) // Error while reading the string
          return;
        if (tools->dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(tools->dataInput) / sizeof(tools->dataInput[0])) // String is too long
          return;
      }
      tools->bluetoothData = true;
      tools->setValues(tools->dataInput);
    }
  }
}


void Bluetooth::readUsb() {
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (Usb.getUsbTaskState() == USB_STATE_ERROR && motor->layingDown) { // Check if the USB state machine is in an error state, but also make sure the robot is laying down
    Serial.println(F("USB fail"));
    Usb.vbusPower(vbus_off);
    delay(1000);
    Usb.vbusPower(vbus_on);
    Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE); // Reset state machine
  }

  readSPPData();

  if (millis() > (tools->receiveControlTimer + receiveControlTimeout)) {
    motor->commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers

    if (!motor->commandSent) // If there hasn't been send a command by now, then send stop
      motor->steer(stop);
  }
}

int Bluetooth::USBInit() {
  return Usb.Init();
}

void Bluetooth::blinkLed() {
  if (Btd.isReady()) {
    uint32_t timer = millis();
    if ((Btd.watingForConnection && timer - motor->blinkTimer > 1000) || (!Btd.watingForConnection && timer - motor->blinkTimer > 100)) {
      motor->blinkTimer = timer;
      LED::Toggle(); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } 
  else
    LED::Clear(); // This will turn it off
}


