/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This file contains functions for controlling the robot via Bluetooth.
*/

#ifndef _bluetooth_h_
#define _bluetooth_h_

#include <SPP.h>
#include <adk.h>
#include <usbhub.h> // Some dongles can have a hub inside

class Motor; //Forward declaration of Motor
class Tools;

class Bluetooth {
private:
  Motor * motor;
  Tools * tools;
public:
  USB Usb; // This will take care of all USB communication
  ADK adk{&Usb, "TKJ Electronics", // Manufacturer Name
            "Balanduino", // Model Name
            "Android App for Balanduino", // Description - user visible string
            "0.6.3", // Version of the Android app
            "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
            "1234"}; // Serial Number - this is not used
  USBHub Hub{&Usb}; // Some dongles have a hub inside
  BTD Btd{&Usb}; // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
  SPP SerialBT{&Btd, "Balanduino", "0000"}; // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones

  static const uint16_t receiveControlTimeout; // After how long time should it should prioritize the other controllers instead of the serial control

  Bluetooth(Motor * m, Tools * t) { motor = m; tools = t; }
  void init();
  
  void readSPPData();
  void readUsb();
  int USBInit();

  void blinkLed();
};

#endif
