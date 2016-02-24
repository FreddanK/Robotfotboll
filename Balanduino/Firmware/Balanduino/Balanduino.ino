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

 This is the algorithm for the Balanduino balancing robot.
 It can be controlled by either an Android app or a computer application via Bluetooth.
 The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 A dedicated Windows application can be found here: https://github.com/TKJElectronics/BalanduinoWindowsApp
 For details, see: http://balanduino.net/
*/

#include "Balanduino.h"

#include <Arduino.h> // Standard Arduino header
#include <Wire.h> // Official Arduino Wire library
#include <SPI.h> // Official Arduino SPI library

#include "Controller.h"
#include "Bluetooth.h"
#include "EEPROM.h"
#include "Motor.h"
#include "I2C.h"
#include "Tools.h"


#ifdef ENABLE_ADK
#include <adk.h>
#endif

#ifdef ENABLE_SPP
#include <SPP.h>
#endif

#if defined(ENABLE_SPP) || defined(ENABLE_ADK)
#define ENABLE_USB
USB Usb; // This will take care of all USB communication
#endif

#ifdef ENABLE_ADK
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
ADK adk(&Usb, "TKJ Electronics", // Manufacturer Name
              "Balanduino", // Model Name
              "Android App for Balanduino", // Description - user visible string
              "0.6.3", // Version of the Android app
              "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
              "1234"); // Serial Number - this is not used
#endif


#if defined(ENABLE_SPP)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
USBHub Hub(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
SPP SerialBT(&Btd, "Balanduino", "0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif


float batteryVoltage; // Measured battery level
uint8_t batteryCounter; // Counter used to check if it should check the battery level

Motor motor;


void setup() {

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup buzzer pin */
  motor.initBuzzer();

#ifdef ENABLE_USB
  if (Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start"));
    motor.setBuzzer();
    while (1); // Halt
  }
#endif
  
  /* Read the PID values, target angle and other saved values in the EEPROM */
  if (!checkInitializationFlags()) {
    readEEPROMValues(); // Only read the EEPROM values if they have not been restored
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    motor.soundBuzzer(1000);
    delay(100); // Wait a little after the pin is cleared
  }

  motor.setupEncoders();
  
  motor.setupMotors();

  motor.setupIMU();

#ifdef ENABLE_TOOLS
  printMenu();
#endif
  
  motor.calibrateAndReset();
  
  /* Beep to indicate that it is now ready */
  motor.soundBuzzer(100);

  motor.setupTiming();
}

void loop() {

  motor.checkMotors();

#if defined(ENABLE_AI)
  setControlOffset();
#endif

  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);
  motor.calculatePitch();

  motor.driveMotors();

  motor.updateEncoders();

    batteryCounter++;
    if (batteryCounter >= 10) { // Measure battery every 1s
      batteryCounter = 0;
      batteryVoltage = (float)analogRead(VBAT) / 63.050847458f; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
      if (batteryVoltage < 11.1 && batteryVoltage > 5) // Equal to 3.7V per cell - don't turn on if it's below 5V, this means that no battery is connected
        motor.setBuzzer();
      else
        motor.clearBuzzer();
    }
  }

  /* Read the Bluetooth dongle and send PID and IMU values */
#if defined(ENABLE_TOOLS)
  checkSerialData();
#endif
#if defined(ENABLE_USB)
  readUsb();
#endif
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
  printValues();
#endif

#ifdef ENABLE_BTD
  if (Btd.isReady()) {
    timer = millis();
    if ((Btd.watingForConnection && timer - blinkTimer > 1000) || (!Btd.watingForConnection && timer - blinkTimer > 100)) {
      blinkTimer = timer;
      LED::Toggle(); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } else
    LED::Clear(); // This will turn it off
#endif
}
