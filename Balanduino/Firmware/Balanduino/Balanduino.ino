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
#include <Arduino.h>

#include "Controller.h"
#include "Bluetooth.h"
#include "Eeprom.h"
#include "Motor.h"
#include "Tools.h"



#if ARDUINO < 156 // Make sure that at least Arduino IDE version 1.5.6 is used
  #error "Please update the Arduino IDE to version 1.5.6 or newer at the following website: http://arduino.cc/en/Main/Software"
#endif


uint8_t batteryCounter; // Counter used to check if it should check the battery level

Bluetooth2 bluetooth{};
Motor motor{};
Eeprom eeprom{&motor};


void setup() {

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup buzzer pin */
  motor.initBuzzer();

  if(bluetooth.USBInit() == -1) {
    Serial.print(F("OSC did not start"));
    motor.setBuzzer();
    while (1); // Halt
  }
  
  /* Read the PID values, target angle and other saved values in the EEPROM */
  if (!eeprom.checkInitializationFlags()) {
    eeprom.readEEPROMValues(); // Only read the EEPROM values if they have not been restored
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    motor.soundBuzzer(1000);
    delay(100); // Wait a little after the pin is cleared
  }

  motor.setupEncoders();
  
  motor.setupMotors();

  motor.setupIMU();


  //tools.printMenu();
  
  motor.calibrateAndReset();
  
  /* Beep to indicate that it is now ready */
  motor.soundBuzzer(100);

  motor.setupTiming();
}

void loop() {

  motor.checkMotors();

  setControlOffset();

  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);
  motor.calculatePitch();

  motor.driveMotors();

  motor.updateEncoders();

  batteryCounter++;
  if (batteryCounter >= 10) { // Measure battery every 1s
    batteryCounter = 0;
    //batteryVoltage = (float)analogRead(VBAT) / 63.050847458f; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
    //if (batteryVoltage < 11.1 && batteryVoltage > 5) // Equal to 3.7V per cell - don't turn on if it's below 5V, this means that no battery is connected
      //motor.setBuzzer();
    //else
      //motor.clearBuzzer();
  }
}

  /* Read the Bluetooth dongle and send PID and IMU values */
  //tools.checkSerialData();

  bluetooth.readUsb();

  //tools.printValues();


  bluetooth.blinkLed();

}
