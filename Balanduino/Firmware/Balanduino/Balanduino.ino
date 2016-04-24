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
#include <Pixy.h>

#include "Motor.h"
#include "Eeprom.h"
#include "Controller.h"
#include "Tools.h"
#include "Bluetooth.h"
#include "Microphone.h"


#if ARDUINO < 156 // Make sure that at least Arduino IDE version 1.5.6 is used
  #error "Please update the Arduino IDE to version 1.5.6 or newer at the following website: http://arduino.cc/en/Main/Software"
#endif


Motor motor{};
Eeprom eeprom{&motor};
Pixy pixy{};
Controller controller{motor, pixy};
Tools tools{&motor, &eeprom};
Bluetooth bluetooth{&motor, &tools};
Microphone microphone{};

void setup() {

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup buzzer pin */
  motor.initBuzzer();

  bluetooth.init();
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


  tools.printMenu();

  pixy.init();
  
  motor.calibrateAndReset();
  
  /* Beep to indicate that it is now ready */
  motor.soundBuzzer(100);

  motor.setupTiming();
}

void loop() {

  motor.checkMotors();
  //controller.tiltServo();

  microphone.readMic();
  if(microphone.robotOn)
    controller.doTask();
  else
    motor.steer(stop);

  //Serial.print(motor.accAngle);Serial.print('\t');Serial.print(motor.gyroAngle);Serial.print('\t');Serial.println(motor.pitch);
  motor.calculatePitch();

  motor.driveMotors();

  motor.updateEncoders();

  tools.checkBatteryVoltage();
  /* Read the Bluetooth dongle and send PID and IMU values */
  tools.checkSerialData();

  bluetooth.readUsb();

  tools.printValues();


  bluetooth.blinkLed();

}
