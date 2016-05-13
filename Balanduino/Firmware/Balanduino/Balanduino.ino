/* 
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This is the main program for the soccer playing robot.
*/
#include <Arduino.h>
#include <Pixy.h>

#include "Motor.h"
#include "Eeprom.h"
#include "Controller.h"
//#include "ControllerGoal.h"
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
//ControllerGoal controllerGoal{motor,pixy}; //when running goalkeeper
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

  MoveInstruction m = MoveInstruction(line,300,0,20);
  controller.moveInstructionQueue.push(m);
  controller.setupEncoderMove();
}

void loop() {

  motor.checkMotors();
  //controller.tiltServo();
  if(millis() > 2000){
    controller.encoderMove();
  }

  // microphone.readMic();
  // if(microphone.robotOn){
  //   controller.doTask(); //when running offensive player
  // }
  // else{
  //   motor.steer(stop);
  //   controller.resetValues();
  // }
  // controller.doTask();

  //Serial.print(motor.accAngle);Serial.print('\t');Serial.print(motor.gyroAngle);Serial.print('\t');Serial.println(motor.pitch);
  motor.calculatePitch();

  motor.driveMotors();

  motor.updateEncoders();

 // tools.checkBatteryVoltage();
  /* Read the Bluetooth dongle and send PID and IMU values */
  tools.checkSerialData();

  bluetooth.readUsb();

  tools.printValues();


  bluetooth.blinkLed();

}
