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
/*
#include "Bluetooth.h"

#include "Motor.h"
#include "Tools.h"


#define MAKE_PIN(pin) _MAKE_PIN(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define _MAKE_PIN(pin) P ## pin
#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

const uint16_t Bluetooth::receiveControlTimeout = 500;

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


void Bluetooth::printValues() {
#ifdef ENABLE_SPP
  Print *out; // This allows the robot to use either the hardware UART or the Bluetooth SPP connection dynamically
  if (SerialBT.connected && bluetoothData)
    out = dynamic_cast<Print *> (&SerialBT); // Print using the Bluetooth SPP interface
  else
    out = dynamic_cast<Print *> (&Serial); // Print using the standard UART port
#else
  HardwareSerial *out = &Serial; // Print using the standard UART port
#endif

  if (sendPairConfirmation) {
    sendPairConfirmation = false;

    out->println(F("PC"));
  } else if (sendPIDValues) {
    sendPIDValues = false;

    out->print(F("P,"));
    out->print(cfg.P);
    out->print(F(","));
    out->print(cfg.I);
    out->print(F(","));
    out->print(cfg.D);
    out->print(F(","));
    out->println(cfg.targetAngle);
  } else if (sendSettings) {
    sendSettings = false;

    out->print(F("S,"));
    out->print(cfg.backToSpot);
    out->print(F(","));
    out->print(cfg.controlAngleLimit);
    out->print(F(","));
    out->println(cfg.turningLimit);
  } else if (sendInfo) {
    sendInfo = false;

    out->print(F("I,"));
    out->print(version);
    out->print(F(","));
    out->print(eepromVersion);

#if defined(__AVR_ATmega644__)
    out->println(F(",ATmega644"));
#elif defined(__AVR_ATmega1284P__)
    out->println(F(",ATmega1284P"));
#else
    out->println(F(",Unknown"));
#endif
  } else if (sendKalmanValues) {
    sendKalmanValues = false;

    out->print(F("K,"));
    out->print(kalman.getQangle(), 4);
    out->print(F(","));
    out->print(kalman.getQbias(), 4);
    out->print(F(","));
    out->println(kalman.getRmeasure(), 4);
  } else if (sendIMUValues && millis() - imuTimer > 50) { // Only send data every 50ms
    imuTimer = millis();

    out->print(F("V,"));
    out->print(accAngle);
    out->print(F(","));
    out->print(gyroAngle);
    out->print(F(","));
    out->println(pitch);
  } else if (sendStatusReport && millis() - reportTimer > 500) { // Send data every 500ms
    reportTimer = millis();

    out->print(F("R,"));
    out->print(batteryVoltage);
    out->print(F(","));
    out->println((float)reportTimer / 60000.0f);
  }
}

void Bluetooth::setValues(char *input) {
  if (input[0] == 'A' && input[1] == ';') { // Abort
    stopAndReset();
#ifdef ENABLE_SPP
    while (Serial.read() != 'C' && SerialBT.read() != 'C') // Wait until continue is sent
      Usb.Task();
#else
    while (Serial.read() != 'C');
#endif
  }

  else if (input[0] == 'A' && input[1] == 'C') // Accelerometer calibration
    calibrateAcc();
  else if (input[0] == 'M' && input[1] == 'C') // Motor calibration
    calibrateMotor();

  //For sending PID and IMU values
  else if (input[0] == 'G') { // The different application sends when it needs the PID, settings or info
    if (input[1] == 'P') // Get PID Values
      sendPIDValues = true;
    else if (input[1] == 'S') // Get settings
      sendSettings = true;
    else if (input[1] == 'I') // Get info
      sendInfo = true;
    else if (input[1] == 'K') // Get Kalman filter values
      sendKalmanValues = true;
  }

  else if (input[0] == 'S') { // Set different values
    //Set PID and target angle
    if (input[1] == 'P') {
      strtok(input, ","); // Ignore 'P'
      cfg.P = atof(strtok(NULL, ";"));
    } else if (input[1] == 'I') {
      strtok(input, ","); // Ignore 'I'
      cfg.I = atof(strtok(NULL, ";"));
    } else if (input[1] == 'D') {
      strtok(input, ","); // Ignore 'D'
      cfg.D = atof(strtok(NULL, ";"));
    } else if (input[1] == 'T') { // Target Angle
      strtok(input, ","); // Ignore 'T'
      cfg.targetAngle = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'K') { // Kalman values
      strtok(input, ","); // Ignore 'K'
      cfg.Qangle = atof(strtok(NULL, ","));
      cfg.Qbias = atof(strtok(NULL, ","));
      cfg.Rmeasure = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'A') { // Controlling max angle
      strtok(input, ","); // Ignore 'A'
      cfg.controlAngleLimit = atoi(strtok(NULL, ";"));
    } else if (input[1] == 'U') { // Turning max value
      strtok(input, ","); // Ignore 'U'
      cfg.turningLimit = atoi(strtok(NULL, ";"));
    }
    else if (input[1] == 'B') { // Set Back To Spot
      if (input[3] == '1')
        cfg.backToSpot = 1;
      else
        cfg.backToSpot = 0;
    }

    updateConfig();
  }

  else if (input[0] == 'I') { // IMU transmitting states
    if (input[1] == 'B') // Begin sending IMU values
      sendIMUValues = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending IMU values
      sendIMUValues = false; // Stop sending output to application
  }

  else if (input[0] == 'R') { // Report states
    if (input[1] == 'B') // Begin sending report values
      sendStatusReport = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending report values
      sendStatusReport = false; // Stop sending output to application
  }

  else if (input[0] == 'C') { // Commands
    if (input[1] == 'S') // Stop
      steer(stop);
    else if (input[1] == 'J') { // Joystick
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'J'
      sppData1 = atof(strtok(NULL, ",")); // x-axis
      sppData2 = atof(strtok(NULL, ";")); // y-axis
      steer(joystick);
    }
    else if (input[1] == 'M') { // IMU
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'M'
      sppData1 = atof(strtok(NULL, ",")); // Pitch
      sppData2 = atof(strtok(NULL, ";")); // Roll
      steer(imu);
    }

    else if (input[1] == 'R') {
      restoreEEPROMValues(); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
  }
}

*/

