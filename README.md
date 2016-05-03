# Robotfotboll
### Introduction
-------------------

This project's aim is to devlop automated football playing 
robots. The code runs on predeveloped hardware such as the Balanduino 
robot which is a balancing robot based on the arduino platform. 
A camera called Pixy has ben used for objectdetection.

### License
-------------------
This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation.

### Requirements
-------------------

#### Hardware
Balanduinoplatform: http://www.balanduino.net/

CMUcam5 Pixy: http://www.cmucam.org/projects/cmucam5

#### Software
Arduino IDE: https://www.arduino.cc/en/Main/Software

PixyMon: http://cmucam.org/projects/cmucam5/wiki/Latest_release 

#### Arduino Libraries
Kalman Filter Library https://github.com/TKJElectronics/KalmanFilter

Pixy Arduino Library http://cmucam.org/projects/cmucam5/wiki/Latest_release

QueueList Library http://playground.arduino.cc/Code/QueueList

USB Host Shield Library 2.0 https://github.com/felis/USB_Host_Shield_2.0

####Arduino Boards
Hardware add-on whith the Balanduino board manager. 
See Installation for information on how to set this up. 
When adding this the Kalman Filter Library and USB Host Shield Library 2.0 
will be installed automatically.


### Installation
-------------------

Clone this git repository `git clone https://github.com/FreddanK/Robotfotboll/`
or download it as a zip-file.

Download the Pixy Arduino Library http://cmucam.org/projects/cmucam5/wiki/Latest_release 
and the 
QueueList Library http://playground.arduino.cc/Code/QueueList#Download 

Add the libraries as zip-libraries. In the Arduino IDE go to Sketch->Include Library->Add .ZIP Library

Add the Balanduino board manager. In the Arduino IDE open Preferences then Copy `https://raw.githubusercontent.com/TKJElectronics/Balanduino/master/package_tkj_balanduino_index.json` and Paste it into the 'Additional Boards Manager URLs:' field.

For Arduino IDE version 1.6.3 or lower, or for more information see Getting started for the Balanduino: http://www.balanduino.net/get-started 


### Configuration
-------------------

Before uploading the code to the Balanduion, make sure to select the right board and revision.

In the Arduino IDE under Tools->Boards choose 'Balanduino' and under Tools->Revision choose 'Revision 1.3'

#### Teaching Pixy objects
TODO: add guide here


### Troubleshooting
-------------------
TODO: add FAQ and troubleshooting here

### Authors
-------------------
Bachelor project group SSYX02-1613:

Sara Boström

Edvin Eriksson Johansson

Fredrik Kjellberg

Rickard Larsson

Klar Lundgren

My Resare