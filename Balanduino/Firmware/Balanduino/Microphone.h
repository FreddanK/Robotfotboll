#ifndef _microphone_h_
#define _microphone_h_

#include <stdint.h>
#include <Arduino.h>

class Microphone {
  
  uint32_t micTimer = 0;  //Timer that keeps track of when the mic input should be read.
  uint16_t micCounter = 0; //Counting number of times a loud sound is recorded.
  uint16_t micResetCounter = 0; //Counts number of times a quiet or no sound i recorded.
public:
  bool robotOn = false;
  void readMic() {
    if(millis()-micTimer>100){
      int val=analogRead(0);   //connect mic sensor to Analog 0
      if(val>50) {
        micCounter++;
        if(micCounter>1){
          if(robotOn) {
            robotOn = false;
          }
          else {
            robotOn = true;
          }
          micCounter=0;
        }
        micResetCounter=0;
      }
      else {
        micResetCounter++;
        if(micResetCounter>10){
          micCounter=0;
        }
      }
      //Serial.println(val,DEC);//print the sound value to serial **For debugging**
      micTimer = millis();
    }
  }

};

#endif
