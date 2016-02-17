#ifndef _tools_h_
#define _tools_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

void checkSerialData();
void printMenu();
void calibrateMotor();
void testMotorSpeed(float *leftSpeed, float *rightSpeed, float leftScaler, float rightScaler);
void calibrateAcc();
void printValues();
void setValues(char *input);
bool calibrateGyro();
bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);

#endif
