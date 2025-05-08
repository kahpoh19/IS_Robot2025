#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include <Arduino.h>
#include <TaskManager.h>

extern TaskManager taskManager;
extern TaskManager taskRGBLed;
extern TaskManager taskPrint;
extern TaskManager taskRXRead;
extern TaskManager taskMotorDriving;
extern TaskManager taskServot1;
// extern TaskManager taskPWMExt;

extern const int NUM_CHANNELS;                // Number of channels to read
extern volatile unsigned int channelValues[]; // Array to store channel values

extern int cservo;

#endif