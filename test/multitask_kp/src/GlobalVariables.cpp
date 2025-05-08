#include "GlobalVariables.h"

TaskManager taskManager;
TaskManager taskRGBLed;
TaskManager taskPrint;
TaskManager taskRXRead;
TaskManager taskMotorDriving;
TaskManager taskServot1;
// TaskManager taskPWMExt;

const int NUM_CHANNELS = 7; // Number of channels to read
volatile unsigned int channelValues[NUM_CHANNELS] = {0};

int cservo = 0;