#ifndef TASK_PPM_READ_H
#define TASK_PPM_READ_H

#include <Arduino.h>
#include "GlobalVariables.h"

void RXRead(void *pvParameters);
void IRAM_ATTR ppmInterruptHandler();

#endif