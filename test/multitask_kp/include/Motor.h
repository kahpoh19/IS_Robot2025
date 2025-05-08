#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include "GlobalVariables.h"
#include <Arduino.h>


// Motor control pins
const int motorL1A = 21;
const int motorL1B = 47;
const int motorL2A = 19;
const int motorL2B = 20;
const int motorR1A = 0;
const int motorR1B = 35;
const int motorR2A = 48;
const int motorR2B = 45;

const int channelLA = 0;
const int channelLB = 1;
const int channelRA = 2;
const int channelRB = 3;

const int pwmFreq = 5000;
const int pwmResolution = 8;
// const int motorProt = 500; // Default protection time for motor direction change
// const int speedlim = 230;  // Maximum input for the motor driver (0 - 255)

// // Channel values
// const int minres = 1450;
// const int maxres = 1550;

// // Ramp-up configuration
// const int rampStep = 5;  // Step size for speed increase
// const int rampDelay = 5; // Delay in milliseconds for each ramp step

// void MotorDriving(void *pvParameters); // Main task

// // Function prototypes
// void updateState(int advSignal, int turnSignal);
// void executeState();
// void stopMotors();
// void executeActionImmediately();
// void setMotorDirection(bool dirM1, int targetSpeedM1, bool dirM2,
//                        int targetSpeedM2);
// void rampSpeed(int &currentSpeed, int targetSpeed, int controlPinForward,
//                int controlPinReverse);
// void controlForward();
// void controlReverse();

void Motor();
void stop();

#endif