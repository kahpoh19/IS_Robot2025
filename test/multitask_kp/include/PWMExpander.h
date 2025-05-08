#ifndef PWM_EXPANDER_H
#define PWM_EXPANDER_H

#include "GlobalVariables.h"
#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SDA_PIN 17
#define SCL_PIN 16

#define SERVOMIN 102
#define SERVOMAX 510

#define SERVO_FREQ 50

#define NUM_JOINTS 4

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// called this way, it uses the default address 0x40
const int MIN_PULSE_WIDTH = 650;
const int MAX_PULSE_WIDTH = 2350;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;

int AngleNow[4];
int AngleInitial[4];

int current = 0;
int flag = 0;

int current2 = 0;
int flag2 = 0;

// void PWMExtender(void *pvParameters);
void PWMExpander(int Channel, int val);
int angleToPulse(int ang);
void moveJoint(int jointNum, int angle);
void moveSelectedJointsSmooth(int joints[], int targetAngles[], int jointCount, int stepDelay = 5);
void retractArm();
void rotateArm(int ch0);
void DownArm();
void Grab();
void UpArm();
void Put();
void Down2();

#endif