#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


#define SDA_PIN 17
#define SCL_PIN 16
#define PWM_I2C_ADDR 0x40
#define SERVOMIN 102
#define SERVOMAX 510
#define SERVO_FREQ 50
#define NUM_JOINTS 4
const int PPM_PIN = 2;
const int SYNC_GAP = 3000;
const int MIN_PULSE_WIDTH = 900;
const int MAX_PULSE_WIDTH = 2100;
const int NUM_CHANNELS = 7;
#define PWM_FREQ 1000
#define PWM_RES 8
const int CENTER = (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2;
const int DEADZONE = 100;

volatile unsigned long lastTime = 0;
volatile int currentChannel = 0;
volatile unsigned long channelValues[NUM_CHANNELS];

#define motorL1A 21
#define motorL1B 47
#define motorL2A 19
#define motorL2B 20
#define motorR1A 0
#define motorR1B 35
#define motorR2A 48
#define motorR2B 45

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);

int AngleNow[4];
int AngleInitial[4];

int angleToPulse(int ang) {
  ang = constrain(ang, -135, 135);
  return map(ang, -135, 135, SERVOMIN, SERVOMAX);
}
void moveJoint(int jointNum, int angle) {
  int step = (angle > AngleNow[jointNum]) ? 1 : -1;
  for (int i = AngleNow[jointNum]; i != angle; i += step) {
    pwm.setPWM(jointNum, 0, angleToPulse(i));
    delay(5);
  }
  pwm.setPWM(jointNum, 0, angleToPulse(angle));
  delay(20);
  AngleNow[jointNum] = angle;
}
void moveSelectedJointsSmooth(int joints[], int targetAngles[], int jointCount,
                              int stepDelay = 5) {
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < jointCount; i++) {
      int ch = joints[i];
      if (AngleNow[ch] != targetAngles[i]) {
        int step = (targetAngles[i] > AngleNow[ch]) ? 1 : -1;
        AngleNow[ch] += step;
        pwm.setPWM(ch, 0, angleToPulse(AngleNow[ch]));
        moving = true;
      }
    }
    delay(stepDelay);
  }
}
void retractArm() {
  int targetAngles0[4] = {-120, -6, 0, 60};
  int targetJoints0[4] = {0, 1, 2, 3};
  moveSelectedJointsSmooth(targetJoints0, targetAngles0, 4);
}

void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long duration = currentTime - lastTime;
  lastTime = currentTime;

  if (duration > SYNC_GAP) {
    currentChannel = 0;
  } else if (duration >= MIN_PULSE_WIDTH && duration <= MAX_PULSE_WIDTH &&
             currentChannel < NUM_CHANNELS) {
    channelValues[currentChannel] = duration;
    currentChannel++;
  }
}

void stop() {
  analogWrite(motorL1A, 0);
  analogWrite(motorL1B, 0);
  analogWrite(motorR1A, 0);
  analogWrite(motorR1B, 0);
  analogWrite(motorL2A, 0);
  analogWrite(motorL2B, 0);
  analogWrite(motorR2A, 0);
  analogWrite(motorR2B, 0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(0, 0, angleToPulse(40));
  pwm.setPWM(1, 0, angleToPulse(-6));
  pwm.setPWM(2, 0, angleToPulse(-35));
  pwm.setPWM(3, 0, angleToPulse(120));
  AngleNow[0] = 40;
  AngleNow[1] = -6;
  AngleNow[2] = -35;
  AngleNow[3] = 120;

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  pinMode(motorL1A, OUTPUT);
  pinMode(motorL1B, OUTPUT);
  pinMode(motorR1A, OUTPUT);
  pinMode(motorR1B, OUTPUT);
  pinMode(motorL2A, OUTPUT);
  pinMode(motorL2B, OUTPUT);
  pinMode(motorR2A, OUTPUT);
  pinMode(motorR2B, OUTPUT);
  channelValues[5] = 0;
  channelValues[4] = 0;
  channelValues[6] = 0;
}

void DownArm() {
  int targetAngles1[3] = {40, -6, -35};
  int targetJoints1[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints1, targetAngles1, 3);
  moveJoint(3, 120);
}
void Grab() { moveJoint(3, 65); }
void UpArm() {
  int targetAngles2[3] = {-90, -6, 35};
  int targetJoints2[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints2, targetAngles2, 3);
}
void Put() { moveJoint(3, 120); }

void Down2() {
  int targetAngles3[3] = {-5, -6, 80};
  int targetJoints3[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints3, targetAngles3, 3);
  Put();
}

int current = 0;
int flag = 0;

int current2 = 0;
int flag2 = 0;
void loop() {
  Serial.print("Channel2: ");
  Serial.print(channelValues[1]);
  Serial.print(" Channel4: ");
  Serial.println(channelValues[3]);
  // 如果没有收到遥控信号，则停止并跳过电机控制
  bool noSignal = true;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channelValues[i] != 0) {
      noSignal = false;
      break;
    }
  }
  if (noSignal) {
    stop();
    return;
  }
  if (flag == 2 && (int)channelValues[5] <= 1100 &&
      (int)channelValues[5] >= 900) {
    current = 0;
    flag = 0;
    DownArm();
  }
  if (current == 0 && (int)channelValues[5] >= 1400 &&
      (int)channelValues[5] <= 1600) {
    Grab();
    retractArm();
    flag = 1;
    current = 1;
  }
  if (flag == 1 && (int)channelValues[5] >= 1900) {
    UpArm();
    delay(100);
    Put();
    flag = 2;
  }

  // 机械臂补救方案(欸嘿）
  if ((int)channelValues[4] >= 1800 && current2 == 0) {
    Down2();
    current2 = 1;
  }
  if ((int)channelValues[6] >= 1800 && flag2 == 0) {
    Grab();
    delay(200);
    retractArm();
    flag2 = 1;
  }
  if ((int)channelValues[4] <= 1200 && (int)channelValues[6] >= 1800 &&
      current2 == 1) {
    UpArm();
    delay(200);
    Put();
    current2 = 0;
  }
  if ((int)channelValues[4] <= 1200 && (int)channelValues[6] <= 1200 &&
      flag2 == 1) {
    DownArm();
    flag2 = 0;
  }
  // int adv = (int)channelValues[1] - CENTER;
  // int turn = (int)channelValues[3] - CENTER;

  // if (abs(adv) < DEADZONE) adv = 0;
  // if (abs(turn) < DEADZONE) turn = 0;

  // int maxRange = MAX_PULSE_WIDTH - CENTER;
  // int advSpeed = map(adv, -maxRange, maxRange, -255, 255);
  // int turnSpeed = map(turn, -maxRange, maxRange, -255, 255);

  // if (abs(advSpeed) > abs(turnSpeed)){
  //    if (advSpeed >= 0){
  //      analogWrite(motorR1A, advSpeed);
  //      analogWrite(motorR2A, advSpeed);
  //      analogWrite(motorR1B, 0);
  //      analogWrite(motorR2B, 0);
  //      analogWrite(motorL1A, 0);
  //      analogWrite(motorL2A, 0);
  //      analogWrite(motorL1B, advSpeed);
  //      analogWrite(motorL2B, advSpeed);
  //    }
  //    else{
  //      analogWrite(motorR1A, 0);
  //      analogWrite(motorR2A, 0);
  //      analogWrite(motorR1B, -advSpeed);
  //      analogWrite(motorR2B, -advSpeed);
  //      analogWrite(motorL1A, -advSpeed);
  //      analogWrite(motorL2A, -advSpeed);
  //      analogWrite(motorL1B, 0);
  //      analogWrite(motorL2B, 0);
  //    }
  //  }
  //  else {
  //    if(turnSpeed >= 0){
  //      analogWrite(motorR1A, 0);
  //      analogWrite(motorR2A, 0);
  //      analogWrite(motorR1B, turnSpeed);
  //      analogWrite(motorR2B, turnSpeed);
  //      analogWrite(motorL1A, 0);
  //      analogWrite(motorL2A, 0);
  //      analogWrite(motorL1B, turnSpeed);
  //      analogWrite(motorL2B, turnSpeed);
  //    }
  //    else{
  //      analogWrite(motorR1A, -turnSpeed);
  //      analogWrite(motorR2A, -turnSpeed);
  //      analogWrite(motorR1B, 0);
  //      analogWrite(motorR2B, 0);
  //      analogWrite(motorL1A, -turnSpeed);
  //      analogWrite(motorL2A, -turnSpeed);
  //      analogWrite(motorL1B, 0);
  //      analogWrite(motorL2B, 0);
  //    }
  //  }

  int adv = (int)channelValues[1] - CENTER;
  int turn = (int)channelValues[3] - CENTER;

  if (abs(adv) < DEADZONE)
    adv = 0;
  if (abs(turn) < DEADZONE)
    turn = 0;

  int maxRange = 500;
  int advSpeed = map(adv, -maxRange, maxRange, -230, 230);
  int turnSpeed = map(turn, -maxRange, maxRange, -230, 230);

  if (abs(advSpeed) > abs(turnSpeed)) {
    if (adv >= 0) {
      analogWrite(motorR1A, advSpeed);
      analogWrite(motorR2A, advSpeed);
      analogWrite(motorR1B, 0);
      analogWrite(motorR2B, 0);
      analogWrite(motorL1A, 0);
      analogWrite(motorL2A, 0);
      analogWrite(motorL1B, advSpeed);
      analogWrite(motorL2B, advSpeed);
    } else {
      analogWrite(motorR1A, 0);
      analogWrite(motorR2A, 0);
      analogWrite(motorR1B, -advSpeed);
      analogWrite(motorR2B, -advSpeed);
      analogWrite(motorL1A, -advSpeed);
      analogWrite(motorL2A, -advSpeed);
      analogWrite(motorL1B, 0);
      analogWrite(motorL2B, 0);
    }
  } else {
    if (turnSpeed >= 0) {
      analogWrite(motorR1A, 0);
      analogWrite(motorR2A, 0);
      analogWrite(motorR1B, turnSpeed);
      analogWrite(motorR2B, turnSpeed);
      analogWrite(motorL1A, 0);
      analogWrite(motorL2A, 0);
      analogWrite(motorL1B, turnSpeed);
      analogWrite(motorL2B, turnSpeed);
    } else {
      analogWrite(motorR1A, -turnSpeed);
      analogWrite(motorR2A, -turnSpeed);
      analogWrite(motorR1B, 0);
      analogWrite(motorR2B, 0);
      analogWrite(motorL1A, -turnSpeed);
      analogWrite(motorL2A, -turnSpeed);
      analogWrite(motorL1B, 0);
      analogWrite(motorL2B, 0);
    }
  }
}
