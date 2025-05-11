#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 17
#define SCL_PIN 16
#define PWM_I2C_ADDR 0x40
#define SERVOMIN 102
#define SERVOMAX 510
#define SERVO_FREQ 50
#define NUM_JOINTS 5
const int PPM_PIN = 2;
const int SYNC_GAP = 3000;
const int MIN_PULSE_WIDTH = 900;
const int MAX_PULSE_WIDTH = 2100;
const int NUM_CHANNELS = 10;
#define PWM_FREQ 1000
#define PWM_RES 8
const int CENTER = (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2;
const int DEADZONE = 100;

volatile unsigned long lastTime = 0;
volatile int currentChannel = 0;
volatile unsigned long channelValues[NUM_CHANNELS];

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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);
int AngleNow[5] = {-120,-6,0,60,0};

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
void moveSelectedJointsSmooth(int joints[], int targetAngles[], int jointCount, int stepDelay = 5) {
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

// 摇杆控制底盘舵机
//控制1
// void rotateArm(int ch0) {
//   int rt = ch0 - 1500;
//   if (abs(rt) < 100) {
//     rt = 0;
//   }
//   int rotate = map(rt, -500, 500, -30, 30);
//   moveJoint(4,-rotate);
// }
//控制2
void rotateArm(int ch0) {
  int rt = ch0 - 1500;
  if (abs(rt) < 50) {
    rt = 0;
  }
  if (AngleNow[4] >= 20){
    moveJoint(4,19);
  }
  else if (AngleNow[4] <= -20){
    moveJoint(4,-19);
  }
  if (rt > 0){
    moveJoint(4,AngleNow[4]-1);
  }
  else if (rt < 0){
    moveJoint(4,AngleNow[4]+1);
  }
}
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long duration = currentTime - lastTime;
  lastTime = currentTime;

  if (duration > SYNC_GAP) {
    currentChannel = 0;
  } else if (duration >= MIN_PULSE_WIDTH && duration <= MAX_PULSE_WIDTH && currentChannel < NUM_CHANNELS) {
    channelValues[currentChannel] = duration;
    currentChannel++;
  }
}
void stop() {
  ledcWrite(channelLA, 0);
  ledcWrite(channelLB, 0);
  ledcWrite(channelRA, 0);
  ledcWrite(channelRB, 0);
}
// 每通道双区间状态：inZone[ch][0]=low, inZone[ch][1]=high
static bool inZone[NUM_CHANNELS][2] = {};
// 双区间触发：只有“外→内”才执行
static void handleZoneSwitch(int ch, int val,
                             int lowMin, int lowMax, void (*lowAction)(),
                             int highMin, int highMax, void (*highAction)()) {
  bool low  = (val >= lowMin  && val <= lowMax);
  bool high = (val >= highMin && val <= highMax);
  if (low  && !inZone[ch][0]) lowAction();
  if (high && !inZone[ch][1]) highAction();
  inZone[ch][0] = low;
  inZone[ch][1] = high;
}
// 单区间触发
static bool inZoneSingle[NUM_CHANNELS] = {};
static void handleZoneSingle(int ch, int val, int minVal, int maxVal, void (*action)()) {
  bool now = (val >= minVal && val <= maxVal);
  if (now && !inZoneSingle[ch]) action();
  inZoneSingle[ch] = now;
}

void move(int ch1, int ch3){
  int adv = ch1 - CENTER;
  int turn = ch3 - CENTER;
  if (abs(adv) < DEADZONE)
    adv = 0;
  if (abs(turn) < DEADZONE)
    turn = 0;
  int maxRange = 500;
  int advSpeed = map(adv, -maxRange, maxRange, -230, 230);
  int turnSpeed = map(turn, -maxRange, maxRange, -230, 230);

  if (abs(advSpeed) > abs(turnSpeed)) {

    if (adv >= 0) {
      ledcWrite(channelLA, 0);
      ledcWrite(channelLB, advSpeed/3);
      ledcWrite(channelRA, advSpeed/3);
      ledcWrite(channelRB, 0);
    } else {
      ledcWrite(channelLA, -advSpeed/3);
      ledcWrite(channelLB, 0);
      ledcWrite(channelRA, 0);
      ledcWrite(channelRB, -advSpeed/3);
    }
  } else {
    if (turnSpeed >= 0) {
      ledcWrite(channelLA, 0);
      ledcWrite(channelLB, turnSpeed/5);
      ledcWrite(channelRA, 0);
      ledcWrite(channelRB, turnSpeed/5);
    } else {
      ledcWrite(channelLA, -turnSpeed/5);
      ledcWrite(channelLB, 0);
      ledcWrite(channelRA, -turnSpeed/5);
      ledcWrite(channelRB, 0);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(0, 0, angleToPulse(-120));
  pwm.setPWM(1, 0, angleToPulse(-6));
  pwm.setPWM(2, 0, angleToPulse(0));
  pwm.setPWM(3, 0, angleToPulse(60));
  pwm.setPWM(4, 0, angleToPulse(0));
  pwm.setPWM(5, 0, angleToPulse(15));


  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  // channelValues[5] = 0;//放
  // channelValues[4] = 0;//拿2
  // channelValues[7] = 0;//拿1

  ledcSetup(channelLA, pwmFreq, pwmResolution);
  ledcSetup(channelLB, pwmFreq, pwmResolution);
  ledcSetup(channelRA, pwmFreq, pwmResolution);
  ledcSetup(channelRB, pwmFreq, pwmResolution);

  ledcAttachPin(motorL1A, channelLA);
  ledcAttachPin(motorL2A, channelLA);
  ledcAttachPin(motorL1B, channelLB);
  ledcAttachPin(motorL2B, channelLB);
  ledcAttachPin(motorR1A, channelRA);
  ledcAttachPin(motorR2A, channelRA);
  ledcAttachPin(motorR1B, channelRB);
  ledcAttachPin(motorR2B, channelRB);
}

void DownArm() {
  int targetAngles1[4] = {38, 0, -32, 120};
  int targetJoints1[4] = {0, 1, 2, 3};
  moveSelectedJointsSmooth(targetJoints1, targetAngles1, 4);
}
void Down2() {
  int targetAngles3[4] = {2, -6, 80, 120};
  int targetJoints3[4] = {0, 1, 2, 3};
  moveSelectedJointsSmooth(targetJoints3, targetAngles3, 4);
}
void Put() {
  int targetAngles2[3] = {-105, -6, 55};
  int targetJoints2[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints2, targetAngles2, 3);
  delay(100);
  moveJoint(3, 80);
  delay(50);
  moveJoint(3, 110);
}
void retractArm() {
  moveJoint(3,60);
  delay(100);
  int targetAngles0[4] = {-120, -6, 0,0};
  int targetJoints0[4] = {0, 1, 2,4};
  moveSelectedJointsSmooth(targetJoints0, targetAngles0, 4);
}

bool isSignalInRange(int signal, int lowerBound, int upperBound, int threshold, int &lastSignal) {
  if (abs(lastSignal - signal) >= threshold) {
    if (signal >= lowerBound && signal <= upperBound) {
      lastSignal = signal;
      return true;
    }
  }
  return false;
}
int v5 = 1000;
int v4 = 1000;
int v7 = 1000;
void loop() {
  Serial.print("Channel4: ");
  Serial.print(channelValues[4]);
  Serial.print(" Channel5: ");
  Serial.print(channelValues[5]);
  Serial.print(" Channel7: ");
  Serial.println(channelValues[7]);

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

  // 机械臂操作
  handleZoneSwitch(7, v7,
                   900, 1100, retractArm,
                   1900,2100, DownArm);
  handleZoneSwitch(4, v4,
                   900,1100, retractArm,
                   1900,2100, Down2);
  handleZoneSingle(5, v5, 1400, 1600, Put);

  // 底盘旋转
  rotateArm((int)channelValues[0]);
  move((int)channelValues[1], (int)channelValues[3]);

  // 更新信号值
  v7 = (int)channelValues[7];
  v4 = (int)channelValues[4];
  // int v6 = channelValues[6];
  v5 = (int)channelValues[5];
}