#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SDA_PIN        17
#define SCL_PIN        16
#define PWM_I2C_ADDR   0x40
#define SERVOMIN       102
#define SERVOMAX       510
#define SERVO_FREQ     50
#define NUM_JOINTS     4
const int PPM_PIN            = 2;
const int SYNC_GAP           = 3000;
const int MIN_PULSE_WIDTH    = 900;
const int MAX_PULSE_WIDTH    = 2100;
const int NUM_CHANNELS       = 10;
#define PWM_FREQ       1000
#define PWM_RES        8
const int CENTER             = (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2;
const int DEADZONE           = 100;

volatile unsigned long lastTime            = 0;
volatile int          currentChannel      = 0;
volatile unsigned long channelValues[NUM_CHANNELS];

const int motorL1A = 21, motorL1B = 47, motorL2A = 19, motorL2B = 20;
const int motorR1A = 0,  motorR1B = 35, motorR2A = 48, motorR2B = 45;
const int channelLA = 0, channelLB = 1, channelRA = 2, channelRB = 3;

const int pwmFreq       = 5000;
const int pwmResolution = 8;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);

int AngleNow[NUM_JOINTS];

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

void retractArm() {
  moveJoint(3, 60);
  delay(100);
  int targetAngles0[3] = {-120, -6, 0};
  int targetJoints0[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints0, targetAngles0, 3);
}

void DownArm() {
  int targetAngles1[4] = {40, -6, -35, 120};
  int targetJoints1[4] = {0, 1, 2, 3};
  moveSelectedJointsSmooth(targetJoints1, targetAngles1, 4);
}

void Put() {
  int targetAngles2[3] = {-95, -6, 47};
  int targetJoints2[3] = {0, 1, 2};
  moveSelectedJointsSmooth(targetJoints2, targetAngles2, 3);
  delay(100);
  moveJoint(3, 80);
  delay(200);
  moveJoint(3, 120);
}

void Down2() {
  int targetAngles3[4] = {-10, -6, 80, 120};
  int targetJoints3[4] = {0, 1, 2, 3};
  moveSelectedJointsSmooth(targetJoints3, targetAngles3, 4);
}

void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long duration    = currentTime - lastTime;
  lastTime = currentTime;

  if (duration > SYNC_GAP) {
    currentChannel = 0;
  } else if (duration >= MIN_PULSE_WIDTH && duration <= MAX_PULSE_WIDTH &&
             currentChannel < NUM_CHANNELS) {
    channelValues[currentChannel++] = duration;
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

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  // 初始位置
  pwm.setPWM(0, 0, angleToPulse(-120));
  pwm.setPWM(1, 0, angleToPulse(-6));
  pwm.setPWM(2, 0, angleToPulse(0));
  pwm.setPWM(3, 0, angleToPulse(120));
  pwm.setPWM(4, 0, angleToPulse(0));
  pwm.setPWM(5, 0, angleToPulse(15));

  AngleNow[0] = 40;
  AngleNow[1] = -6;
  AngleNow[2] = -35;
  AngleNow[3] = 120;

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // 初始化为 0
  for (int i = 0; i < NUM_CHANNELS; i++) channelValues[i] = 0;

  ledcSetup(channelLA, pwmFreq, pwmResolution);
  ledcSetup(channelLB, pwmFreq, pwmResolution);
  ledcSetup(channelRA, pwmFreq, pwmResolution);
  ledcSetup(channelRB, pwmFreq, pwmResolution);

  ledcAttachPin(motorL1A, channelLA);
  ledcAttachPin(motorL1B, channelLB);
  ledcAttachPin(motorL2A, channelLA);
  ledcAttachPin(motorL2B, channelLB);
  ledcAttachPin(motorR1A, channelRA);
  ledcAttachPin(motorR1B, channelRB);
  ledcAttachPin(motorR2A, channelRA);
  ledcAttachPin(motorR2B, channelRB);
}

void loop() {
  // 检查信号
  bool noSignal = true;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channelValues[i] != 0) { noSignal = false; break; }
  }
  if (noSignal) {
    stop();
    return;
  }

  int v7 = channelValues[7];
  int v4 = channelValues[4];
  int v6 = channelValues[6];
  int v5 = channelValues[5];


  // 机械臂操作
  handleZoneSwitch(7, v7,
                   900, 1100, retractArm,
                   1900,2100, DownArm);
  handleZoneSwitch(4, v4,
                   900,1100, retractArm,
                   1900,2100, Down2);
  handleZoneSingle(5, v5, 1400, 1600, Put);

  // 底盘驱动
  int adv   = channelValues[1] - CENTER;
  int turn  = channelValues[3] - CENTER;
  if (abs(adv)  < DEADZONE) adv  = 0;
  if (abs(turn) < DEADZONE) turn = 0;

  int maxRange   = 500;
  int advSpeed   = map(adv,  -maxRange, maxRange, -230, 230);
  int turnSpeed  = map(turn, -maxRange, maxRange, -230, 230);

  if (abs(advSpeed) > abs(turnSpeed)) {
    if (adv >= 0) {
      ledcWrite(channelLA, 0);
      ledcWrite(channelLB, advSpeed/2);
      ledcWrite(channelRA, advSpeed/2);
      ledcWrite(channelRB, 0);
    } else {
      ledcWrite(channelLA, -advSpeed/2);
      ledcWrite(channelLB, 0);
      ledcWrite(channelRA, 0);
      ledcWrite(channelRB, -advSpeed/2);
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