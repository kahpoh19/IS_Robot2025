#include "PWMExpander.h"



void PWMExpander(int Channel, int val) {
  // void PWMExtender(void *pvParamenters){
  Wire.begin(SDA_PIN, SCL_PIN, 400000); // SDA,SCL
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // PWM freq 50Hz or T=16,66ms
  pwm.setPWM(0, 0, angleToPulse(40));
  pwm.setPWM(1, 0, angleToPulse(-6));
  pwm.setPWM(2, 0, angleToPulse(-35));
  pwm.setPWM(3, 0, angleToPulse(120));
  AngleNow[0] = 40;
  AngleNow[1] = -6;
  AngleNow[2] = -35;
  AngleNow[3] = 120;
  // while(true){
  //     drivePWM.setPWM(0, 0, pulseWidth(channelValues[1]));
  //     drivePWM.setPWM(1, 0, pulseWidth(channelValues[2]));
  //     Serial.println(channelValues[1]);

  //     vTaskDelay(taskPWMExt.getIntervalms() / portTICK_PERIOD_MS);
  // }
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
}

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

// 摇杆控制底盘舵机
void rotateArm(int ch0) {
  int rt = ch0 - 1500;

  int rotate = map(rt, -500, 500, 102, 510);
  if (abs(rotate) < 100) {
    rotate = 0;
  }
  pwm.setPWM(4, 0, rotate);
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