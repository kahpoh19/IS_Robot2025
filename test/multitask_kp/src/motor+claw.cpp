

const int CENTER = (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2;
const int DEADZONE = 100;


void setup() {
  
  AngleNow[0] = 40;
  AngleNow[1] = -6;
  AngleNow[2] = -35;
  AngleNow[3] = 120;

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
  // Serial.print("Channel2: ");
  // Serial.print(channelValues[1]);
  // Serial.print(" Channel4: ");
  // Serial.println(channelValues[3]);

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
