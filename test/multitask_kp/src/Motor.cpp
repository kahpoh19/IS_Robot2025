#include "Motor.h"

void Motor() {
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

void stop() {
  ledcWrite(channelLA, 0);
  ledcWrite(channelLB, 0);
  ledcWrite(channelRA, 0);
  ledcWrite(channelRB, 0);
}
