#include "main.h"

// Task function
// void myTask(void *pvParameters)
// {

//     analogWrite(ledpin,map(channelValues[0],950,2005,0,255));
//     vTaskDelay(50);

// }

void setup() {
  Serial.begin(115200);

  // Configure the task to run on Core 0
  // task.taskCreation(Function, Name, Size, sleepTime, priority (0-5), core);
  taskRGBLed.taskCreation(RGBLed, "Core0_blinkRGB", 2048, 50, 2,
                          0); // External Task
  taskRXRead.taskCreation(RXRead, "Core0_RXReader", 2048, 20, 5, 0);
  // taskPWMExt.taskCreation(PWMExtender,"Core0_PWMExtender",2048,50,3,0);

  // Configure the task to run on Core 1
  // taskManager.taskCreation(myTask, "Core1_Task", 2048, 500, 3, 1); // Task
  // created on main.cpp taskPrint.taskCreation(Printer, "Core1_Printer", 2048,
  // 100,1,1);
  taskMotorDriving.taskCreation(MotorDriving, "Core1_MotorDriving", 4096, 50, 5,
                                0);
  taskServot1.taskCreation(Servot1, "Core0_sweepServo", 2048, 500, 2, 1);
}

void loop() {
  // Nothing in loop, tasks are handled by FreeRTOS
}
