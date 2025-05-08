#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class TaskManager
{
public:
    // Constructor and Destructor
    TaskManager();
    ~TaskManager();

    // Configure the task to run on a specific core
    void taskCreation(TaskFunction_t taskFunction,
                   const String &taskName,
                   uint32_t stackDepth,
                   uint32_t intervalms,
                   uint8_t priority,
                   uint8_t core);

    // Getter for intervalMs
    uint32_t getIntervalms() const;

    // Get the last execution time of the task
    TickType_t getLastExecutionTime() const;

private:
    TaskFunction_t taskFunction;   // Function to run as a task
    String taskName;               // Name of the task
    uint32_t stackDepth;           // Stack depth for the task
    uint32_t intervalms;           // Interval between executions in ms
    uint8_t priority;              // Task priority
    uint8_t core;                  // Core where the task will run (0 or 1)
    TaskHandle_t handler;          // Task handle for FreeRTOS
    TickType_t lastExecutionTime;  // Stores the last execution time in FreeRTOS ticks

    // Function to handle task sleep
    void sleepTask();
};

#endif // TASKMANAGER_H
