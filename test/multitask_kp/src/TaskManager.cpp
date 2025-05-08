#include "TaskManager.h"

// Constructor and Destructor
TaskManager::TaskManager() {}

TaskManager::~TaskManager() {}

void TaskManager::taskCreation(TaskFunction_t _function,
                            const String &_name,
                            uint32_t _stackDepth,
                            uint32_t _sleep,
                            uint8_t _priority,
                            uint8_t _core)
{
    taskFunction = _function;
    taskName = _name;
    stackDepth = _stackDepth;
    intervalms = _sleep;
    priority = _priority;
    core = _core;

    // Initialize the last execution time with the current time
    lastExecutionTime = xTaskGetTickCount();

    // Create the task
    xTaskCreatePinnedToCore([](void *param)
                            {
                                TaskManager *task = (TaskManager *)param;
                                while (true)
                                {
                                    // Execute the assigned task function
                                    task->taskFunction(NULL);

                                    // Update last execution time
                                    task->lastExecutionTime = xTaskGetTickCount();

                                    // Handle task sleep (interval delay)
                                    task->sleepTask();
                                }
                            },
                            taskName.c_str(),  // Use c_str to convert String to const char*
                            stackDepth,
                            this,
                            priority,
                            &handler,
                            core);
}

uint32_t TaskManager::getIntervalms() const
{
    return intervalms;
}

// Get the last execution time of the task
TickType_t TaskManager::getLastExecutionTime() const
{
    return lastExecutionTime;
}

// Handle the sleep of the task based on the last execution time
void TaskManager::sleepTask()
{
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t timeSinceLastExecution = currentTime - lastExecutionTime;
    TickType_t delayTime = pdMS_TO_TICKS(intervalms);

    // Calculate the remaining time until the next execution
    if (timeSinceLastExecution < delayTime)
    {
        vTaskDelay(delayTime - timeSinceLastExecution);
    }
    else
    {
        // If we're behind schedule, start immediately without delay
        vTaskDelay(0);
    }
}