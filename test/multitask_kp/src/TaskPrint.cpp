#include "TaskPrint.h"

void Printer(void *pvParameters) {
    Serial.begin(115200);

    while(true){
        for(int i=0; i<100; i++){
            Serial.print(i);Serial.print(" - ");
            Serial.println("Running Task...");
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        vTaskDelay(taskPrint.getIntervalms() / portTICK_PERIOD_MS);
    }
    
}