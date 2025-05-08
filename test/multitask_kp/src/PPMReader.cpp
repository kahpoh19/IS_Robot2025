#include "PPMReader.h"

const int PPM_PIN = 2;              // Pin connected to the PPM signal
const int SYNC_GAP = 3000;          // Minimum time (in microseconds) to consider as a sync gap
const int MIN_PULSE_WIDTH = 900;    // Minimum pulse width (in microseconds) for a valid signal
const int MAX_PULSE_WIDTH = 2100;   // Maximum pulse width (in microseconds) for a valid signal

volatile unsigned long lastTime = 0;
volatile int currentChannel = 0;

void RXRead(void *pvParameters) {
    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

    // // Print the most recent values for each channel
    // for (int i = 0; i < NUM_CHANNELS; i++) {

    //     Serial.print(channelValues[i]);
    //     Serial.print(" \t");
    // }
    // Serial.println();
    vTaskDelay(taskRXRead.getIntervalms() / portTICK_PERIOD_MS);  // Short delay to limit serial output rate
}

void IRAM_ATTR ppmInterruptHandler() {
    unsigned long currentTime = micros();
    unsigned long duration = currentTime - lastTime;
    lastTime = currentTime;

    if (duration > SYNC_GAP) {
        // Detected sync pulse, reset to the first channel
        currentChannel = 0;
    } else if (duration >= MIN_PULSE_WIDTH && duration <= MAX_PULSE_WIDTH && currentChannel < NUM_CHANNELS) {
        // If duration is within valid range, store it as a channel value
        channelValues[currentChannel] = duration;
        currentChannel++;
    }
}