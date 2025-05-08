#include "TaskServo.h"

void Servot1(void *pvParameters) {
    Servo myservo;

    int pos = 95;        // variable to store the servo position
    int servoPin = 9;  

    // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
    
    while(true){
        // if(cservo==0){
        //     myservo.write(95);
        //     vTaskDelay(500);
        //     cservo+=1;
        // }
        if(channelValues[4]>900 && channelValues[4]<1300 && pos<90){
            for (pos = 30; pos <= 95; pos += 1) { // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                myservo.write(pos);    // tell servo to go to position in variable 'pos'
                delay(20);             // waits 20ms for the servo to reach the position
            }
        } else if(channelValues[4]>1700 && channelValues[4]<2100 && pos>35){
            for (pos = 95; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
                myservo.write(pos);    // tell servo to go to position in variable 'pos'
                delay(20);             // waits 20ms for the servo to reach the position
            }
        }

        vTaskDelay(taskServot1.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}