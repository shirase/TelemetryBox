#include "led.h"
#include <Arduino.h>

#define LED 2

volatile int ledDelay = 500;

void ledTask(void * parameter)
{
    pinMode(LED, OUTPUT);

    for( ;; ) {
        TickType_t start = xTaskGetTickCount();

        digitalWrite(LED, !digitalRead(LED));

        vTaskDelay(ledDelay / portTICK_PERIOD_MS - (xTaskGetTickCount() - start));
    }
}