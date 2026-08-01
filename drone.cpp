#include <cstddef>
#include <cstdio>

#include "pico/stdlib.h"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

#include "ADXL345.h"
#include "ServoESC.h"
#include "Servo.h"

void vTaskPrint(void *pvParameters);
void vTaskServo(void *pvParameters);

struct ServoTaskData
{
    Servo *servos;
    size_t count;
};

int main()
{
    stdio_init_all();

    Servo servos[4] = {Servo(16), Servo(17), Servo(18), Servo(19)};
    ServoTaskData taskData{servos, 4};

    xTaskCreate(vTaskPrint, "PrintTask", 1024, nullptr, 1, nullptr);
    xTaskCreate(vTaskServo, "ServoTask", 2048, &taskData, 1, nullptr);

    vTaskStartScheduler();
}

void vTaskPrint(void *pvParameters)
{
    while (true)
    {
        printf("Task 1: Printing from FreeRTOS!\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskServo(void *pvParameters)
{
    auto *data = static_cast<ServoTaskData *>(pvParameters);

    while (true)
    {
        for (size_t i = 0; i < data->count; ++i)
        {
            data->servos[i].setPosition(30);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}