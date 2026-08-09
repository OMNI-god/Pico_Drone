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
#include "Elrs.h"

void vTaskPrint(void *pvParameters);
void vTaskServo(void *pvParameters);
void vTaskElrs(void *pvParameters);

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

    // xTaskCreate(vTaskPrint, "PrintTask", 1024, nullptr, 1, nullptr);
    // xTaskCreate(vTaskServo, "ServoTask", 2048, &taskData, 1, nullptr);
    xTaskCreate(
        vTaskElrs,
        "ELRS",
        2048,
        nullptr,
        2,
        nullptr);
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

    int angle = 0;
    int direction = 1;

    while (true)
    {
        for (size_t i = 0; i < data->count; ++i)
        {
            data->servos[i].setPosition(angle);
        }

        printf("Angle: %d\n", angle);

        angle += direction;

        if (angle >= 180)
        {
            angle = 180;
            direction = -1;
        }
        else if (angle <= 0)
        {
            angle = 0;
            direction = 1;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void vTaskElrs(void *pvParameters)
{
    Elrs elrs(
        uart1,
        420000,
        5, // RX
        4  // TX
    );

    elrs.initialize();

    uint8_t buffer[64];

    while (true)
    {
        int count = elrs.read(buffer, sizeof(buffer));

        if (count > 0)
        {
            printf("Received %d bytes: ", count);

            for (int i = 0; i < count; i++)
            {
                printf("%02X ", buffer[i]);
            }

            printf("\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}