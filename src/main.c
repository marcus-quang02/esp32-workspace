#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// Define the GPIO pin where the LED is connected
#define BLINK_GPIO GPIO_NUM_2

const char* data = "leminhquang";
void blink_led_task(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void blink_led_task_2(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(18, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(18, 0);
        vTaskDelay(750 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(18, 1);
        vTaskDelay(750 / portTICK_PERIOD_MS);
    }
}

void blink_led_task_3(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(25, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(25, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(25, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void echo_task()
{
    const int uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    

    };
    //Configure UART1 parameters
    uart_param_config(uart_num, &uart_config);
    //Set UART1 pins(TX: IO4, RX: I05)
    uart_set_pin(uart_num, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0);
    while(1) {
        uart_write_bytes(uart_num, "le minh quang\n", 15);
        vTaskDelay(pdMS_TO_TICKS(1000));
        //printf("Hello from usart scope \n");
    }

    
}
void app_main(void)
{
    // Create the blink LED task
    xTaskCreate(&blink_led_task, "blink_led_task", 1024, NULL, 5, NULL);
    // Create the UART task
    xTaskCreate(&blink_led_task_2, "blink_led_task_2", 1024, NULL, 5, NULL);
    xTaskCreate(&blink_led_task_3, "blink_led_task_3", 1024, NULL, 5, NULL);
    xTaskCreate(&echo_task, "usart_task_3", 1024, NULL, 5, NULL);
}


