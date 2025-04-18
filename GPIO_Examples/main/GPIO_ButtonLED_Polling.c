#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "esp_log.h"

void app_main(void)
{
    static const char* TAG = "Blink Example";

    // Initialization GPIO22 button input with internal pull up
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);

    // Initializing GPIO26 as output LED.
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);

    while(true) {
        if (gpio_get_level(GPIO_NUM_12)) {
            gpio_set_level(GPIO_NUM_11, 0);
            ESP_EARLY_LOGI(TAG, "LED is off.");
        } else {
            gpio_set_level(GPIO_NUM_11, 1);
            ESP_EARLY_LOGI(TAG, "LED is on.");
        }
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );  // delay for 10 000 ms (10 s)
    }


}
