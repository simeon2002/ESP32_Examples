#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "esp_log.h"

#define GPIO_BUTTON GPIO_NUM_12
#define GPIO_LED GPIO_NUM_11

static const char *TAG = "Blink Example";
static TaskHandle_t button_task_handle = NULL;

/**
 * Button ISR used to notify button_task function. Is ISR-safe
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;                         // for priority of task to be called
    vTaskNotifyGiveFromISR(button_task_handle, &higherPriorityTaskWoken); // sends notification to the task handle to put task to ready
    portYIELD_FROM_ISR(higherPriorityTaskWoken);                          // tells CPU to immediately switch to this task without waiting till next tick
}

/**
 * LED button task handler. Blocking when ISR notification is reaches, then toggles LED.
 */
static void button_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
        int button_state = gpio_get_level(GPIO_BUTTON);
        if (button_state) {
            gpio_set_level(GPIO_LED, 0);
            ESP_LOGI(TAG, "LED is off. (after button switch)");
        }
        else {
            gpio_set_level(GPIO_LED, 1);
            ESP_LOGI(TAG, "LED is on. (after button switch)");
        }
    }
}

void app_main(void)
{

    // Initialization GPIO12 button input with internal pull up
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_ANYEDGE);

    // Initializing GPIO11 as output LED.
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // set up button ISR.
    gpio_install_isr_service(0); // initiate gpio 'engine', i.e. ISR dispatcher with one isr vector in cpu int table
    gpio_isr_handler_add(GPIO_BUTTON, button_isr_handler, NULL);
    xTaskCreate(
        button_task,
        "button_task",
        2048,
        NULL,
        10,
        &button_task_handle);
}