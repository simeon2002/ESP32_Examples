#include <stdio.h>
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#ifndef DEBUG
#define DEBUG 1
#endif

#define PIN_MOTION_SENSOR GPIO_NUM_8
#define PIN_BTN_SERVO GPIO_NUM_9
#define PIN_SERVO GPIO_NUM_10

#define SERVO_MAX_DUTY_CYCLE 116 // ~2ms pulse width (180 degrees) NOTE: TUNED TO GET EXACT 180 DEGREES FOR SPECIFIC SERVO
#define SERVO_MIN_DUTY_CYCLE 25  // ~1ms pulse width (0 degrees) NOTE: TUNED TO GET EXACT 180 DEGREES FOR SPECIFIC SERVO
#define SERVO_LEDC_TIMER LEDC_TIMER_0
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define SERVO_LEDC_FREQUENCY 50 // 20ms period for servo motor.
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_INTERRUPT LEDC_INTR_DISABLE

#define BIT_SERVO_BTN_PRESSED (1 << 0)
#define BIT_MOTION_SENSOR_CLEARED (1 << 1)

static const char *TAG = "Servo motor";
EventGroupHandle_t servo_event_group;
TaskHandle_t servo_button_task_handle = NULL;
volatile bool servo_busy = false; // prevents repeated button ISR triggers

int angle_to_duty(float angle)
{
    float slope = (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE) / 180.0f;
    int duty = (int)round(SERVO_MIN_DUTY_CYCLE + slope * angle);
    float period_ms = 1000.0f / SERVO_LEDC_FREQUENCY;
    float pulse_width_ms = ((float)duty / (1 << SERVO_LEDC_DUTY_RES)) * period_ms;
    ESP_LOGI(TAG, "Duty cycle value %d for angle %.2f, pulse width ~%.2f ms", duty, angle, pulse_width_ms);
    return duty;
}

void IRAM_ATTR servo_button_isr_handler(void *args)
{
    if (!servo_busy) {
        xEventGroupSetBitsFromISR(servo_event_group, BIT_SERVO_BTN_PRESSED, NULL);
        ESP_EARLY_LOGI("ISR", "Button ISR triggered!\n");
    }
}

void IRAM_ATTR motion_sensor_isr_handler(void *args)
{
    xEventGroupSetBitsFromISR(servo_event_group, BIT_MOTION_SENSOR_CLEARED, NULL);
    ESP_EARLY_LOGI("ISR", "Motion sensor ISR triggered!\n");
}

void servo_button_task(void *args)
{
    int duty;
    for (;;)
    {
        ESP_LOGI(TAG, "Waiting for door to open");
        xEventGroupWaitBits(servo_event_group, BIT_SERVO_BTN_PRESSED, pdTRUE, pdFALSE, portMAX_DELAY);
        servo_busy = true;

        duty = angle_to_duty(180.0f);
        ESP_ERROR_CHECK(ledc_set_fade_with_time(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty, 750));
        ESP_ERROR_CHECK(ledc_fade_start(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));

        // required delay of 5 ms.
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Waiting for door to close");
        xEventGroupWaitBits(servo_event_group, BIT_MOTION_SENSOR_CLEARED, pdTRUE, pdFALSE, portMAX_DELAY);

        duty = angle_to_duty(0.0f);
        ESP_ERROR_CHECK(ledc_set_fade_with_time(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty, 750));
        ESP_ERROR_CHECK(ledc_fade_start(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));

        // clear interrupts after servo movement finsihed to remove accidental interrupts.
        xEventGroupClearBits(servo_event_group, BIT_SERVO_BTN_PRESSED);
        servo_busy = false;
    }
}

void motion_sensor_init()
{
    gpio_reset_pin(PIN_MOTION_SENSOR);
    gpio_set_direction(PIN_MOTION_SENSOR, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_MOTION_SENSOR, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(PIN_MOTION_SENSOR, motion_sensor_isr_handler, NULL);
}

void servo_init()
{
    ledc_timer_config_t servo_timer_config = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_config));

    ledc_channel_config_t servo_channel_config = {
        .gpio_num = PIN_SERVO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .intr_type = SERVO_LEDC_INTERRUPT,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = angle_to_duty(0),
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel_config));

    ledc_fade_func_install(0);
}

void servo_button_init()
{
    gpio_reset_pin(PIN_BTN_SERVO);
    gpio_set_direction(PIN_BTN_SERVO, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_BTN_SERVO, GPIO_INTR_NEGEDGE);
    gpio_set_pull_mode(PIN_BTN_SERVO, GPIO_PULLUP_ONLY);
    gpio_isr_handler_add(PIN_BTN_SERVO, servo_button_isr_handler, NULL);
}

void app_main(void)
{
    servo_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    servo_init();
    servo_button_init();
    motion_sensor_init();

    xTaskCreate(servo_button_task,
                "servo_button_task",
                2048,
                NULL,
                10,
                &servo_button_task_handle);
}
