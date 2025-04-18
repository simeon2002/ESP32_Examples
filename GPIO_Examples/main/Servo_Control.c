#include <stdio.h>
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifndef DEBUG
#define DEBUG 1
#endif

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

static const char *TAG = "Servo motor";
static TaskHandle_t servo_button_task_handle = NULL;

/**
 * conversion of angle to duty cycle value based on given duty cycle resolution.
 */
int angle_to_duty(float angle)
{
    float slope = (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE) / 180.0f;
    int duty = (int)round(SERVO_MIN_DUTY_CYCLE + slope * angle);
    float period_ms = 1000.0f / SERVO_LEDC_FREQUENCY;
    float pulse_width_ms = ((float)duty / (1 << SERVO_LEDC_DUTY_RES)) * period_ms;
    ESP_LOGI(TAG, "Duty cycle value %d for angle %.2f, pulse width ~%.2f ms", duty, angle, pulse_width_ms);
    return duty;
}

/**
 * servo button pressed causing task to start. note: only possible to press button once during one directional movement. (subsequent interrupts won't be resigstered.)
 */
void IRAM_ATTR servo_button_isr_handler(void *args)
{
    vTaskResume(servo_button_task_handle);
}

/**
 * Task to run when button to move servo is pressed.
 */
void servo_button_task(void *args)
{
    int duty;
    for (;;)
    {
        vTaskSuspend(servo_button_task_handle);
        duty = angle_to_duty(180.0f);
        ESP_ERROR_CHECK(ledc_set_fade_with_time(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty, 750));
        ESP_ERROR_CHECK(ledc_fade_start(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));

        vTaskDelay(pdMS_TO_TICKS(5000));

        duty = angle_to_duty(0.0f);
        ESP_ERROR_CHECK(ledc_set_fade_with_time(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty, 750));
        ESP_ERROR_CHECK(ledc_fade_start(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));
    }
}

/**
 * Setup of servo motor
 */
void servo_init()
{
    // timer configuration
    ledc_timer_config_t servo_timer_config = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_config));

    // channel configuration
    ledc_channel_config_t servo_channel_config = {
        .gpio_num = PIN_SERVO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .intr_type = SERVO_LEDC_INTERRUPT,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = angle_to_duty(0),
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel_config));

    // install fade function
    ledc_fade_func_install(0);
}

/**
 * Set up configuration for the servo button with ISR included.
 */
void servo_button_init()
{
    gpio_reset_pin(PIN_BTN_SERVO);
    gpio_set_direction(PIN_BTN_SERVO, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_BTN_SERVO, GPIO_INTR_NEGEDGE);
    gpio_set_pull_mode(PIN_BTN_SERVO, GPIO_PULLUP_ONLY);
    gpio_isr_handler_add(PIN_BTN_SERVO, servo_button_isr_handler, NULL); // set up isr for the servo button.
}

void app_main(void)
{
    // enable per-gpio interrupt service engine
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // initialize servo motor.
    servo_init();
    // init servo button
    servo_button_init();

    // tasks
    xTaskCreate(servo_button_task,
                "servo_button_task",
                2048,
                NULL,
                10,
                &servo_button_task_handle);
}
