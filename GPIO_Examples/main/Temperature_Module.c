#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include <math.h>

// ADC read parameters
#define ADC_READ_LENGTH (256 * sizeof(adc_digi_output_data_t)) // 64 * 32 BYTES
#define ADC_BUFF_SIZE (1 * ADC_READ_LENGTH)                    // 4 ADC FRAMES CAN BE STORED
#define AVG_FRAME_THRESHOLD 1 // Modify to change update time of temperature sensor (approx. 12.5 ms per frame?? I think?)

// Calibration constants
#define CALIBRATION_VOLTAGE 2.518f     // [V] Measured LM335 voltage at known temp
#define CALIBRATION_TEMPERATURE 24.9f // [°C] Known temperature during calibration
#define SATURATION_VOLTAGE 3.057f     // Measured by using 5V input voltage to ADC channel

// EMA FILTER MACRO CONSTANTS
#define ALPHA_SLOW 0.1f
#define ALPHA_MEDIUM 0.3f
#define ALPHA_FAST 0.6f
#define TEMP_FAST_CHANGE_THRESHOLD 3
#define TEMP_MEDIUM_CHANGE_THRESHOLD 2

float voltage_avg;
float temp_avg;
float smoothed_temp;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
TaskHandle_t adc_task_handle;
static const char *TAG = "Temperature Sensor";

/**
 * Converts measured voltage (V) to temperature (°C) using slope from calibration
 */
static float voltage_to_celcius(float voltage)
{
    float slope = (273.15f + CALIBRATION_TEMPERATURE) / CALIBRATION_VOLTAGE;
    return voltage * slope - 273.15f; // Kelvin to Celsius
}

/**
 * samples_count: sample count from the samples buffer
 * total_samples_count: total sample count across buffer.
 */
static void process_adc_frame(adc_digi_output_data_t *samples, uint32_t samples_length, int *frame_sample_count, float *p_accumulated_voltage) {
    int valid_sample_count = 0;
    int voltage_mv = 0;

    for (int i = 0; i < samples_length; i += SOC_ADC_DIGI_DATA_BYTES_PER_CONV)
    {
        uint32_t raw_adc_value = samples[i].type2.data;
        uint32_t chan_num = (uint32_t)samples[i].type2.channel;
        uint32_t unit = (uint32_t)samples[i].type2.unit;

        if (chan_num >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1) || chan_num != ADC_CHANNEL_1) continue; // skip if chan_num doesn't correspond
        if (unit != ADC_UNIT_1) continue;

        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw_adc_value, &voltage_mv));
        // ESP_LOGI(TAG,
        //          "raw adc: %" PRIu32 ", voltage value [mV]: %d, channel: %" PRIu32 ", unit: %" PRIu32,
        //          raw_adc_value,
        //          voltage_mv,
        //          chan_num,
        //          unit);

        *p_accumulated_voltage += ((float)voltage_mv / 1000.0f);
        valid_sample_count++;
    }

    *frame_sample_count += valid_sample_count;
}

static void adc_log_info_average(uint32_t chan_num, uint32_t unit) {
    int gpio_num;
    ESP_ERROR_CHECK(adc_continuous_channel_to_io(ADC_UNIT_1, chan_num, &gpio_num));
    ESP_LOGI(TAG, "GPIO: %d, Unit: %" PRIu32 ", Channel: %" PRIu32 ", Avg Voltage: %.3f V, Avg Temp: %.1f C, Smoothed Temp: %.1f C", gpio_num, unit, chan_num, voltage_avg, temp_avg, smoothed_temp);
}

/**
 * @brief Task for handling ADC data and computing temperature
 * process adc frame to update the average value calculated.
 * todo: add channel validity (check where adc_digi_output_data_t is defined)
 */
static void adc_frame_processing_task(void *arg)
{
    uint8_t frame_result[ADC_READ_LENGTH] = {0};
    uint32_t frame_length;
    int frame_counter = 0;
    int total_samples_counter = 0;
    float accumulated_voltage = 0;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // ESP_LOGI(TAG, "Frame number: %d", frame_counter);
        // Read buffered ADC values
        ESP_ERROR_CHECK(adc_continuous_read(adc_handle, frame_result, ADC_READ_LENGTH, &frame_length, 0));
        adc_digi_output_data_t *temp_samples = (adc_digi_output_data_t *)frame_result;

        process_adc_frame(temp_samples, frame_length, &total_samples_counter, &accumulated_voltage);

        if (frame_counter++ < AVG_FRAME_THRESHOLD) continue; // exit if threshold hasn't been reached yet

        voltage_avg = accumulated_voltage / total_samples_counter;
        temp_avg = voltage_to_celcius(voltage_avg);
        float delta = fabsf(temp_avg - smoothed_temp);
        float alpha = delta > TEMP_FAST_CHANGE_THRESHOLD ? ALPHA_FAST : delta > TEMP_MEDIUM_CHANGE_THRESHOLD ? ALPHA_MEDIUM : ALPHA_SLOW;
        smoothed_temp = smoothed_temp == 0 ? temp_avg : (alpha * temp_avg + (1.0f - alpha) * smoothed_temp);

        
        // for now: only logging it.
        uint32_t chan_num = temp_samples[0].type2.channel; // NOTE: THESE TWO LINES ARE ONLY POSSIBLE WHEN ONE CHANNEL IS WORKING!!!
        uint32_t unit = temp_samples[0].type2.unit;
        adc_log_info_average(chan_num, unit);

        frame_counter = 0;
        total_samples_counter = 0;
        accumulated_voltage = 0;
    }
}

// ISR callback when ADC conversion completes
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    TaskHandle_t task_handle = (TaskHandle_t)user_data;
    vTaskNotifyGiveFromISR(task_handle, &mustYield); // mustYields == must have priority and start now, not the case because it's not time sensitive.
    return (mustYield == pdTRUE);
}

// ADC + Calibration + Interrupt config
static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_handle_config = {
        .max_store_buf_size = ADC_BUFF_SIZE,
        .conv_frame_size = ADC_READ_LENGTH};
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_handle_config, &adc_handle));

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12, // 12 dB = ~3.3V-4.1V input range
        .channel = ADC_CHANNEL_1,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12};

    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW, // 20 kHz for ESP32-S3
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2};

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));

    // implementing register callback to get interrrupt when frame conversion is done.
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, adc_task_handle));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void app_main(void)
{
    // adc task and adc initialization
    xTaskCreate(adc_frame_processing_task, "adc_task", 4096, NULL, 10, &adc_task_handle);
    continuous_adc_init();
}