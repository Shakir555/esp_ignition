//(c) 2024 Shakir Salam
//This software is released under an open-source license, 
//Allowing unrestricted use and modification
//Distribution in both source and binary forms. No rights reserved.

//Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

//TAG
#define TAG "ADC"

//TaskHandle_t Definitions
static TaskHandle_t adc_task_handle = NULL;

//ADC General Macros
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_CHAN0      ADC_CHANNEL_2
#endif

//ADC Variables and Function
static int adc_raw[2][10];
static int voltage[2][10];
static bool adc_calibration_init(adc_unit_t unit,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

//ADC Calibration
static bool adc_calibration_init(adc_unit_t unit,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret   = ESP_FAIL;
    bool calibrated = false;
//ADC_CALI_SCHEME_CURVER_FITTING_SUPPORTED
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "curve fitting");
        adc_cali_curve_fitting_config_t cali_config =
        {
            .unit_id  = unit,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif
//ADC_CAL_SCHEME_LINE_FITTING_SUPPORTED
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config =
        {
            .unit_id  = unit,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif 
    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGI(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg no memory");
    }
    return calibrated;
}

//ADC Calibration Deinitialize
static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregister %s calibration scheme", "curve fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

//TaskHandle_t for adc_task
static void adc_task(void *pvParam)
{
    adc_oneshot_unit_handle_t adc_channel1_handle;
    adc_oneshot_unit_init_cfg_t adc_init_channel1_config =
    {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_channel1_config,
                                         &adc_channel1_handle));
    //ADC1 Config
    adc_oneshot_chan_cfg_t config =
    {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_channel1_handle,
                                               ADC1_CHAN0,
                                               &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_channel1_handle,
                                               ADC1_CHAN0,
                                               &config));
    //ADC1 Calibration Init
    adc_cali_handle_t adc_channel1_cali_handle = NULL;
    bool adc_channel1_do_calibration = adc_calibration_init(ADC_UNIT_1,
                                                            ADC_ATTEN_DB_11,
                                                            &adc_channel1_cali_handle);
    for (;;)
    {
        //ADC1_CHAN0
        ESP_ERROR_CHECK(adc_oneshot_read(adc_channel1_handle,
                                         ADC1_CHAN0,
                                         &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data %d",
                                       ADC_UNIT_1 + 1,
                                       ADC1_CHAN0,
                                       adc_raw[0][0]);
        if (adc_channel1_do_calibration)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_channel1_cali_handle,
                                                    adc_raw[0][0],
                                                    &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV",
                                                   ADC_UNIT_1 + 1,
                                                   ADC1_CHAN0,
                                                   voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_channel1_handle));
    if (adc_channel1_do_calibration)
    {
        adc_calibration_deinit(adc_channel1_cali_handle);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    //Create ADC Task
    xTaskCreate(adc_task, "adc_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, &adc_task_handle);
    //Main Task can do other things or just vTaskDelete(NULL) to exit
    vTaskDelay(portMAX_DELAY);
}
