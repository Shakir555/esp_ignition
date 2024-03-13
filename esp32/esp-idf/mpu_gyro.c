//(c) 2024 Shakir Salam
//This software is released under an open-source license, 
//Allowing unrestricted use and modification
//Distribution in both source and binary forms. No rights reserved.

//Libraries
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include <stdio.h>

//MPU6050 TAG
#define TAG "MPU6050"

//MPU6050 Definition
#define who_Am_I               0x75
#define I2C_MASTER_FREQ_HZ     100000
#define MPU6050_I2C_ADDR       0x70
#define MPU6050_I2C_CH1        0x01
#define MPU6050_ADD            0x68
#define MPU6050_PWR            0x6B
#define MPU6050_RAW_GYRO       0x43
#define MPU6050_ACK_VAL        0x1
#define MPU6050_NACK_VAL       0x0

//RGB LED Definition
#define RGB_RED_LED_GPIO    25
#define RGB_GREEN_LED_GPIO  26
#define RGB_BLUE_LED_GPIO   27

//TaskHandle_t LED
TaskHandle_t rgb_red_led_task_handle   = NULL;
TaskHandle_t rgb_green_led_task_handle = NULL;
TaskHandle_t rgb_blue_led_task_handle  = NULL;

//MPU6050 Gyro Definition
uint8_t gyroXH;
uint8_t gyroXL;
uint8_t gyroYH;
uint8_t gyroYL;
uint8_t gyroZH;
uint8_t gyroZL;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

//MPU6050 
uint8_t mpu6050_bits;

//Configure GPIO Pins
void rgb_led_gpio_conf()
{
    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL << RGB_RED_LED_GPIO)    |
                        (1ULL << RGB_GREEN_LED_GPIO)  |
                        (1ULL << RGB_BLUE_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
}

//Blue LED Blinking when gyro encounter > 20 degree
void rgb_blue_led_task(void *pvParam)
{

    int count = 0;
    //Configuration for Blue LED
    gpio_set_direction(RGB_BLUE_LED_GPIO, GPIO_MODE_OUTPUT);
    while(count < 1)
    {
        ESP_LOGI(TAG, "RGB BLUE LED Turn On");
        gpio_set_level(RGB_BLUE_LED_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(RGB_BLUE_LED_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        count++;
    }
    vTaskDelete(NULL);
}

//Configure I2C GPIO Configuration
void i2c_gpio_conf()
{
    //Configure i2c controller 0 in master mode, normal speed
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num       = 21;
    conf.scl_io_num       = 22;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_LOGI(TAG, "I2C Controller Configured\r\n");
    //Install the driver
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C Driver installed\r\n");
}

//i2c channel initialization
void mpu6050_i2c_channel(int mux_channel, int mpu6050_channel)
{
    //Create and execute command link 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mux_channel << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mpu6050_channel, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK)
    {
        ESP_LOGI(TAG, "Channel 0x%02x I2c Mux Selected", mpu6050_channel);
    }
    else
    {
        ESP_LOGI(TAG, "MPU6050 i2c Channel is not connected");
    }
    i2c_cmd_link_delete(cmd);
}

//MPU6050 Initialization
void mpu6050_init(int mpu6050, int mpu6050_channel)
{
    //Create and execute command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR, true);
    i2c_master_write_byte(cmd, 0x0, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK)
    {
        ESP_LOGI(TAG, "MPU6050 Channel 0x%02x Initialized", mpu6050_channel);
    }
    else
    {
        ESP_LOGI(TAG, "MPU6050 is not connected");
    }
    i2c_cmd_link_delete(cmd);
}

//MPU6050 Address
uint8_t mpu6050_whoAMI(int mpu6050)
{
    uint8_t mpu6050_buffer;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, who_Am_I, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &mpu6050_buffer, MPU6050_NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, (1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return mpu6050_buffer;
}

//MPU6050 Get Raw Gyro
void mpu6050_getRawGyro(int mpu6050)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_RAW_GYRO, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mpu6050 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &gyroXH, MPU6050_ACK_VAL);
    i2c_master_read_byte(cmd, &gyroXL, MPU6050_ACK_VAL);
    i2c_master_read_byte(cmd, &gyroYH, MPU6050_ACK_VAL);
    i2c_master_read_byte(cmd, &gyroYL, MPU6050_ACK_VAL);
    i2c_master_read_byte(cmd, &gyroZH, MPU6050_ACK_VAL);
    i2c_master_read_byte(cmd, &gyroZL, MPU6050_NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0,cmd, (1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    //Gyro Degrees
    //Convert raw values to degree per second(dps)
    gyroX = (gyroXH << 8 | gyroXL);
    gyroY = (gyroYH << 8 | gyroYL);
    gyroZ = (gyroZH << 8 | gyroZL);
    //Sensitivity factor for ±250 dps range
    float sensitivity = 131.0;
    double fmode(double x, double y);
    gyroX = gyroX / sensitivity;
    gyroY = gyroY / sensitivity;
    gyroZ = gyroZ / sensitivity;
    //Ensure values are in the range -180 to 180 degrees
    gyroX = fmod(gyroX + 180.0, 360.0) - 180.0;
    gyroY = fmod(gyroY + 180.0, 360.0) - 180.0;
    gyroZ = fmod(gyroZ + 180.0, 360.0) - 180.0;
    //Printing Gyro LOG
    ESP_LOGI(TAG, "gyroX: %0.2f degrees/s", (double)gyroX);
    ESP_LOGI(TAG, "gyroY: %0.2f degrees/s", (double)gyroY);
    ESP_LOGI(TAG, "gyroZ: %0.2f degrees/s", (double)gyroZ);
    //Blue LED Gyro Indication
    //Gyro Values exceed 20 degrees
    //Trigger Blue LED
    if (abs(gyroX) > 20 || abs(gyroY) > 20 || abs(gyroZ) > 20)
    {
        //Gyro Values are not within the specified range
        //Turn Off the Blue LED
        gpio_set_level(RGB_BLUE_LED_GPIO, 1);
        //Start the Blue LED Task
        xTaskCreate(rgb_blue_led_task, "rgb_blue_led_task", 4056, NULL, 6, &rgb_blue_led_task_handle);
    }
    else
    {
        //Gyro Values are not within the specified range
        //Turn Off the Blue LED
        gpio_set_level(RGB_BLUE_LED_GPIO, 0);
    }
}

//MPU6050 Task
void mpu6050_task(void *pvParam)
{
    mpu6050_i2c_channel(MPU6050_I2C_ADDR, MPU6050_I2C_CH1);
    mpu6050_init(MPU6050_ADD, MPU6050_I2C_CH1);

    while(1)
    {
        mpu6050_bits = mpu6050_whoAMI(MPU6050_ADD);
        ESP_LOGI(TAG, "MPU Name: 0x%02x", mpu6050_bits);
        ESP_LOGI(TAG, "*************************\n");
        mpu6050_getRawGyro(MPU6050_ADD);
        ESP_LOGI(TAG, "*************************\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    //i2c GPIO Configuration
    i2c_gpio_conf();
    //Start MPU6050 Task
    xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}
