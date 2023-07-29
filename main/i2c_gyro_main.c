/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_GYRO_DEVICE_ADDR     0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU6050_ACCEL_XOUT_REG  0x3B        /*!< Register address to read accelerometer X-axis output */
#define MPU6050_ACCEL_YOUT_REG  0x3D        /*!< Register address to read accelerometer X-axis output */
#define MPU6050_ACCEL_ZOUT_REG  0x3F        /*!< Register address to read accelerometer X-axis output */
#define MPU6050_TEMPERATURE_REG 0x41        /*!< Register address to read temperature output */

#define RED_LED_GPIO 4
#define GREEN_LED_GPIO 5
#define YELLOW_LED_GPIO 2

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu6050_register_read_16bit(uint8_t reg_addr, uint16_t* value)
{
    esp_err_t err;
    uint8_t data[4];
    // ESP-IDF already has a convenience function for this. Hopefully it will: 
    //    write device address, write desired register address,  stop, start, send "read" device address, read MSB data, continue to LSB data until we NACK.
    memset(data, 0, sizeof(data));
    err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_GYRO_DEVICE_ADDR, &reg_addr, 1, data, 2, 2);
    if (err) { ESP_LOGE(TAG, "mpu6050_register_read_16bit failed i2c_master_write_read_device device x%x reg x%x error x%x",MPU6050_GYRO_DEVICE_ADDR,reg_addr,err); return err; }
    *value = (data[0] << 8) | data[1];
    return ESP_OK;
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{   
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_GYRO_DEVICE_ADDR, write_buf, sizeof(write_buf), 2);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t mpu6050_init(int features)
{
    // set sample rate
    // set full-scale range. Registers 0x1B, 0x1C
    // should calibrate the Factory Trim really.
    // for serious applications, should buffer the data in the FIFO which needs setting up (which features, reset FIFO, sample rate)
    esp_err_t err;
    err = mpu6050_register_write_byte(0x23 /* FIFO_EN */, 0x78);
    if (err) { ESP_LOGE(TAG, "Failed mpu6050_init() > mpu6050_register_write_byte FIFO_EN err x%x", err); return err; }
    err = mpu6050_register_write_byte(0x19 /* SMPRT_DIV */, 0x10);
    if (err) { ESP_LOGE(TAG, "Failed mpu6050_init() > mpu6050_register_write_byte SMPRT_DIV err x%x", err); return err; }
    err = mpu6050_register_write_byte(0x1C /* ACCEL_CONFIG */, 0x00);
    err = mpu6050_register_write_byte(0x6B /* PWR_MGMT_1 */, 0x00); // disable wake sleep CYCLE mode in PWR_MGMT_1
    err = mpu6050_register_write_byte(0x6C /* PWR_MGMT_2 */, 0x40); // LP_WAKE_CTRL at 5Hz for wake sleep CYCLE 
    return ESP_OK;
}

/* Fade green LED on GPIO5 according to some 12ish bit input value */
static esp_err_t led_init()
{  ledc_channel_config_t cconfig = {
        .channel = LEDC_CHANNEL_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0, // hpoint is the duty cycle start point, which means the duty cycle will be hpoint/2**duty_resolution (this comment was auto generated by GitHub CoPilot X)
        .duty = 0,
        .gpio_num = GREEN_LED_GPIO
    };
    ledc_channel_config(&cconfig);
    cconfig.gpio_num = RED_LED_GPIO;
    cconfig.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&cconfig);
    cconfig.gpio_num = YELLOW_LED_GPIO;
    cconfig.channel = LEDC_CHANNEL_2;
    ledc_channel_config(&cconfig);
    ledc_timer_config_t tconfig = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tconfig);
    tconfig.timer_num = LEDC_TIMER_1;
    ledc_timer_config(&tconfig);
    tconfig.timer_num = LEDC_TIMER_2;
    ledc_timer_config(&tconfig);
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 1000); // 1kHz
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;
    int16_t valueY, valueX, valueZ;
    int sampleCounter = 0;
    int maxAccelX = -(1<<30), minAccelX = (1<<30);
    int maxAccelY = -(1<<30), minAccelY = (1<<30);
    int maxAccelZ = -(1<<30), minAccelZ = (1<<30);
    int dutyX=0, dutyY=0, dutyZ=0;
    err = i2c_master_init();
    if (err) { ESP_LOGE(TAG, "i2c_master_init failed"); return; }
    mpu6050_init(0x01); /* enable accelerometer */
    led_init();
    ESP_LOGI(TAG, "Fading up LED ?");
    for (int i = 0; i < (1<<13); i+=64)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, i);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "LED faded on accelerometer Y");
    while (true) // should use button to change mode.
    { 
        /* Read the MP6050  X-axis acceleration register, which is 16 bits*/
        err = mpu6050_register_read_16bit(MPU6050_ACCEL_XOUT_REG, (uint16_t*)&valueX);
        err |= mpu6050_register_read_16bit(MPU6050_ACCEL_YOUT_REG, (uint16_t*)&valueY);
        err |= mpu6050_register_read_16bit(MPU6050_ACCEL_ZOUT_REG, (uint16_t*)&valueZ);
        if (err) { ESP_LOGE(TAG, "Error around mpu6050_register_read_16bit might be x%x",err); return; }
        if (valueX < minAccelX) minAccelX = valueX;
        if (valueX > maxAccelX) maxAccelX = valueX;
        if (valueY < minAccelY) minAccelY = valueY;
        if (valueY > maxAccelY) maxAccelY = valueY;
        if (valueZ < minAccelZ) minAccelZ = valueZ;
        if (valueZ > maxAccelZ) maxAccelZ = valueZ;
        if (sampleCounter-- <= 0)
        { ESP_LOGI(TAG,"Accel Y   = %04x %8d max=%d min=%d duty=%d", (unsigned int)valueY, valueY,maxAccelY,minAccelY,dutyY);
          sampleCounter = 20;
        }
        if (maxAccelX > minAccelX) // avoid divide by zero (or negative
        { dutyX = (valueX - minAccelX) * (1<<LEDC_TIMER_12_BIT) / (maxAccelX - minAccelX);
          ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyX); // expect 12 bit range
          ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }
        if (maxAccelY > minAccelY) // avoid divide by zero (or negative
        { dutyY = (valueY - minAccelY) * (1<<LEDC_TIMER_12_BIT) / (maxAccelY - minAccelY);
          ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyY); // expect 12 bit range
          ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }
        if (maxAccelZ > minAccelZ) // avoid divide by zero (or negative
        { dutyZ = (valueZ - minAccelZ) * (1<<LEDC_TIMER_12_BIT) / (maxAccelZ - minAccelZ);
          ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyZ); // expect 12 bit range
          ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        }
        /*
        err = mpu6050_register_read_16bit(MPU6050_TEMPERATURE_REG, &value);
        ESP_LOGI(TAG,"Temperature = %04x %08d", (unsigned int)value, (int)*psvalue);
        */
        vTaskDelay(10); // wait 100ms
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
