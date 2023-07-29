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

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           19      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_GYRO_DEVICE_ADDR     0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU6050_ACCEL_XOUT_REG  0x3B        /*!< Register address to read accelerometer X-axis output */
#define MPU6050_TEMPERATURE_REG 0x41        /*!< Register address to read temperature output */
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

void app_main(void)
{
    esp_err_t err;
    uint16_t value;
    int16_t *psvalue = (void*)&value;
    err = i2c_master_init();
    if (err) { ESP_LOGE(TAG, "i2c_master_init failed"); return; }
    mpu6050_init(0x01); /* enable accelerometer */


    while (true) // should use button to change mode.
    { 
        /* Read the MP6050  X-axis acceleration register, which is 16 bits*/
        err = mpu6050_register_read_16bit(MPU6050_ACCEL_XOUT_REG, &value);
        ESP_LOGI(TAG,"Accel Y                                 = %04x %08d", (unsigned int)value, (int)*psvalue);
        err = mpu6050_register_read_16bit(MPU6050_TEMPERATURE_REG, &value);
        ESP_LOGI(TAG,"Temperature = %04x %08d", (unsigned int)value, (int)*psvalue);
        vTaskDelay(20); // wait 20ms
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
