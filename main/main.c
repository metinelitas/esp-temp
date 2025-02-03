#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM             I2C_NUM_0   // I2C port number
#define I2C_MASTER_SDA_IO          14          // SDA pin
#define I2C_MASTER_SCL_IO          9           // SCL pin
#define I2C_MASTER_FREQ_HZ         100000      // I2C clock frequency
#define I2C_MASTER_TX_BUF_DISABLE  0           // No TX buffer for master
#define I2C_MASTER_RX_BUF_DISABLE  0           // No RX buffer for master

#define CONFIG_LOG_DEFAULT_LEVEL_VERBOSE 1
#define CONFIG_LOG_MAXIMUM_LEVEL  5

#define SLAVE_ADDR                 0x22        // Slave I2C address

static const char *TAG = "ESP32";

static esp_err_t i2c_master_read_slave(uint8_t *data, size_t len);

/*
 * Initialize the I2C master interface.
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }

    // Confirm that the I2C slave is connected to the bus
    uint8_t data;
    err = i2c_master_read_slave(&data, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave not found: %s", esp_err_to_name(err));
    } 
    else { 
        ESP_LOGI(TAG, "I2C slave found");
    }
    return err;
}

/*
 * Read a sequence of bytes from the I2C slave device.
 * data : pointer to a buffer to store the read bytes.
 * len  : number of bytes to read.
 */
static esp_err_t i2c_master_read_slave(uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    
    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    // Write the slave address with the read bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute the command link with a 1000 ms timeout
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_write_slave(uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    
    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Send a start condition
    i2c_master_start(cmd);
    
    // Send the slave address with write bit (I2C_MASTER_WRITE)
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    // Write the data bytes to the I2C bus
    i2c_master_write(cmd, data, len, true);
    
    // Send a stop condition
    i2c_master_stop(cmd);
    
    // Execute the command link with 1000 ms timeout
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    
    // Free the command link resources
    i2c_cmd_link_delete(cmd);
    
    return ret;
}


void i2c_read_task(void *arg)
{
    // Buffer to store the six bytes from sensor (temp high, temp low, CRC, rh high, rh low, CRC)
    uint8_t sensor_data[6];
    uint8_t command = 0xFD;  // Measurement command for the simulated SHT40

    while (1) {
        esp_err_t ret;

        // First, write the measurement command (0xFD) to the slave
        ret = i2c_master_write_slave(&command, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error writing command to slave: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue; 
        }

        // Wait at least 10ms for measurement to complete on the slave side
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read 6 bytes of data from the slave
        ret = i2c_master_read_slave(sensor_data, 6);
        if (ret == ESP_OK) {
            // Extract temperature ticks (first two bytes) and humidity ticks (bytes 4 and 5)
            uint16_t t_ticks = ((uint16_t)sensor_data[0] << 8) | sensor_data[1];
            uint16_t rh_ticks = ((uint16_t)sensor_data[3] << 8) | sensor_data[4];

            // Convert ticks to actual values using the formulas from the SHT40 datasheet:
            // Temperature (°C) = -45 + 175 * (t_ticks / 65535)
            float temperature = -45.0f + (175.0f * ((float)t_ticks / 65535.0f));

            // Relative Humidity (% pRH) = -6 + 125 * (rh_ticks / 65535)
            float humidity = -6.0f + (125.0f * ((float)rh_ticks / 65535.0f));
            // Clamp the humidity value between 0 and 100 %
            if (humidity < 0) {
                humidity = 0.0f;
            }
            if (humidity > 100) {
                humidity = 100.0f;
            }
            
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "Error reading from slave: %s", esp_err_to_name(ret));
        }
        // Wait 2 seconds before the next measurement cycle
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    // Initialize I2C master
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Create a FreeRTOS task to read I2C data every 2 seconds
    xTaskCreate(i2c_read_task, "i2c_read_task", 4096, NULL, 5, NULL);
}