#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const int ADDRESS = 0x22;

typedef struct {
  pin_t pin_int;  
  // Storage for the 6-byte sensor reading that will be returned.
  // Bytes 0-1: temperature ticks, byte 2: CRC for temperature,
  // Bytes 3-4: humidity ticks,   byte 5: CRC for humidity.
  uint8_t sensor_data[6];
  int sensor_data_index;  // keeps track of which byte should be sent next.
} chip_state_t;

// Function prototypes.
static bool on_i2c_connect(void *user_data, uint32_t address, bool connect);
static uint8_t on_i2c_read(void *user_data);
static bool on_i2c_write(void *user_data, uint8_t data);
static void on_i2c_disconnect(void *user_data);

// The CRC algorithm from the SHT4x datasheet.
// Polynomial 0x31 (x^8 + x^5 + x^4 +1), init 0xFF.
static uint8_t calc_crc(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
    
  // Initialize sensor data index to 6 so that if no measurement is pending,
  // reads will return 0.
  chip->sensor_data_index = 6;
  
  const i2c_config_t i2c_config = {
    .user_data = chip,
    .address = ADDRESS,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect, // Optional
  };
  i2c_init(&i2c_config);

  printf("Initializing simulated SHT40 sensor!\n");
}

// Helper: generate sensor data when a measurement command is received.
static void generate_sensor_reading(chip_state_t *chip) {
  // Choose a random temperature and humidity in realistic ranges.
  // For example, temperature between 10°C and 35°C, and relative humidity between 20% and 80%.
  float r1 = (float)rand() / (float)RAND_MAX;  // random float between 0 and 1
  float r2 = (float)rand() / (float)RAND_MAX;
  float temperature = 10.0f + r1 * (35.0f - 10.0f);
  float humidity = 20.0f + r2 * (80.0f - 20.0f);
  
  // Convert temperature to ticks using:
  // t_degC = -45 + 175 * (t_ticks/65535)
  // => t_ticks = ((temperature + 45) / 175) * 65535
  uint16_t t_ticks = (uint16_t)round(((temperature + 45.0f) / 175.0f) * 65535.0f);

  // Convert humidity to ticks using:
  // rh_pRH = -6 + 125 * (rh_ticks/65535)
  // => rh_ticks = ((humidity + 6) / 125) * 65535
  uint16_t rh_ticks = (uint16_t)round(((humidity + 6.0f) / 125.0f) * 65535.0f);

  // Prepare the sensor_data buffer.
  chip->sensor_data[0] = t_ticks >> 8;      // temperature high byte
  chip->sensor_data[1] = t_ticks & 0xFF;      // temperature low byte
  chip->sensor_data[2] = calc_crc(chip->sensor_data, 2); // temperature CRC

  chip->sensor_data[3] = rh_ticks >> 8;       // humidity high byte
  chip->sensor_data[4] = rh_ticks & 0xFF;       // humidity low byte
  chip->sensor_data[5] = calc_crc(chip->sensor_data + 3, 2); // humidity CRC

  // Reset the sensor reading index so that subsequent I2C read calls will
  // return the six bytes in order.
  chip->sensor_data_index = 0;

  // For debugging purposes, print out the measured temperature and humidity.
  printf("New measurement: Temperature = %.2f °C, Humidity = %.2f %%\n", temperature, humidity);

}

bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  // Always acknowledge connection.
  return true;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = user_data;
  // If there is data in our sensor_data buffer,
  // return bytes sequentially.
  if (chip->sensor_data_index < 6) {
    return chip->sensor_data[chip->sensor_data_index++];
  } else {
    // After 6 bytes, simply return 0.
    return 0;
  }
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = user_data;
  // In this SHT40 simulation we expect the host to send a command byte.
  // In real SHT40, for example, the command 0xFD starts a measurement.
  if (data == 0xFD) {
    generate_sensor_reading(chip);
  }
  // For any command we simply acknowledge.
  return true;
}

void on_i2c_disconnect(void *user_data) {
  // No disconnect actions needed.
}