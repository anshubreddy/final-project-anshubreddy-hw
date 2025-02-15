#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c-dev.h>

#define BME280_DEVICE_ADDR 0x77 // Default I2C address of the BME280 sensor
#define I2C_NODE           1    // I2C bus number (e.g., I2C bus 1)
#define PASS               0    // Success return code
#define FAIL              -1    // Failure return code
#define MAX_BUFF_LEN       8    // Maximum buffer length for I2C data transfer
#define MAX_STR_LEN       15    // Maximum string length for device path

// Function Prototypes
static int init_bme280_sensor(uint8_t i2c_node);
static void read_bme280_sensor(int file_id, float *temperature, float *humidity, float *pressure);

// Initializes BME280 sensor by configuring I2C slave address
static int init_bme280_sensor(uint8_t i2c_node)
{
    char device_path[MAX_STR_LEN] = {0};
    int file_fd = -1;
    snprintf(device_path, MAX_STR_LEN, "/dev/i2c-%d", i2c_node);
    file_fd = open(device_path, O_RDWR);
    if (FAIL == file_fd)
    {
        syslog(LOG_ERR, "Error opening i2c device %s file: %s", device_path, strerror(errno));
        goto exit;
    }

    if (FAIL == ioctl(file_fd, I2C_SLAVE, BME280_DEVICE_ADDR))
    {
        syslog(LOG_ERR, "Error setting i2c slave address %s", strerror(errno));
        close(file_fd);
        file_fd = -1;
        goto exit;
    }

    // Write configuration to the BME280 sensor
    char config[2] = {0};
    config[0] = 0xF2; // Humidity oversampling rate = 1
    config[1] = 0x01;
    if (write(file_fd, config, 2) != 2)
    {
        syslog(LOG_ERR, "Error writing to i2c device %s", strerror(errno));
        close(file_fd);
        file_fd = -1;
        goto exit;
    }
    printf("Debug: Humidity config written\n");

    config[0] = 0xF4; // Temperature and pressure oversampling rate = 1, mode = normal
    config[1] = 0x27;
    if (write(file_fd, config, 2) != 2)
    {
        syslog(LOG_ERR, "Error writing to i2c device %s", strerror(errno));
        close(file_fd);
        file_fd = -1;
        goto exit;
    }
    printf("Debug: Temp/Pressure config written\n");

    config[0] = 0xF5; // Standby time = 1000ms
    config[1] = 0xA0;
    if (write(file_fd, config, 2) != 2)
    {
        syslog(LOG_ERR, "Error writing to i2c device %s", strerror(errno));
        close(file_fd);
        file_fd = -1;
        goto exit;
    }
    printf("Debug: Standby time config written\n");

exit:
    return file_fd;
}

// Reads temperature, humidity, and pressure values from BME280 sensor
static void read_bme280_sensor(int file_id, float *temperature, float *humidity, float *pressure)
{
    char data[MAX_BUFF_LEN] = {0};
    char reg[1] = {0xF7};

    // Write register address
    if (write(file_id, reg, 1) != 1)
    {
        syslog(LOG_ERR, "Error writing to i2c device %s", strerror(errno));
        return;
    }

    // Read data back from BME280 sensor
    if (read(file_id, data, MAX_BUFF_LEN) != MAX_BUFF_LEN)
    {
        syslog(LOG_ERR, "Error reading from i2c device %s", strerror(errno));
        return;
    }

    // Debug: Print raw data
    printf("Debug: Raw data=");
    for (int i = 0; i < MAX_BUFF_LEN; i++)
    {
        printf(" 0x%02x", data[i]);
    }
    printf("\n");

    // Convert the data
    int adc_T = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16;
    int adc_H = (data[6] * 256) + data[7];
    float var1, T;

    var1 = (((float)adc_T) / 16384.0 - ((float)415) / 1024.0) * 175.72;
    T = var1;

    // Output data to screen
    *temperature = T;
    *humidity = ((float)adc_H / 1024.0) * 100.0;
    *pressure = ((data[0] << 12) + (data[1] << 4) + (data[2] >> 4)) / 100.0;

    // Debug print statements
    printf("Debug: Temperature = %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa\n", *temperature, *humidity, *pressure);
}

// Main function to initialize and read from BME280 sensor
int main(int argc, char *argv[])
{
    uint8_t i2c_node = I2C_NODE;
    int file_id = -1;
    float temperature = 0, humidity = 0, pressure = 0;

    if (argc > 2)
    {
        i2c_node = atoi(argv[1]);
        syslog(LOG_DEBUG, "Configured i2c_node = %d", i2c_node);
    }

    file_id = init_bme280_sensor(i2c_node);
    if (FAIL == file_id)
    {
        syslog(LOG_ERR, "Error initializing i2c device");
        return FAIL;
    }

    while (true)
    {
        read_bme280_sensor(file_id, &temperature, &humidity, &pressure);
        if (temperature == FAIL || humidity == FAIL || pressure == FAIL)
        {
            syslog(LOG_ERR, "Error reading sensor values");
            goto exit;
        }

        syslog(LOG_DEBUG, "Temperature: %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa", temperature, humidity, pressure);

        usleep(1000000); // Sleep for 1 second
    }

exit:
    close(file_id);
    return 0;
}
