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

// Calibration Parameters
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

// Function Prototypes
static int init_bme280_sensor(uint8_t i2c_node);
static void readCalibrationParams(int file_id);
static float compensateTemperature(int32_t adc_T, int32_t *t_fine);
static float compensateHumidity(int32_t adc_H, int32_t t_fine);
static float compensatePressure(int32_t adc_P, int32_t t_fine);
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

// Reads calibration parameters from BME280 sensor
static void readCalibrationParams(int file_id)
{
    char reg[1] = {0x88}; // Calibration parameter start address
    char cal_data[26]; // Buffer to store calibration data

    // Write register address
    write(file_id, reg, 1);

    // Read calibration data
    read(file_id, cal_data, 24);

    // Parse temperature and pressure calibration data
    dig_T1 = (cal_data[1] << 8) | cal_data[0];
    dig_T2 = (cal_data[3] << 8) | cal_data[2];
    dig_T3 = (cal_data[5] << 8) | cal_data[4];
    dig_P1 = (cal_data[7] << 8) | cal_data[6];
    dig_P2 = (cal_data[9] << 8) | cal_data[8];
    dig_P3 = (cal_data[11] << 8) | cal_data[10];
    dig_P4 = (cal_data[13] << 8) | cal_data[12];
    dig_P5 = (cal_data[15] << 8) | cal_data[14];
    dig_P6 = (cal_data[17] << 8) | cal_data[16];
    dig_P7 = (cal_data[19] << 8) | cal_data[18];
    dig_P8 = (cal_data[21] << 8) | cal_data[20];
    dig_P9 = (cal_data[23] << 8) | cal_data[22];
    dig_H1 = cal_data[25];

    // Read humidity calibration parameters
    reg[0] = 0xE1;
    write(file_id, reg, 1);
    read(file_id, cal_data, 7);

    dig_H2 = (cal_data[1] << 8) | cal_data[0];
    dig_H3 = cal_data[2];
    dig_H4 = (cal_data[3] << 4) | (cal_data[4] & 0x0F);
    dig_H5 = (cal_data[5] << 4) | (cal_data[4] >> 4);
    dig_H6 = cal_data[6];
}

// Compensates raw temperature data to actual temperature
static float compensateTemperature(int32_t adc_T, int32_t *t_fine)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

// Compensates raw humidity data to actual humidity
static float compensateHumidity(int32_t adc_H, int32_t t_fine)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                                                (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                                                 ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                                             ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (v_x1_u32r >> 12) / 1024.0;
}

// Compensates raw pressure data to actual pressure
static float compensatePressure(int32_t adc_P, int32_t t_fine)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // Avoid division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0;
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
    int32_t adc_T = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    int32_t adc_P = ((data[0] >> 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t adc_H = (data[6] << 8) | data[7];
    int32_t t_fine = 0;

    // Compensate the data
    *temperature = compensateTemperature(adc_T, &t_fine) / 100.0;
    *humidity = compensateHumidity(adc_H, t_fine);
    *pressure = compensatePressure(adc_P, t_fine) / 100.0;

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

    readCalibrationParams(file_id);

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
