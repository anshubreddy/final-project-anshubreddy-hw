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
#include <termios.h>

//
// Constants
//
#define BME280_DEVICE_ADDR 0x77          // Default I2C address of the BME280 sensor
#define I2C_NODE           1             // I2C bus number (e.g., I2C bus 1)
#define PASS               0             // Success return code
#define FAIL              -1             // Failure return code
#define MAX_BUFF_LEN       8             // Maximum buffer length for I2C data transfer
#define MAX_STR_LEN       15             // Maximum string length for device path
#define SERIAL_PORT       "/dev/ttyUSB0" // Serial port for XBee Coordinator

//
// Calibration Parameters (global variables)
//
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4, dig_H5;
int8_t   dig_H6;

//
// Function Prototypes
//
static int   init_bme280_sensor(uint8_t i2c_node);
static void  readCalibrationParams(int file_id);
static float compensateTemperature(int32_t adc_T, int32_t *t_fine);
static float compensateHumidity(int32_t adc_H, int32_t t_fine);
static float compensatePressure(int32_t adc_P, int32_t t_fine);
static void  read_bme280_sensor(int file_id, float *temperature, float *humidity, float *pressure);
static int   setup_serial(const char *portname);

//
// Initialize BME280 sensor via I2C
//
static int init_bme280_sensor(uint8_t i2c_node)
{
   char device_path[MAX_STR_LEN] = {0};
   int file_fd = -1;
   snprintf(device_path, MAX_STR_LEN, "/dev/i2c-%d", i2c_node);
   file_fd = open(device_path, O_RDWR);

   if (file_fd == FAIL)
   {
      syslog(LOG_ERR, "Error opening I2C device %s: %s", device_path, strerror(errno));
      return FAIL;
   }

   if (ioctl(file_fd, I2C_SLAVE, BME280_DEVICE_ADDR) == FAIL)
   {
      syslog(LOG_ERR, "Error setting I2C slave address: %s", strerror(errno));
      close(file_fd);
      return FAIL;
   }

   //
   // Configure humidity oversampling to 1 (register 0xF2)
   //
   char config[2] = {0xF2, 0x01};
   if (write(file_fd, config, 2) != 2)
   {
      syslog(LOG_ERR, "Error writing humidity config: %s", strerror(errno));
      close(file_fd);
      return FAIL;
   }

   // printf("Humidity config written\n");

   //
   // Configure temp/pressure oversampling to 1 and mode to normal (register 0xF4)
   //
   config[0] = 0xF4;
   config[1] = 0x27;
   if (write(file_fd, config, 2) != 2)
   {
      syslog(LOG_ERR, "Error writing temp/pressure config: %s", strerror(errno));
      close(file_fd);
      return FAIL;
   }

   // printf("Temp/Pressure config written\n");

   //
   // Configure standby time to 1000ms (register 0xF5)
   //
   config[0] = 0xF5;
   config[1] = 0xA0;
   if (write(file_fd, config, 2) != 2)
   {
      syslog(LOG_ERR, "Error writing standby config: %s", strerror(errno));
      close(file_fd);
      return FAIL;
   }

   // printf("Standby time config written\n");

   return file_fd;
}

//
// Read BME280 calibration parameters
//
static void readCalibrationParams(int file_id)
{
   //
   // Start of calibration data
   //
   char reg[1] = {0x88};
   char cal_data[26];

   write(file_id, reg, 1);      // Write register address to start reading calibration data
   read(file_id, cal_data, 24); // Read #24 bytes of calibration data for temperature and pressure

   //
   // Parse temperature calibration parameters
   //
   dig_T1 = (cal_data[1] << 8) | cal_data[0];
   dig_T2 = (cal_data[3] << 8) | cal_data[2];
   dig_T3 = (cal_data[5] << 8) | cal_data[4];

   //
   // Parse pressure calibration parameters
   //
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

   reg[0] = 0xE1;              // Switching to humidity calibration data
   write(file_id, reg, 1);     // Start of humidity calibration data
   read(file_id, cal_data, 7); // Read #7 bytes of humidity calibration data

   //
   // Parse humidity calibration parameters
   //
   dig_H2 = (cal_data[1] << 8) | cal_data[0];
   dig_H3 = cal_data[2];
   dig_H4 = (cal_data[3] << 4) | (cal_data[4] & 0x0F);
   dig_H5 = (cal_data[5] << 4) | (cal_data[4] >> 4);
   dig_H6 = cal_data[6];
}

//
// Compensate raw temperature data
//
static float compensateTemperature(int32_t adc_T, int32_t *t_fine)
{
   int32_t var1, var2;

   //
   // Step #1: Calculate first intermediate value
   // Shift raw ADC temperature and subtract doubled T1 calculation, then multiply by T2
   //
   var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;

   //
   // Step #2: Calculate second intermediate value
   // Compute squared difference from T1, scale with T3, and adjust with bit shifts
   //
   var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

   //
   // Step #3: Compute fine temperature (used by other compensation functions)
   //
   *t_fine = var1 + var2;

   //
   // Step #4: Convert to temperature in 0.01°C units (as per datasheet)
   //
   return (*t_fine * 5 + 128) >> 8;
}

//
// Compensate raw humidity data
//
static float compensateHumidity(int32_t adc_H, int32_t t_fine)
{
   int32_t v_x1_u32r;

   //
   // Step #1: Adjust fine temperature relative to a reference point (76800)
   //
   v_x1_u32r = (t_fine - ((int32_t)76800));

   //
   // Step #2: Apply humidity compensation formula (part 1)
   // Incorporate raw humidity, calibration parameters H4 and H5, and scale appropriately
   //
   v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
               ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
               (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
               ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

   //
   // Step #3: Apply additional correction with H1
   //
   v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));

   //
   // Step #4: Clamp the value to valid range (0 to 100% RH in scaled units)
   //
   v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
   v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

   //
   // Return humidity in percentage (%RH)
   //
   return (v_x1_u32r >> 12) / 1024.0;
}

//
// Compensate raw pressure data
//
static float compensatePressure(int32_t adc_P, int32_t t_fine)
{
   int64_t var1, var2, p;

   //
   // Step #1: Initialize first intermediate value based on fine temperature
   //
   var1 = ((int64_t)t_fine) - 128000;

   //
   // Step #2: Compute second intermediate value with polynomial terms
   //
   var2 = var1 * var1 * (int64_t)dig_P6;           // Quadratic term
   var2 = var2 + ((var1 * (int64_t)dig_P5) << 17); // Linear term
   var2 = var2 + (((int64_t)dig_P4) << 35);        // Offset term

   //
   // Step #3: Adjust first intermediate value with additional terms
   //
   var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
   var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

   //
   // Avoid division by zero
   //
   if (var1 == 0)
   {
      return 0;
   }

   //
   // Step #4: Compute pressure with scaling and corrections
   //
   p = 1048576 - adc_P;
   p = (((p << 31) - var2) * 3125) / var1;
   var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25; // Fine correction #1
   var2 = (((int64_t)dig_P8) * p) >> 19;                     // Fine correction #2
   p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

   //
   // Return pressure in hPa
   //
   return (float)p / 256.0;
}

//
// Read sensor data from BME280
//
static void read_bme280_sensor(int file_id, float *temperature, float *humidity, float *pressure)
{
   char data[MAX_BUFF_LEN] = {0};
   char reg[1] = {0xF7}; // Data register for temperature, humidity, and pressure

   //
   // Write the register address to select data registers
   //
   if (write(file_id, reg, 1) != 1)
   {
      syslog(LOG_ERR, "Error writing to I2C device: %s", strerror(errno));
      return;
   }

   //
   // Read #8 bytes of sensor data
   //
   if (read(file_id, data, MAX_BUFF_LEN) != MAX_BUFF_LEN)
   {
      syslog(LOG_ERR, "Error reading from I2C device: %s", strerror(errno));
      return;
   }

   //
   // Construct raw ADC values from the data
   //
   int32_t adc_T = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4)); // 20-bit temperature
   int32_t adc_P = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4)); // 20-bit presurre
   int32_t adc_H = (data[6] << 8) | data[7];                            // 16-bit humidity

   //
   // Compute compensated values
   //
   int32_t t_fine = 0;
   *temperature = compensateTemperature(adc_T, &t_fine) / 100.0;
   *humidity = compensateHumidity(adc_H, t_fine);
   *pressure = compensatePressure(adc_P, t_fine) / 100.0;

   // printf("Temperature = %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa\n", *temperature, *humidity, *pressure);
}

//
// Set up serial port for Zigbee communication
// This function configures a serial port to communicate with a Zigbee device, using settings
// that align with tools like XCTU and environments like Raspberry Pi.
// Typical settings include a baud rate of 9600, #8 data bits, no parity,
// #1 stop bit, and no hardware flow control, matching the "9600/8/N/1" configuration in XCTU.
//
static int setup_serial(const char *portname)
{
   //
   // Open the serial port specified by 'portname' (e.g., "/dev/ttyUSB0") in read/write mode
   // The flags used ensure proper operation:
   // - O_RDWR: Opens the port for both reading and writing, necessary for two-way communication
   //   with Zigbee devices.
   // - O_NOCTTY: Prevents the serial port from becoming the controlling terminal, which is
   //   important in embedded systems like Raspberry Pi to avoid unintended behavior.
   // - O_SYNC: Ensures synchronous I/O operations, providing reliable data transfer with
   //   hardware devices like Zigbee modules by synchronizing reads and writes.
   //
   int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

   //
   // Verify that the serial port opened successfully. If it fails (e.g., due to an invalid port
   // name or permissions issue), print an error message with perror for debugging and return
   // a failure code. This is critical for diagnosing issues seen in terminal setups.
   //
   if (fd < 0)
   {
      perror("open");
      return FAIL;
   }

   //
   // Declare a termios structure to store the serial port's configuration settings.
   // This structure will be used to customize the port's behavior for Zigbee communication.
   //
   struct termios tty;

   //
   // Retrieve the current serial port settings into the 'tty' structure. This step is necessary
   // to modify specific settings while preserving others. If it fails (e.g., due to an invalid
   // file descriptor), print an error, close the port, and return failure to prevent further issues.
   //
   if (tcgetattr(fd, &tty) != 0)
   {
      perror("tcgetattr");
      close(fd);
      return FAIL;
   }

   //
   // Set the output baud rate to 9600 bits per second, matching the configuration observed in
   // XCTU (e.g., "/dev/ttyUSB0 - 9600/8/N/1") and used in Raspberry Pi terminal output for
   // Zigbee sensor data. This ensures the port sends data at the correct speed.
   //
   cfsetospeed(&tty, B9600);

   //
   // Set the input baud rate to 9600 bits per second to maintain consistent communication speed
   // for receiving data from the Zigbee device, aligning with the XCTU settings.
   //
   cfsetispeed(&tty, B9600);

   //
   // Configure the port for 8 data bits, as specified in the "8" of "8/N/1" from XCTU.
   // Clear the character size bits with '~CSIZE' and set 'CS8' to use 8-bit data, which is
   // standard for Zigbee communication and ensures proper data framing.
   //
   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

   //
   // Disable ignoring of break signals by clearing the IGNBRK flag. This ensures that break
   // conditions (e.g., interruptions in the serial line) are handled properly, which is
   // important for robust communication with Zigbee devices in real-time applications.
   //
   tty.c_iflag &= ~IGNBRK;

   //
   // Disable all local flags (e.g., canonical mode, echo) by setting c_lflag to 0.
   // Canonical mode processes input line-by-line, which is unsuitable for raw binary data or
   // real-time sensor data like that seen in the Raspberry Pi terminal output (e.g., TEMP, HUM).
   //
   tty.c_lflag = 0;

   //
   // Disable output processing flags by setting c_oflag to 0. This ensures that data is sent
   // raw, without system modifications (e.g., carriage return translation), which is critical
   // for sending exact byte sequences to Zigbee devices as shown in XCTU console logs.
   //
   tty.c_oflag = 0;

   //
   // Set the minimum number of characters to read before returning to 1 in non-canonical mode.
   // This allows the read function to return as soon as a single byte is available, which is
   // useful for processing continuous sensor data streams like those in the terminal output.
   //
   tty.c_cc[VMIN] = 1;

   //
   // Set the read timeout to 0.5 seconds (5 tenths of a second, as VTIME is in tenths).
   // This prevents indefinite blocking if no data is received, balancing responsiveness and
   // reliability in applications like the sensor data logging seen on the Raspberry Pi.
   //
   tty.c_cc[VTIME] = 5;

   //
   // Disable software flow control (XON/XOFF) by clearing IXON, IXOFF, and IXANY flags.
   // Software flow control uses special characters that could interfere with raw data
   // transmission, such as the hexadecimal bytes (e.g., "61 62 63") seen in XCTU console logs.
   //
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);

   //
   // Enable the receiver and set the port to local mode by setting CLOCAL and CREAD flags:
   // - CLOCAL: Ignores modem control lines, suitable for direct serial communication with
   //   Zigbee devices without modem involvement, as seen in XCTU setups.
   // - CREAD: Enables the receiver to read incoming data, essential for capturing sensor
   //   readings like temperature and humidity in the terminal output.
   //
   tty.c_cflag |= (CLOCAL | CREAD);

   //
   // Disable parity checking by clearing PARENB and PARODD flags, aligning with the "N" in
   // "8/N/1" from XCTU. Zigbee devices typically use no parity, ensuring data is transmitted
   // without additional parity bits that could complicate communication.
   //
   tty.c_cflag &= ~(PARENB | PARODD);

   //
   // Set to 1 stop bit by clearing CSTOPB, matching the "1" in "8/N/1" from XCTU.
   // One stop bit is standard for Zigbee communication and ensures proper data framing.
   //
   tty.c_cflag &= ~CSTOPB;

   //
   // Disable hardware flow control (RTS/CTS) by clearing CRTSCTS. This is not required by the
   // Zigbee setups in XCTU, as evidenced by the absence of RTS/CTS indicators in the interface,
   // and prevents interference from unused control lines in direct serial communication.
   //
   tty.c_cflag &= ~CRTSCTS;

   //
   // Apply the configured settings to the serial port immediately using TCSANOW.
   // If this fails (e.g., due to invalid settings or hardware issues), print an error message,
   // close the port to free resources, and return failure to alert the caller of the issue.
   //
   if (tcsetattr(fd, TCSANOW, &tty) != 0)
   {
      perror("tcsetattr");
      close(fd);
      return FAIL;
   }

   //
   // Return the file descriptor of the successfully configured serial port.
   // This descriptor can now be used for reading from and writing to the Zigbee device,
   // enabling applications like those seen in XCTU console logs or Raspberry Pi sensor data output.
   //
   return fd;
}

//
// Main function
//
int main(int argc, char *argv[])
{
   uint8_t i2c_node = I2C_NODE;
   int i2c_file_id = -1;
   int serial_fd = -1;
   float temperature = 0, humidity = 0, pressure = 0;

   if (argc > 1)
   {
      i2c_node = atoi(argv[1]);
      syslog(LOG_DEBUG, "Configured i2c_node = %d", i2c_node);
   }

   i2c_file_id = init_bme280_sensor(i2c_node);
   if (i2c_file_id == FAIL)
   {
      syslog(LOG_ERR, "Error initializing I2C device");
      return FAIL;
   }

   readCalibrationParams(i2c_file_id);

   serial_fd = setup_serial(SERIAL_PORT);
   if (serial_fd == FAIL)
   {
      syslog(LOG_ERR, "Error setting up serial port");
      close(i2c_file_id);
      return FAIL;
   }

   while (true)
   {
      read_bme280_sensor(i2c_file_id, &temperature, &humidity, &pressure);

      char sensor_data[100];
      snprintf(sensor_data, sizeof(sensor_data), "Temp: %.2f°C, Hum: %.2f%%, Press: %.2f hPa\n",
               temperature, humidity, pressure);

      ssize_t bytes_written = write(serial_fd, sensor_data, strlen(sensor_data));

      if (bytes_written == -1)
      {
         perror("write");
      }
      else
      {
         printf("Sent: %s", sensor_data);
      }

      usleep(1000000); // Sleep for 1 second
   }

   close(i2c_file_id);
   close(serial_fd);
   return 0;
}
