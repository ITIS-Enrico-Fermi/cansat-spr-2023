#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <nuttx/sensors/ioctl.h>
#include <poll.h>
#include <nuttx/config.h>
#include <debug.h>

#define BARO_DEV_NAME "/dev/sensor/sensor_baro0"
#define UV_DEV_NAME "/dev/sensor/sensor_uv0"
#define GYRO_DEV_NAME "/dev/sensor/sensor_gyro0"
#define CAMERA_DEV_NAME "/dev/sensor/sensor_camera0" // not yet implemented
#define RADIO_DEV_NAME "/dev/radio0"

/*
 * You can obtain 
 */
#define MPU6050_RANGE_2_G 16384 
#define MPU6050_RANGE_4_G 8192 
#define MPU6050_RANGE_8_G 4096 
#define MPU6050_RANGE_16_G 2048

#define MPU6050_RANGE_250_DEG 131
#define MPU6050_RANGE_500_DEG 65.5
#define MPU6050_RANGE_1000_DEG 32.8
#define MPU6050_RANGE_2000_DEG 16.4

/*
TODO
  1. handle errors returned from the read and write system calls.
  2. check the return values of the ioctl calls.
  3. free the memory allocated for the buffer space.
  4. handle the case when the file descriptors returned by the open calls are less than zero.
*/

int uv_fd, gyro_fd, baro_fd, radio_fd; // < file descriptors of sensors
struct pollfd fds;
bool stop = false;            // < stop condition

/**
 * This is the data structure to read from the accelerometer (MPU6050).
 *
 * It contains three fields structures: 3-axis acceleration, temperature as a signed
 * integer of 16 bit and 3-axis rotational momentum.
 *
 * The structure itself, as well as the inner ones, is packed. That means no padding bytes
 * are added by the compiler. This is inefficient for caching purposes but it is a perfect
 * choice when reading data in a structured manner.
 */
struct __attribute__((__packed__)) gyro_accel_data
{
  struct __attribute__((__packed__)) accel_data
  {
    int16_t x;
    int16_t y;
    int16_t z;
  } accel;

  int16_t temp;

  struct __attribute__((__packed__)) gyro_data
  {
    int16_t x;
    int16_t y;
    int16_t z;
  } roto;
};
typedef struct gyro_accel_data gyro_t;

/**
 * Structure of the packet that will be sent to the ground station.
 *
 * Packing is used here to not waste any bytes of the radio link.
 */
struct __attribute__((__packed__)) lora_packet
{
  gyro_t position;
  uint16_t light;
  uint32_t pressure;
  uint32_t counter; // < monotonic packet counter
};

begin_packed_struct struct sensor_gyro /* Type: Gyroscope */
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
}end_packed_struct;

struct sensor_baro /* Type: Barometer */
{
  uint64_t timestamp; /* Units is microseconds */
  float pressure;     /* pressure measurement in millibar or hpa */
  float temperature;  /* Temperature in degrees celsius */
};

int lora_fd;

int open_sensors(void)
{
  uv_fd = open(UV_DEV_NAME, O_RDONLY);
  if (uv_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for light sensor");
    return -1;
  }
  gyro_fd = open(GYRO_DEV_NAME, O_RDONLY);
  if (gyro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for gyro sensor");
    return -1;
  }
  baro_fd = open(BARO_DEV_NAME, O_RDONLY | O_NONBLOCK);
  if (baro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for baro sensor");
    return -1;
  }
#ifdef CONFIG_RF_RFM95
  lora_fd = open(RADIO_DEV_NAME, O_WRONLY);
  if (lora_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for radio module");
    return -1;
  }
#endif
  return 0;
}

int setup_sensors(void)
{
  int ret;

  /* Start setup bmp280 */
  unsigned int interval = 1000000;
  unsigned int latency = 0;
  ret = ioctl(baro_fd, SNIOC_SET_INTERVAL, interval);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      printf("Failed to set interval for sensor:%s, ret:%s\n",
             "bmp280", strerror(errno));
    }
    return ERROR;
  }

  ret = ioctl(baro_fd, SNIOC_BATCH, latency);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      printf("Failed to batch for sensor:%s, ret:%s\n",
             "bmp280", strerror(errno));
    }
    return ERROR;
  }

  fds.fd = baro_fd;
  fds.events = POLLIN;

  /* End setup bmp280 */
  return 0;
}

void read_sensors(void)
{

}

void close_sensors(void)
{
  close(uv_fd);
  close(gyro_fd);
  close(baro_fd);
}

void parse_gyro(uint8_t *buffer)
{
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
  float accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
  rawAccX = buffer[0] << 8 | buffer[1];
  rawAccY = buffer[2] << 8 | buffer[3];
  rawAccZ = buffer[4] << 8 | buffer[5];

  rawTemp = buffer[6] << 8 | buffer[7];

  rawGyroX = buffer[8] << 8 | buffer[9];
  rawGyroY = buffer[10] << 8 | buffer[11];
  rawGyroZ = buffer[12] << 8 | buffer[13];

  temperature = (rawTemp / 340.0) + 36.53;

  // default for the driver
  float accel_scale = MPU6050_RANGE_8_G;

  // setup range dependant scaling
  accX = ((float)rawAccX) / accel_scale;
  accY = ((float)rawAccY) / accel_scale;
  accZ = ((float)rawAccZ) / accel_scale;

  // default for the driver
  float gyro_scale = MPU6050_RANGE_1000_DEG;

  gyroX = ((float)rawGyroX) / gyro_scale;
  gyroY = ((float)rawGyroY) / gyro_scale;
  gyroZ = ((float)rawGyroZ) / gyro_scale;

  printf("## mpu6050 ##\n");
  printf("x_accel: %.2f g\ny_accel: %.2f g\nz_accel: %.2f g\n", accX, accY, accZ);
  printf("temp: %.2f\n", temperature);
  printf("x_gyro: %.2f  dps\ny_gyro: %.2f dps\nz_gyro: %.2f dps\n", gyroX, gyroY, gyroZ);
}

int main(int argc, FAR char *argv[])
{
  int ret;
  struct lora_packet pkt = {.counter = 0};
  struct sensor_gyro *gyro_val;
  struct sensor_baro *baro_val;

  /* Define structure length */
  int len_baro = sizeof(struct sensor_baro);
  int len_gyro = sizeof(struct sensor_gyro);
  int len_uv = sizeof(uint8_t) * 2;
  
  /* Allocate buffer space */
  uint8_t *baro_buf = malloc(len_baro);
  uint8_t *gyro_buf = malloc(len_gyro);
  uint8_t *uv_buf = malloc(len_uv);

  ret = open_sensors();
  if (ret < 0)
  {
    printf("Error while opening sensors.\n");
    return ERROR;
  }

  ret = setup_sensors();
  if (ret < 0)
  {
    printf("Error while setting sensors ioctl.\n");
    return ERROR;
  }

  while (!stop)
  {
    // We assume each read is successful each time, but the return code should be checked
    read(uv_fd, uv_buf, len_uv);

    read(gyro_fd, gyro_buf, len_gyro);
    gyro_val = (struct sensor_gyro *)gyro_buf;

    

    if (poll(&fds, 1, -1) > 0)
    {
      if (read(baro_fd, baro_buf, len_baro) >= len_baro)
      {
        baro_val = (struct sensor_baro *)baro_buf;
      }
    }

    printf("## veml6070 ##\n");
    printf("uv: %d\n", uv_buf[0]);
    parse_gyro(gyro_buf);
    printf("## bmp280 ##\n");
    printf("timestamp: %llu\npress:%.2f (hPa)\ntemp:%.2f (C)\n",
           baro_val->timestamp, baro_val->pressure, baro_val->temperature);

#ifdef CONFIG_RF_RFM95
    write(lora_fd, (void *)&pkt, sizeof(pkt));
#endif

    sleep(1);
  }

  free(gyro_buf);
  free(baro_buf);
  free(uv_buf);
  close_sensors();

#ifdef CONFIG_RF_RFM95
  close(lora_fd);
#endif
  return 0;
}
