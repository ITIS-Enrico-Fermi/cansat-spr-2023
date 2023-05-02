#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
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

#define MPU6050_RANGE_2_G 16384 
#define MPU6050_RANGE_4_G 8192 
#define MPU6050_RANGE_8_G 4096 
#define MPU6050_RANGE_16_G 2048

#define MPU6050_RANGE_250_DEG 131
#define MPU6050_RANGE_500_DEG 65.5
#define MPU6050_RANGE_1000_DEG 32.8
#define MPU6050_RANGE_2000_DEG 16.4

/**************************************************/
/*** SENSOR UTILITY FUNCTIONS *********************/
/**************************************************/

int open_sensors(void);
int setup_sensors(void);
void close_sensors(void);

int read_baro(uint8_t *baro_buf, int len_baro);
int read_gyro(uint8_t *gyro_buf, int len_gyro);
int read_uv(uint8_t *uv_buf, int len_uv);
void parse_gyro(uint8_t *buffer);

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

struct __attribute__((__packed__)) baro_press_data /* Type: Barometer */
{
  uint64_t timestamp; /* Units is microseconds */
  float pressure;     /* pressure measurement in millibar or hpa */
  float temperature;  /* Temperature in degrees celsius */
};
typedef struct baro_press_data baro_t;

/**
 * Structure of the packet that will be sent to the ground station.
 *
 * Packing is used here to not waste any bytes of the radio link.
 */
struct __attribute__((__packed__)) lora_packet
{
  gyro_t position;
  uint16_t light;
  float pressure;
  uint16_t counter; // < monotonic packet counter
};