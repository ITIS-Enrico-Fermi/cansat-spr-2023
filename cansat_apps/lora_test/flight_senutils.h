#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <nuttx/sensors/ioctl.h>
#include <poll.h>
#include <nuttx/config.h>
#include <debug.h>

#define GPS_DEV_NAME "/dev/gps"
#define BARO_DEV_NAME "/dev/sensor/sensor_baro0"
#define UV_DEV_NAME "/dev/sensor/sensor_uv0"
#define GYRO_DEV_NAME "/dev/sensor/sensor_gyro0"
#define CAMERA_DEV_NAME "/dev/sensor/sensor_camera0"
#define RADIO_DEV_NAME "/dev/radio0"

#define GNSS_POLL_FD_NUM 1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define GNSS_USERSPACE_SIG 18

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

void parse_gyro(void *in, void *out);
void parse_gps(void *in, void *out);

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

struct __attribute__((__packed__)) cxd56_gnss_dms_s
{
  int8_t   sign;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
};
void double_to_dmf(double x, struct cxd56_gnss_dms_s *dmf);

struct __attribute__((__packed__)) cxd56_gnss_latlon_s
{
  float latitude;
  float longitude;
};
typedef struct cxd56_gnss_latlon_s gnss_t;

/**
 * Structure of the packet that will be sent to the ground station.
 *
 * Packing is used here to not waste any bytes of the radio link.
 */
struct __attribute__((__packed__)) lora_packet
{
  float pressure;
  gyro_t gyro;
  gnss_t gps;
  uint8_t uv;
  uint16_t counter; // < monotonic packet counter
};