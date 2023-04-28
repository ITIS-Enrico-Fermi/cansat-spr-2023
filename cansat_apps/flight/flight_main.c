#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <nuttx/config.h>
#include <debug.h>
#include <nuttx/sensors/ioctl.h>
#include <poll.h>

#define BARO_DEV_NAME "/dev/sensor/sensor_baro0"
#define UV_DEV_NAME "/dev/sensor/sensor_uv0"
#define GYRO_DEV_NAME "/dev/sensor/sensor_gyro0"
#define CAMERA_DEV_NAME "/dev/sensor/sensor_camera0"
#define RADIO_DEV_NAME "/dev/radio0"

int uv_fd, gyro_fd, press_fd; // < file descriptors of sensors
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

void open_sensors()
{
#ifdef CONFIG_SENSORS_VEML6070
  printf("Opening %s\n", UV_DEV_NAME);
  uv_fd = open(UV_DEV_NAME, O_RDONLY);
#endif
  printf("Opening %s\n", GYRO_DEV_NAME);
  gyro_fd = open(GYRO_DEV_NAME, O_RDONLY);
  printf("Opening %s\n", BARO_DEV_NAME);
  press_fd = open(BARO_DEV_NAME, O_RDONLY | O_NONBLOCK);
}

void close_sensors()
{
#ifdef CONFIG_SENSORS_VEML6070
  close(uv_fd);
#endif
  close(gyro_fd);
  close(press_fd);
}

int main(int argc, FAR char *argv[])
{
  int ret;
  struct pollfd fds;
  int len_uv = sizeof(uint8_t) * 2;
  uint8_t *uv_buf = malloc(len_uv);
  gyro_t position;
  struct lora_packet pkt = {.counter = 0};
  struct sensor_gyro *gyro_val;
  struct sensor_baro *baro_val;
  int len_baro = sizeof(struct sensor_baro);
  char *baro_buf = malloc(len_baro);
  int len_gyro = sizeof(struct sensor_gyro);
  uint8_t *gyro_buf = malloc(len_gyro);

  printf("Open sensors..\n");
  open_sensors();

#ifdef CONFIG_RF_RFM95
  printf("Opening %s\n", RADIO_DEV_NAME);
  int lora_fd = open(RADIO_DEV_NAME, O_WRONLY);
  if (lora_fd < 0)
  {
    syslog(LOG_ERR, "Can't open the file descriptor for LoRa radio");
    return -1;
  }
#endif
  unsigned int interval = 1000000;
  unsigned int latency = 0;
  ret = ioctl(press_fd, SNIOC_SET_INTERVAL, interval);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      printf("Failed to set interval for sensor:%s, ret:%s\n",
             "bmp280", strerror(errno));
    }
  }

  ret = ioctl(press_fd, SNIOC_BATCH, latency);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      printf("Failed to batch for sensor:%s, ret:%s\n",
             "bmp280", strerror(errno));
    }
  }

  fds.fd = press_fd;
  fds.events = POLLIN;

  while (!stop)
  {
    // We assume each read is successful each time, but the return code should be checked
#ifdef CONFIG_SENSORS_VEML6070
    printf("Reading uv..\n");
    read(uv_fd, uv_buf, len_uv);
#endif
    printf("Reading gyro..\n");
    // read(gyro_fd, &position, sizeof(gyro_t));
    read(press_fd, gyro_buf, len_gyro);
    gyro_val = (struct sensor_gyro *)baro_buf;

    printf("Reading baro..\n");
    if (poll(&fds, 1, -1) > 0)
    {
      if (read(press_fd, baro_buf, len_baro) >= len_baro)
      {
        baro_val = (struct sensor_baro *)baro_buf;
      }
    }
    // read(press_fd, buffer, len);
    // struct sensor_baro *event = (struct sensor_baro *)buffer;

    /* Log the gathered data */
#ifdef CONFIG_SENSORS_VEML6070
    printf("UV: %d\n", uv_buf[0]);
#endif
    //printf("Accel position x %d, y %d, z %d\n", position.accel.x, position.accel.y, position.accel.z);
    //printf("Gyro rotation x %d, y %d, z %d\n", position.roto.x, position.roto.y, position.roto.z);
    //printf("Temperature %d\n", position.temp);
    printf("xaccel %d, yaccel %d, zaccel %d\n", gyro_val->x_accel, gyro_val->y_accel, gyro_val->z_accel);
    printf("Temperature mpu %d\n", gyro_val->temp);
    printf("x_gyro %d, y_gyro %d, z_gyro %d\n", gyro_val->x_gyro, gyro_val->y_gyro, gyro_val->z_gyro);
    printf("%s: timestamp:%" PRIu64 " value1:%.2f value2:%.2f\n",
           "BMP280", baro_val->timestamp, baro_val->pressure, baro_val->temperature);

    //pkt.light = light;
    //pkt.position = position;
    //pkt.pressure = pressure;

#ifdef CONFIG_RF_RFM95
    write(lora_fd, (void *)&pkt, sizeof(pkt));
#endif

    pkt.counter++;

    sleep(2);
  }

  free(gyro_buf);
  free(baro_buf);
  close_sensors();

#ifdef CONFIG_RF_RFM95
  close(lora_fd);
#endif
  return 0;
}
