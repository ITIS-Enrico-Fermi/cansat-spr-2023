#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <debug.h>


int uv_fd, gyro_fd; // < file descriptors of sensors
bool stop = false;  // < stop condition


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
struct __attribute__((__packed__)) gyro_accel_data {
  struct __attribute__((__packed__)) accel_data {
    int16_t x;
    int16_t y;
    int16_t z;
  } accel;

  int16_t temp;

  struct __attribute__((__packed__)) gyro_data {
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
struct __attribute__((__packed__)) lora_packet {
  gyro_t position;
  uint16_t light;
  uint32_t counter; // < monotonic packet counter
};



void open_sensors() {
  uv_fd = open("/dev/sensor/uv0", O_RDONLY);
  gyro_fd = open("/dev/sensor/gyro0", O_RDONLY);
}

void close_sensors() {
  close(uv_fd);
  close(gyro_fd);
}



int main(int argc, FAR char *argv[]) {
  uint16_t light = 0;
  gyro_t position;
  struct lora_packet pkt = {.counter = 0};

  open_sensors();

  int lora_fd = open("/dev/radio0", O_WRONLY);
  if(lora_fd < 0) {
    syslog(LOG_ERR, "Can't open the file descriptor for LoRa radio");
    return -1;
  }


  while(!stop) {
    read(uv_fd, &light, sizeof(light));
    read(gyro_fd, &position, sizeof(position));

    /* Log the gathered data */
    printf("UV light: %d\n", light);
    printf("Accel position x %d, y %d, z %d\n", position.accel.x, position.accel.y, position.accel.z);
    printf("Gyro rotation x %d, y %d, z %d\n", position.roto.x, position.roto.y, position.roto.z);
    printf("Temperature %d\n", position.temp);

    pkt.light = light;
    pkt.position = position;
    write(lora_fd, (void *)&pkt, sizeof(pkt));

    pkt.counter++;

    sleep(5);
  }

  close_sensors();
  close(lora_fd);
  return 0;
}
