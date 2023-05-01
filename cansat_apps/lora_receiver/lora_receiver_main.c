#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "nuttx/rf/rfm95.h"

#define DEV_NAME "/dev/radio0"

#pragma pack 1
struct pkt_t {
  uint8_t id;
  char msg[10];
} pkt;

size_t pkt_len;


int main(int argc, FAR char *argv[]) {
  int ret;
  printf("LoRa receiver\n");

  int fd = open(DEV_NAME, O_RDWR);
  if (fd < 0)
    {
    int errcode = errno;
    printf("ERROR: Failed to open device %s: %d\n", DEV_NAME, errcode);
    }

  ret = ioctl(fd, RFM95_IOCTL_INIT, 0);
  if (ret < 0) {
    printf("failed to change init: %d!\n", ret);
    }

  printf("Init done!\n");

  while(true) {
    pkt_len = read(fd, &pkt, sizeof(struct pkt_t));

    printf("Received packet of %d bytes\n", pkt_len);

    printf("%d, %s\n", pkt.id, pkt.msg);

    usleep(500000);
  }

  close(fd);
  return 0;
}
