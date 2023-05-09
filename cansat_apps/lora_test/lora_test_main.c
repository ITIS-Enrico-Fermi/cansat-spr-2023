/****************************************************************************
 * cansat_apps/lora_test/lora_test_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "nuttx/rf/rfm95.h"

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>
#include <nuttx/rf/rfm95.h>
#include <arch/board/board.h>

#define DEV_NAME "/dev/radio0"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#include <arch/chip/gnss.h> //gnss driver structures

#define GNSS_POLL_FD_NUM 1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define GNSS_USERSPACE_SIG 18
#define GPS_DEV_NAME "/dev/gps"

/****************************************************************************
 * main
 ****************************************************************************/
/* GPS data */
static uint32_t posfixflag;
static struct cxd56_gnss_positiondata_s posdat;
int posperiod;
sigset_t mask;
const struct timespec waitgps = {
    .tv_sec = 3,
    .tv_nsec = 0};


struct __attribute__((__packed__)) cxd56_gnss_latlon_s
{
  float latitude;
  float longitude;
};
typedef struct cxd56_gnss_latlon_s gnss_t;
/*
 * Test SPI driver before writing a stable library
 * library will offer a fast and easy way to use rfm95 device.
 */
void parse_gps(FAR void *in, FAR void *out)
{
  struct cxd56_gnss_positiondata_s *posdat = (struct cxd56_gnss_positiondata_s *)in;
  gnss_t *postransform = (gnss_t *)out;
  if (posdat->receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID)
  {
    postransform->latitude = (float)posdat->receiver.latitude;
    postransform->longitude = (float)posdat->receiver.longitude;
  }
  else
  {
    /* No measurement. */
    postransform->latitude = 0.0F;
    postransform->longitude = 0.0F;
    _err(">No Positioning Data\n");
  }
}

int main(int argc, FAR char *argv[])
{
   int ret, gps_fd;
   gnss_t *gps;
   printf("RFM95 driver test app\n");

   gps_fd = open(GPS_DEV_NAME, O_RDONLY);
   if (gps_fd < 0)
   {
      syslog(LOG_ERR, "Can't open GNSS character device at %s. Error code: %d\n", GPS_DEV_NAME, errno);
      return ERROR;
   }
   /* START SETUP GPS0 IOCTL */
   /* Prepare a signal set and enable GNSS signal only. */
   sigemptyset(&mask);
   sigaddset(&mask, GNSS_USERSPACE_SIG);
   ret = sigprocmask(SIG_BLOCK, &mask, NULL);
   if (ret != OK)
   {
      _info("sigprocmask failed. %d\n", ret);
   }

   /* Set the signal to notify GNSS events. */
   struct cxd56_gnss_signal_setting_s setting = {
       .fd = gps_fd,
       .enable = true,
       .gnsssig = CXD56_GNSS_SIG_GNSS,
       .signo = GNSS_USERSPACE_SIG,
       .data = NULL};

   ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_SIGNAL_SET, &setting);
   if (ret < 0)
   {
      _info("Error while configuring signalling\n");
      return -1;
   }

   /* START set GNSS parameters. */
   uint32_t set_satellite;

   /*
    * Set the operation mode (always 1) and the notify cycle, which is the
    * frequency of a new position from the chip.
    */
   struct cxd56_gnss_ope_mode_param_s operation_mode_config = {
       .mode = 1,
       .cycle = 1000 // in milliseconds
   };

   ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, &operation_mode_config);
   if (ret < 0)
   {
      _info("Error setting Operation Mode via ioctl. Return code: %d\n", ret);
      return ret;
   }

   /* Set the type of satellite system used by GNSS. */
   set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS;

   ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
   if (ret < 0)
   {
      _info("Can't set satellite system.\n");
      return ret;
   }
   /* END set GNSS parameters. */

   int fd = open(DEV_NAME, O_RDWR);
   if (fd < 0)
   {
      int errcode = errno;
      printf("ERROR: Failed to open device %s: %d\n", DEV_NAME, errcode);
   }

   ret = ioctl(fd, RFM95_IOCTL_INIT, 0);
   if (ret < 0)
   {
      printf("failed to change init: %d!\n", ret);
   }

   printf("Init done!\n");
   while (1)
   {
      _info("waiting gps signal..\n");
      ret = sigtimedwait(&mask, NULL, &waitgps);
      // ret = sigwaitinfo(&mask, NULL);
      if (ret != GNSS_USERSPACE_SIG)
      {
         _info("Waited signal, but instead of GNSS signal got: %d\n", ret);
      }
      else
         _info("GPS signal received!\n");

      /* Read and print POS data. */
      ret = read(gps_fd, &posdat, sizeof(posdat));
      if (ret < 0)
      {
         _info("Can't read gps0\n");
         return ERROR;
      }
      parse_gps(&posdat, gps);
      write(fd, gps, sizeof(gnss_t));
      up_mdelay(5000);
   }

   close(fd);
   return 0;
}