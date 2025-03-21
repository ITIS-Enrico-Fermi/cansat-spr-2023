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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

/*
 * Test SPI driver before writing a stable library
 * library will offer a fast and easy way to use rfm95 device.
 */
int main(int argc, FAR char *argv[])
{
  printf("RFM95 driver test app\n");

  /* Open SPI Test Driver */

  int fd = open("/dev/radio0", O_RDWR);
  assert(fd >= 0);

  /* Write to SPI Test Driver */

  static char data[] = "Ciao RFM95"; /* Maybe send something more meaningfull.. */
  int bytes_written = write(fd, data, sizeof(data));
  assert(bytes_written == sizeof(data));

  /* Close SPI Test Driver */

  close(fd);
  return 0;
}