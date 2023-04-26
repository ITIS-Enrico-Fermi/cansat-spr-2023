/****************************************************************************
 * apps/examples/camera/camera_fileutil.h
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

#ifndef __CANSAT_APPS_CAMERA_TEST_CAMERA_UTILS_H
#define __CANSAT_APPS_CAMERA_TEST_CAMERA_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <sys/types.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>

#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMAGE_JPG_SIZE     (512*1024)  /* 512kB for FullHD Jpeg file. */
#define IMAGE_RGB_SIZE     (320*240*2) /* QVGA RGB565 */

#define VIDEO_BUFNUM       (3)
#define STILL_BUFNUM       (1)

#define MAX_CAPTURE_NUM     (100)
#define DEFAULT_CAPTURE_NUM (1)

#define START_CAPTURE_TIME  (5)   /* seconds */
#define KEEP_VIDEO_TIME     (10)  /* seconds */

#define APP_STATE_BEFORE_CAPTURE  (0)
#define APP_STATE_UNDER_CAPTURE   (1)
#define APP_STATE_AFTER_CAPTURE   (2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct v_buffer
{
  uint32_t *start;
  uint32_t length;
};

typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int camera_prepare(int fd, enum v4l2_buf_type type,
                          uint32_t buf_mode, uint32_t pixformat,
                          uint16_t hsize, uint16_t vsize,
                          struct v_buffer **vbuf,
                          uint8_t buffernum, int buffersize);
void free_buffer(struct v_buffer  *buffers, uint8_t bufnum);
int get_image(int fd);
int release_image(int fd);
int start_capture(int v_fd);
int stop_capture(int v_fd);

const char *get_imgsensor_name(int fd);
int prepare_stream(int v_fd);

int shoot_photo(int cam_fd);
int camlib_init(int cam_fd);
void camlib_clear(void);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Set images path */

const char *cutil_setpath(void);

/* Write an image file */

int write_image(void);

#endif  /* __APPS_EXAMPLES_CAMERA_CAMERA_FILEUTIL_H */

