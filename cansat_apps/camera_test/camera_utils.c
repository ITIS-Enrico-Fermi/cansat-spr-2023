/****************************************************************************
 * apps/examples/camera/camera_fileutil.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>
#include <debug.h>

#include <camera_utils.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMAGE_FILENAME_LEN (32)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *save_dir;
static bool is_initialized = false;
static const char *sensor;
enum v4l2_buf_type capture_type = V4L2_BUF_TYPE_STILL_CAPTURE;
struct v4l2_buffer v4l2_buf;
static uint16_t w, h;
struct v_buffer *buffers_video = NULL;
struct v_buffer *buffers_still = NULL;


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cutil_setpath()
 *
 * Description:
 *   Choose strage to write a file.
 ****************************************************************************/

const char *cutil_setpath(void)
{
  int ret;
  struct stat stat_buf;

  /* In SD card is available, use SD card.
   * Otherwise, use SPI flash.
   */

  ret = stat("/mnt/sd0", &stat_buf);
  if (ret < 0)
    {
      save_dir = "/mnt/spif";
    }
  else
    {
      save_dir = "/mnt/sd0";
    }

  return save_dir;
}

/****************************************************************************
 * Name: cutil_writeimage()
 *
 * Description:
 *   Write a image file to selected storage.
 ****************************************************************************/

int write_image()
{
  uint8_t *data = (uint8_t *)v4l2_buf.m.userptr;
  size_t len = (size_t)v4l2_buf.bytesused;
  const char *fsuffix = (capture_type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ? "RGB" : "JPG";
  static char s_fname[IMAGE_FILENAME_LEN];
  static int s_framecount = 0;

  FILE *fp;

  s_framecount++;
  if (s_framecount >= 1000)
    {
      s_framecount = 1;
    }

  memset(s_fname, 0, sizeof(s_fname));

  snprintf(s_fname,
           IMAGE_FILENAME_LEN,
           "%s/VIDEO%03d.%s",
           save_dir, s_framecount, fsuffix);

  printf("FILENAME:%s\n", s_fname);

  fp = fopen(s_fname, "wb");
  if (NULL == fp)
    {
      printf("fopen error : %d\n", errno);
      return -1;
    }

  if (len != fwrite(data, 1, len, fp))
    {
      printf("fwrite error : %d\n", errno);
    }

  fclose(fp);
  return 0;
}

/****************************************************************************
 * Name: camera_prepare()
 *
 * Description:
 *   Allocate frame buffer for camera and Queue the allocated buffer
 *   into video driver.
 ****************************************************************************/

int camera_prepare(int fd, enum v4l2_buf_type type,
                          uint32_t buf_mode, uint32_t pixformat,
                          uint16_t hsize, uint16_t vsize,
                          struct v_buffer **vbuf,
                          uint8_t buffernum, int buffersize)
{
  int ret;
  int cnt;
  struct v4l2_format fmt =
  {
    0
  };

  struct v4l2_requestbuffers req =
  {
    0
  };

  struct v4l2_buffer buf =
  {
    0
  };

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = buffernum;
  req.mode   = buf_mode;

  ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_S_FMT set format */

  fmt.type                = type;
  fmt.fmt.pix.width       = hsize;
  fmt.fmt.pix.height      = vsize;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = pixformat;

  ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      return ret;
    }

  /* Prepare video memory to store images */

  *vbuf = malloc(sizeof(v_buffer_t) * buffernum);

  if (!(*vbuf))
    {
      printf("Out of memory for array of v_buffer_t[%d]\n", buffernum);
      return ERROR;
    }

  for (cnt = 0; cnt < buffernum; cnt++)
    {
      (*vbuf)[cnt].length = buffersize;

      /* Note:
       * VIDIOC_QBUF set buffer pointer.
       * Buffer pointer must be 32bytes aligned.
       */

      (*vbuf)[cnt].start  = memalign(32, buffersize);
      if (!(*vbuf)[cnt].start)
        {
          printf("Out of memory for image buffer of %d/%d\n",
                 cnt, buffernum);

          /* Release allocated memory. */

          while (cnt)
            {
              cnt--;
              free((*vbuf)[cnt].start);
            }

          free(*vbuf);
          *vbuf = NULL;
          return ERROR;
        }
    }

  /* VIDIOC_QBUF enqueue buffer */

  for (cnt = 0; cnt < buffernum; cnt++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = type;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = cnt;
      buf.m.userptr = (unsigned long)(*vbuf)[cnt].start;
      buf.length = (*vbuf)[cnt].length;

      ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          free_buffer(*vbuf, buffernum);
          *vbuf = NULL;
          return ERROR;
        }
    }

  /* VIDIOC_STREAMON start stream */

  ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
      free_buffer(*vbuf, buffernum);
      *vbuf = NULL;
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: free_buffer()
 *
 * Description:
 *   All free allocated memory of v_buffer.
 ****************************************************************************/

void free_buffer(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

/****************************************************************************
 * Name: get_image()
 *
 * Description:
 *   DQBUF camera frame buffer from video driver with taken picture data.
 ****************************************************************************/

int get_image(int fd)
{
  int ret;
  struct v4l2_buffer *v4l2_buf_ptr = &v4l2_buf;

  /* VIDIOC_DQBUF acquires captured data. */

  memset(v4l2_buf_ptr, 0, sizeof(v4l2_buffer_t));
  v4l2_buf_ptr->type = capture_type;
  v4l2_buf_ptr->memory = V4L2_MEMORY_USERPTR;

  ret = ioctl(fd, VIDIOC_DQBUF, (unsigned long)v4l2_buf_ptr);
  if (ret)
    {
      printf("Fail DQBUF %d\n", errno);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: release_image()
 *
 * Description:
 *   Re-QBUF to set used frame buffer into video driver.
 ****************************************************************************/

int release_image(int fd)
{
  int ret;
  struct v4l2_buffer *v4l2_buf_ptr = &v4l2_buf;

  /* VIDIOC_QBUF sets buffer pointer into video driver again. */

  ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)v4l2_buf_ptr);
  if (ret)
    {
      printf("Fail QBUF %d\n", errno);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: start_stillcapture()
 *
 * Description:
 *   Start STILL capture stream by TAKEPICT_START if buf_type is
 *   STILL_CAPTURE.
 ****************************************************************************/

int start_capture(int cam_fd)
{
  int ret;

  if (capture_type == V4L2_BUF_TYPE_STILL_CAPTURE)
    {
      ret = ioctl(cam_fd, VIDIOC_TAKEPICT_START, 0);
      if (ret < 0)
        {
          printf("Failed to start taking picture\n");
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stop_capture()
 *
 * Description:
 *   Stop STILL capture stream by TAKEPICT_STOP if buf_type is STILL_CAPTURE.
 ****************************************************************************/

int stop_capture(int cam_fd)
{
  int ret;

  if (capture_type == V4L2_BUF_TYPE_STILL_CAPTURE)
    {
      ret = ioctl(cam_fd, VIDIOC_TAKEPICT_STOP, false);
      if (ret < 0)
        {
          printf("Failed to stop taking picture\n");
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: get_imgsensor_name()
 *
 * Description:
 *   Get image sensor driver name by querying device capabilities.
 ****************************************************************************/

const char *get_imgsensor_name(int fd)
{
  static struct v4l2_capability cap;

  ioctl(fd, VIDIOC_QUERYCAP, (unsigned long)&cap);

  return (const char *)cap.driver;
}

int prepare_stream(int v_fd)
{
  int ret;
  /* Prepare for STILL_CAPTURE stream.
   *
   * The video buffer mode is V4L2_BUF_MODE_FIFO mode.
   * In this FIFO mode, if all VIDIOC_QBUFed frame buffers are captured image
   * and no additional frame buffers are VIDIOC_QBUFed, the capture stops and
   * waits for new VIDIOC_QBUFed frame buffer.
   * And when new VIDIOC_QBUF is executed, the capturing is resumed.
   *
   * Allocate frame buffers for JPEG size (512KB).
   * Set FULLHD size in ISX012 case, QUADVGA size in ISX019 case or other
   * image sensors,
   * Number of frame buffers is defined as STILL_BUFNUM(1).
   * And all allocated memorys are VIDIOC_QBUFed.
   */

  if (DEFAULT_CAPTURE_NUM != 0)
  {
    /* Determine image size from connected image sensor name,
     * because video driver does not support VIDIOC_ENUM_FRAMESIZES
     * for now.
     */

    sensor = get_imgsensor_name(v_fd);
    if (strncmp(sensor, "ISX012", strlen("ISX012")) == 0)
    {
      w = VIDEO_HSIZE_FULLHD;
      h = VIDEO_VSIZE_FULLHD;
    }
    else if (strncmp(sensor, "ISX019", strlen("ISX019")) == 0)
    {
      w = VIDEO_HSIZE_QUADVGA;
      h = VIDEO_VSIZE_QUADVGA;
    }
    else
    {
      w = VIDEO_HSIZE_QUADVGA;
      h = VIDEO_VSIZE_QUADVGA;
    }

    ret = camera_prepare(v_fd, V4L2_BUF_TYPE_STILL_CAPTURE,
                         V4L2_BUF_MODE_FIFO, V4L2_PIX_FMT_JPEG,
                         w, h,
                         &buffers_still, STILL_BUFNUM, IMAGE_JPG_SIZE);
    if (ret != OK)
    {
      return ret;
    }
  }

  /* Prepare for VIDEO_CAPTURE stream.
   *
   * The video buffer mode is V4L2_BUF_MODE_RING mode.
   * In this RING mode, if all VIDIOC_QBUFed frame buffers are captured image
   * and no additional frame buffers are VIDIOC_QBUFed, the capture continues
   * as the oldest image in the V4L2_BUF_QBUFed frame buffer is reused in
   * order from the captured frame buffer and a new camera image is
   * recaptured.
   *
   * Allocate freame buffers for QVGA RGB565 size (320x240x2=150KB).
   * Number of frame buffers is defined as VIDEO_BUFNUM(3).
   * And all allocated memorys are VIDIOC_QBUFed.
   */

  ret = camera_prepare(v_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       V4L2_BUF_MODE_RING, V4L2_PIX_FMT_RGB565,
                       VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA,
                       &buffers_video, VIDEO_BUFNUM, IMAGE_RGB_SIZE);
  if (ret != OK)
  {
    return ret;
  }
  return ret;
}

int camlib_init(int cam_fd){
  int ret;

  save_dir = cutil_setpath();
  if(save_dir != "/dev/sd0"){
    snwarn("WARNING: can't get sd0 as write image path\n");
  }

  ret = prepare_stream(cam_fd);
  if (ret != 0)
  {
    snerr("ERROR: Failed to prepare stream!\n");
    return ERROR;
  }

  is_initialized = true;
  return OK;
}

void camlib_clear(void)
{
  free_buffer(buffers_video, VIDEO_BUFNUM);
  free_buffer(buffers_still, STILL_BUFNUM);
}