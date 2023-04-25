
#include <nuttx/config.h>
#include <stdio.h>
#include <nuttx/video/video.h>
#include <camera_utils.h>

static int prepare_stream(int v_fd, const char *sensor, uint16_t w, uint16_t h,
  struct v_buffer *buffers_still, struct v_buffer *buffers_video)
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

int main(int argc, FAR char *argv[])
{
  int ret;
  int v_fd;
  int capture_num = DEFAULT_CAPTURE_NUM;
  enum v4l2_buf_type capture_type = V4L2_BUF_TYPE_STILL_CAPTURE;
  struct v4l2_buffer v4l2_buf;
  const char *save_dir;
  const char *sensor;
  uint16_t w;
  uint16_t h;

  struct v_buffer *buffers_video = NULL;
  struct v_buffer *buffers_still = NULL;

  /* Select storage to save image files */
  save_dir = cutil_setpath();

  /* Initialize video driver to create a device file
   *  It's like a register but it's requiref for this kind of driver
   *  TODO: move this into cxd56_bringup.c
   */
  ret = video_initialize("/dev/video0");
  if (ret != 0)
  {
    printf("ERROR: Failed to initialize video: errno = %d\n", errno);
    goto exit_without_cleaning_videodriver;
  }

  /* Open the device file. */
  v_fd = open("/dev/video0", 0);
  if (v_fd < 0)
  {
    printf("ERROR: Failed to open video0.errno = %d\n", errno);
    ret = ERROR;
    goto exit_without_cleaning_buffer;
  }

  ret = prepare_stream(v_fd, sensor, w, h, buffers_still, buffers_video);
  if (ret != 0)
  {
    printf("ERROR: Failed to prepare stream!\n");
    goto exit_without_cleaning_videodriver;
  }

  printf("Start capturing...\n");
  ret = start_stillcapture(v_fd, capture_type);
  if (ret != OK)
  {
    goto exit_this_app;
  }

  while (capture_num)
  {
    ret = get_camimage(v_fd, &v4l2_buf, capture_type);
    if (ret != OK)
    {
      goto exit_this_app;
    }

    cutil_writeimage(
        (uint8_t *)v4l2_buf.m.userptr,
        (size_t)v4l2_buf.bytesused,
        (capture_type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ? "RGB" : "JPG");

    ret = release_camimage(v_fd, &v4l2_buf);
    if (ret != OK)
    {
      goto exit_this_app;
    }

    capture_num--;
  }

  ret = stop_stillcapture(v_fd, capture_type);
  if (ret != OK)
  {
    goto exit_this_app;
  }
  printf("Finished capturing...\n");

exit_this_app:

  /* Close video device file makes dequeue all buffers */

  close(v_fd);

  free_buffer(buffers_video, VIDEO_BUFNUM);
  free_buffer(buffers_still, STILL_BUFNUM);

exit_without_cleaning_buffer:

  video_uninitialize();

exit_without_cleaning_videodriver:

  return 0;
}
