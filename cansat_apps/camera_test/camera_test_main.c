
#include <nuttx/config.h>
#include <stdio.h>
#include <nuttx/video/video.h>
#include <camera_utils.h>

int main(int argc, FAR char *argv[])
{
  int ret;
  int cam_fd;
  int capture_num = DEFAULT_CAPTURE_NUM;

  /* Initialize video driver to create a device file
   *  It's like a register but it's required for this kind of driver
   *  TODO: move this into cxd56_bringup.c
  ret = video_initialize("/dev/camera0");
  if (ret != 0)
  {
    printf("ERROR: Failed to initialize video: errno = %d\n", errno);
    return ERROR;
  }
  */

  /* Open the device file. */
  cam_fd = open("/dev/camera0", 0);
  if (cam_fd < 0)
  {
    printf("ERROR: Failed to open video0.errno = %d\n", errno);
    return ERROR;
  }

  /* init library before doing ANYTHING */
  ret = camlib_init(cam_fd);

  printf("Start capturing...\n");
  ret = start_capture(cam_fd);
  if (ret != OK)
  {
    printf("Can't start capture...\n");
    return ERROR;
  }

  while (capture_num)
  {
    ret = get_image(cam_fd);
    if (ret != OK)
    {
      printf("Can't get image...\n");
      return ERROR;
    }
    /* Write image on persistent memory */
    write_image();

    ret = release_image(cam_fd);
    if (ret != OK)
    {
      printf("Can't release image...\n");
      return ERROR;
    }

    capture_num--;
  }

  ret = stop_capture(cam_fd);
  if (ret != OK)
  {
    printf("Can't stop capture...\n");
    return ERROR;
  }
  printf("Finish capturing...\n");

  /* Close camera device */
  close(cam_fd);

  /* Clear lib buffers */
  camlib_clear();

  return 0;
}
