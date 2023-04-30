
#include <nuttx/config.h>
#include <stdio.h>
#include <nuttx/video/video.h>
#include <isxcamera.h>

int main(int argc, FAR char *argv[])
{
  int ret;
  int cam_fd;
  int capture_num = DEFAULT_CAPTURE_NUM;

  /* Open the device file. */
  cam_fd = open("/dev/sensor/sensor_camera0", 0);
  if (cam_fd < 0)
  {
    printf("ERROR: Failed to open video0.errno = %d\n", errno);
    return ERROR;
  }

  /* init library before doing ANYTHING */
  ret = camlib_init(cam_fd);

  printf("Start capturing...\n");
  /* Avvio della fase lettura */
  ret = start_capture(cam_fd);
  if (ret != OK)
  {
    printf("Can't start capture...\n");
    return ERROR;
  }

  /************************************
   * MODALITÀ ACCORCIATA
   * Questa modalità fa tutto da sola. Scatta la foto, la scrive sul disco
   * e rilascia la foto. tutto dentro a shoot_photo()
   * Più semplice di così non si può
   */
  while (capture_num)
  {
    ret = shoot_photo(cam_fd);
    if (ret != OK)
    {
      printf("Can't shoot photo...\n");
      return ERROR;
    }

    capture_num--;
    printf("%d photo left.\n", capture_num);
  }

  /************************************
   * MODALITÀ STANDARD
   * Qui sostanzialmente il processo con l'uso della libreria
   * è solo semplificato, ma le funzioni sono quelle..
   */
  /*
  while (capture_num)
  {
    ret = get_image(cam_fd);
    if (ret != OK)
    {
      printf("Can't get image...\n");
      return ERROR;
    }

    write_image();

    ret = release_image(cam_fd);
    if (ret != OK)
    {
      printf("Can't release image...\n");
      return ERROR;
    }

    capture_num--;
  }
  */

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
