#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/timers/pwm.h>


#define PWM_DEVPATH "/dev/pwm0"

int main(int argc, FAR char *argv[]) {
  struct pwm_info_s info;
  int fd;
  int ret;


  fd = open(PWM_DEVPATH, O_RDONLY);
  if (fd < 0) {
    _err("Open %s failed: %d\n", PWM_DEVPATH, errno);
    return -1;
  }

  //Configure PWM

  info = (struct pwm_info_s){
    .frequency = CONFIG_CANSAT_APPS_PWM_TEST_FREQUENCY,
    .duty = b16divi(uitoub16(CONFIG_CANSAT_APPS_PWM_TEST_DUTYCYCLE), 100)
  };

  printf("pwm_main: starting output "
         "with frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
         info.frequency, (uint32_t)info.duty);

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, &info);
  if (ret < 0) {
    _err("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    return -1;
  }

  /* Start the pulse train.  Since the driver was opened in blocking
   * mode, this call will block if the count value is greater than zero.
   */
  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0) {
    _err("ioctl(PWMIOC_START) failed: %d\n", errno);
    return -1;
  }

  usleep(CONFIG_CANSAT_APPS_PWM_TEST_DURATION);

  /* Stop the pulse train */
  printf("pwm_main: stopping output\n");

  ret = ioctl(fd, PWMIOC_STOP, 0);
  if (ret < 0) {
    _err("ioctl(PWMIOC_STOP) failed: %d\n", errno);
    return -1;
  }

  close(fd);
  return 0;
}
