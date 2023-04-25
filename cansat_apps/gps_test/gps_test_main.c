#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <debug.h>
#include <arch/chip/gnss.h>

#define GNSS_POLL_FD_NUM 1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define GNSS_USERSPACE_SIG 18

const char *GPS_DEV_PATH = "/dev/gps";


struct cxd56_gnss_dms_s
{
  int8_t   sign;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
};


static uint32_t                         posfixflag;
static struct cxd56_gnss_positiondata_s posdat;


/**
 * Convert from double format to degree-minute-frac format.
 * 
 * @param x a double value.
 * @param dmf a pointer to cxd56_gnss_dms_s structure to store the conversion result.
 * 
*/
static void double_to_dmf(double x, struct cxd56_gnss_dms_s * dmf) {
  int    b;
  int    d;
  int    m;
  double f;
  double t;

  if (x < 0) {
      b = 1;
      x = -x;
  } else {
    b = 0;
  }

  d = (int)x; /* = floor(x), x is always positive */
  t = (x - d) * 60;
  m = (int)t; /* = floor(t), t is always positive */
  f = (t - m) * 10000;

  dmf->sign   = b;
  dmf->degree = d;
  dmf->minute = m;
  dmf->frac   = f;
}


/**
 * Get position data from GNSS character device and print in a human-readable format.
*/
static int read_and_print(int fd)
{
  int ret;
  struct cxd56_gnss_dms_s dmf;

  /* Read POS data. */

  ret = read(fd, &posdat, sizeof(posdat));

  if(ret == sizeof(posdat)) {
    /* Print positioning data */
    printf(">Hour:%d, minute:%d, sec:%d, usec:%ld\n", posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);
    
    if (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID) {
      /* 2D fix or 3D fix.
      * Convert latitude and longitude into dmf format and print it. */

      posfixflag = 1;

      double_to_dmf(posdat.receiver.latitude, &dmf);
      printf(">LAT %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);

      double_to_dmf(posdat.receiver.longitude, &dmf);
      printf(">LNG %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);
    } else {
      /* No measurement. */

      printf(">No Positioning Data\n");
    }

    return OK;

  }
  
  else if (ret < 0) {

    printf("read error\n");
    return ret;
  
  } else {
  
    printf("Read size error\n");
    return ERROR;

  }

  return OK;
}


/**
 * Configure the parameters for GNSS chip.
*/
static int gnss_setparams(int fd) {
  int      ret = 0;
  uint32_t set_satellite;

  /*
   * Set the operation mode (always 1) and the notify cycle, which is the
   * frequency of a new position from the chip.
   */
  struct cxd56_gnss_ope_mode_param_s operation_mode_config = {
    .mode = 1,
    .cycle = 1000   // in milliseconds
  };
  
  ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, &operation_mode_config);
  if (ret < 0) {
    printf("Error setting Operation Mode via ioctl. Return code: %d\n", ret);
    return ret;
  }

  /* Set the type of satellite system used by GNSS. */
  set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS | CXD56_GNSS_SAT_GALILEO;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
  if (ret < 0) {
    printf("Can't set satellite system.\n");
    return ret;
  }

  return OK;
}


int main(int argc, FAR char *argv[]) {
  int      fd;
  int      ret;
  int      posperiod;
  sigset_t mask;



  printf("Starting GNSS (global navigation satellite system) test app\n");


  fd = open(GPS_DEV_PATH, O_RDONLY);
  if (fd < 0) {
    syslog(LOG_ERR, "Can't open GNSS character device at %s. Error code: %d\n", GPS_DEV_PATH, errno);
    return -ENODEV;
  }


  /* Prepare a signal set and enable GNSS signal only. */
  sigemptyset(&mask);
  sigaddset(&mask, GNSS_USERSPACE_SIG);
  ret = sigprocmask(SIG_BLOCK, &mask, NULL);
  if (ret != OK) {
    printf("sigprocmask failed. %d\n", ret);
  }

  /* Set the signal to notify GNSS events. */
  struct cxd56_gnss_signal_setting_s setting = {
    .fd = fd,
    .enable = true,
    .gnsssig = CXD56_GNSS_SIG_GNSS,
    .signo = GNSS_USERSPACE_SIG,
    .data = NULL
  };

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, &setting);
  if (ret < 0) {
    printf("Error while configuring signalling\n");
    return -1;
  }

  /* Set GNSS parameters. */
  ret = gnss_setparams(fd);
  if (ret != OK) {
    printf("gnss_setparams failed. %d\n", ret);
    return -1;
  }

  /* Initial positioning measurement becomes cold start if specified hot
   * start, so working period should be long term to receive ephemeris. */

  posperiod  = 200;
  posfixflag = 0;

  /* Start GNSS. */
  ret = ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    printf("Error while starting GNSS in Hot Mode. Error code: %d\n", errno);
  else
    syslog(LOG_INFO, "GNSS subsystem started correctly.");
  

  do {
      /* Wait for positioning to be fixed. After fix, idle for the specified seconds. */
      ret = sigwaitinfo(&mask, NULL);
      if (ret != GNSS_USERSPACE_SIG) {
        printf("Waited signal, but instead of GNSS signal got: %d\n", ret);
        break;
      }

      /* Read and print POS data. */
      ret = read_and_print(fd);
      if (ret < 0)
        break;

      if (posfixflag)
        posperiod--;

    } while (posperiod > 0);


  /* Stop GNSS. */
  ret = ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    printf("Error stopping GNSS\n");


  setting.enable = 0;
  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
  if (ret < 0)
    printf("signal error\n");

  sigprocmask(SIG_UNBLOCK, &mask, NULL);

  ret = close(fd);
  if (ret < 0)
    printf("Error while closing %s fd: %d\n", GPS_DEV_PATH, errno);

  return ret;
}
