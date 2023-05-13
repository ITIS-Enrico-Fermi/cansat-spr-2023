#include <flight_fsm.h>
#include <math.h>
#include <arch/chip/gnss.h> //gnss driver structures
#include <isxcamera.h>      //custom camera library utils

/* TODO check every error condition consequence */
/* TODO change sleep method to millis interval */

#define ALTITUDE_STABILIZE 5
#define MAX_PHOTO 7
#define PHOTO_DELAY 10000
#define MEASURE_DELAY 3000

int ret;
int uv_fd, gyro_fd, baro_fd, camera_fd, gps_fd, radio_fd; // < file descriptors of sensors
struct pollfd fds;

/* Sensors data */
baro_t baro;
gyro_t gyro;
uint16_t uv;
gnss_t gps;
struct lora_packet pkt = {.counter = 0};

/* Define structure length */
int len_baro = sizeof(baro_t);
int len_gyro = sizeof(gyro_t);
int len_uv = sizeof(uint16_t);

/* GPS data */
static uint32_t posfixflag;
static struct cxd56_gnss_positiondata_s posdat;
int posperiod;
sigset_t mask;
const struct timespec waitgps = {
    .tv_sec = 3,
    .tv_nsec = 0};

float ground_altitude = 0;
float pressureToAltitude(float pres);

// Funzioni di stato
int boot_state(void)
{
  bool errorBoot = false;
  radio_fd = open(RADIO_DEV_NAME, O_WRONLY);
  if (radio_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for radio module");
    errorBoot = true;
  }
  camera_fd = open(CAMERA_DEV_NAME, 0);
  if (camera_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for camera module");
    errorBoot = true;
  }
  gps_fd = open(GPS_DEV_NAME, O_RDONLY);
  if (gps_fd < 0)
  {
    syslog(LOG_ERR, "Can't open GNSS character device at %s. Error code: %d\n", GPS_DEV_NAME, errno);
    errorBoot = true;
  }
  uv_fd = open(UV_DEV_NAME, O_RDONLY);
  if (uv_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for light sensor");
    errorBoot = true;
  }
  gyro_fd = open(GYRO_DEV_NAME, O_RDONLY);
  if (gyro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for gyro sensor");
    errorBoot = true;
  }
  baro_fd = open(BARO_DEV_NAME, O_RDONLY | O_NONBLOCK);
  if (baro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for baro sensor");
    errorBoot = true;
  }

  /* START SETUP BARO0 IOCTL */
  unsigned int interval = 1000000;
  unsigned int latency = 0;
  ret = ioctl(baro_fd, SNIOC_SET_INTERVAL, interval);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      _info("Failed to set interval for sensor:%s, ret:%s\n",
            "bmp280", strerror(errno));
    }
    errorBoot = true;
  }
  ret = ioctl(baro_fd, SNIOC_BATCH, latency);
  if (ret < 0)
  {
    ret = -errno;
    if (ret != -ENOTSUP)
    {
      _info("Failed to batch for sensor:%s, ret:%s\n",
            "bmp280", strerror(errno));
    }
    errorBoot = true;
  }
  fds.fd = baro_fd;
  fds.events = POLLIN;
  /* END SETUP BARO0 IOCTL */

  /* START SETUP RADIO0 IOCTL */
  ret = ioctl(radio_fd, 0);
  if (ret < 0)
  {

    _info("Failed to reset radio0\n");
    errorBoot = true;
  }
  /* END SETUP RADIO0 IOCTL */

  /* START SETUP GPS0 IOCTL */
  /* Prepare a signal set and enable GNSS signal only. */
  sigemptyset(&mask);
  sigaddset(&mask, GNSS_USERSPACE_SIG);
  ret = sigprocmask(SIG_BLOCK, &mask, NULL);
  if (ret != OK)
  {
    _info("sigprocmask failed. %d\n", ret);
    errorBoot = true;
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
    errorBoot = true;
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
    errorBoot = true;
  }

  /* Set the type of satellite system used by GNSS. */
  set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS;

  ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
  if (ret < 0)
  {
    _info("Can't set satellite system.\n");
    errorBoot = true;
  }
  /* END set GNSS parameters. */

  /* Initial positioning measurement becomes cold start if specified hot
   * start, so working period should be long term to receive ephemeris. */

  posperiod = 200;
  posfixflag = 0;

  /* Start GNSS. */
  ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
  {
    _info("Error while starting GNSS in Hot Mode. Error code: %d\n", errno);
    errorBoot = true;
  }
  else
    syslog(LOG_INFO, "GNSS subsystem started correctly.\n");

  /* END SETUP GPS0 IOCTL */

  /* Camera lib init */
  ret = camlib_init(camera_fd);
  if (ret < 0)
  {
    snerr("Can't init camlib!\n");
  }

  if (errorBoot)
  {
    /* TODO: reboot here */
    _info("FATAL: ERROR OCCURED ON BOOT!!\n");
    return ERROR;
  }
  return IDLE;
}

/* Idle state is focused on LOW POWER MODE and LAUNCH DETECTION */
int idle_state(void)
{
  static uint8_t calibrateCounter = ALTITUDE_STABILIZE;
  float new_altitude = 0;
  // read baro
  if (poll(&fds, 1, -1) > 0)
  {
    if (read(baro_fd, &baro, len_baro) >= len_baro)
    {
    }
    else
    {
      snerr("Can't read baro0\n");
    }
  }
  ret = sigtimedwait(&mask, NULL, &waitgps);
  if (ret != GNSS_USERSPACE_SIG)
  {
    snerr("Waited signal, but instead of GNSS signal got: %d\n", ret);
  }
  else /* We read gps from the beginning since it takes minutes to sync from boot */
  {
    ret = read(gps_fd, &posdat, sizeof(posdat));
    if (ret < 0)
    {
      snerr("Can't read gps0\n");
    }
  }

  /* FSM IDLE --> COLLECT CONDITION:
   * altitude should be 300m high at least
   */
  new_altitude = pressureToAltitude((float)baro.pressure);
  _info("new_altitude = %f\n", new_altitude);

  /* Altitude calibration takes ALTITUDE_STABILIZE * 2 seconds */
  if (calibrateCounter > 0)
  {
    ground_altitude += new_altitude;
    calibrateCounter--;
    if (calibrateCounter == 0)
    {
      ground_altitude /= (float)ALTITUDE_STABILIZE;
      _info("ground_altitude is %f\n", ground_altitude);
    }
    sleep(2);
    return IDLE;
  }

  //new_altitude += 479; /* TODO this is made for testing, REMOVE BEFORE LAUNCH! */
  /* When cansat gets at least 400m higher then ground */
  if (new_altitude - 450 > ground_altitude)
  {
    _info("Switching state from IDLE to COLLECT\n");
    return COLLECT;
  }
  sleep(5);
  return IDLE;
}

/* COLLECT state is the crucial phase. Every measure is taken here
 * each packet is sent 3 times to help the recever to get the message
 */
int collect_state(void)
{
  static int photoCount = MAX_PHOTO;
  static unsigned long lastPhoto = 0;
  static unsigned long lastMeasure = 0;

  /* Get clocktime ms */
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  unsigned long millis = now.tv_sec * 1000 + now.tv_nsec / 1000000;

  if (millis - lastMeasure > MEASURE_DELAY)
  {
    lastMeasure = millis;
    /* Read baro data. */
    if (poll(&fds, 1, -1) > 0)
    {
      if (read(baro_fd, &baro, len_baro) >= len_baro)
      {
      }
      else
      {
        snerr("Can't read baro0\n");
      }
    }
    /* Read gyro data. */
    ret = read(gyro_fd, &gyro, len_gyro);
    if (ret < 0)
    {
      snerr("Can't read gyro0\n");
    }
    /* Wait GPS data. */
    ret = sigtimedwait(&mask, NULL, &waitgps);
    if (ret != GNSS_USERSPACE_SIG)
    {
      gps.latitude = 102.0F; // error code
      gps.longitude = 102.F; // error code
      snerr("Waited signal, but instead of GNSS signal got: %d\n", ret);
    }
    else /* Read only if there's usefull data */
    {
      ret = read(gps_fd, &posdat, sizeof(posdat));
      if (ret < 0)
      {
        snerr("Can't read gps0\n");
      }
      else
      {
        parse_gps(&posdat, &gps);
      }
    }

    ret = read(uv_fd, &uv, len_uv);
    if (ret < 0)
    {
      _info("Can't read uv0\n");
    }
    uint8_t *uv_ptr = (uint8_t *)&uv;

    /* Prepare packet after measure */
    pkt.pressure = baro.pressure;
    pkt.gps = gps;
    pkt.gyro = gyro;
    pkt.uv = uv_ptr[0];
    pkt.imgclass = 2;
    pkt.counter++;
    if(pkt.counter %5 == 0)
    {
      pkt.imgclass = 6;
    }
  }

  if (millis - lastPhoto > PHOTO_DELAY)
  {
    lastPhoto = millis;
    /* Not the cleanest code here.. */
    if (photoCount > 0)
    {
      /* Take photo */
      ret = start_capture(camera_fd);
      if (ret != OK)
      {
        snerr("Can't start capture...\n");
      }
      else
      {
        ret = shoot_photo(camera_fd);
        if (ret != OK)
        {
          snerr("Can't shoot photo...\n");
        }
        else
        {
          photoCount--;
          _info("%d photo left.\n", photoCount);
        }
        ret = stop_capture(camera_fd);
        if (ret != OK)
        {
          snerr("Can't stop capture...\n");
        }
      }
    }
  }

  /* Tensorflow function for imgclass result */

  ret = write(radio_fd, &pkt, sizeof(struct lora_packet));
  if (ret < 0)
  {
    snerr("Error while sending packet\n");
  }
  _info("LoRa packet inside:\n");
  _info("pres: %f\n", pkt.pressure);
  _info("accelx %d; accely %d; accelz %d\n", pkt.gyro.accel.x, pkt.gyro.accel.y, pkt.gyro.accel.z);
  _info("temp %d\n", pkt.gyro.temp);
  _info("rotox %d; rotoy %d; rotoz %d\n", pkt.gyro.roto.x, pkt.gyro.roto.y, pkt.gyro.roto.z);
  _info("lat: %f; lon: %f\n", pkt.gps.latitude, pkt.gps.longitude);
  _info("uv: %d\n", pkt.uv);
  _info("imgclass: %d\n", pkt.imgclass);
  _info("counter: %d\n", pkt.counter);

  float now_altitude = pressureToAltitude(baro.pressure);
  if (now_altitude - ground_altitude < 30)
  {
    return RECOVER;
  }
  sleep(1);
  return COLLECT;
}

/* Recover_state sends only GPS data */
int recover_state(void)
{
  /* Wait GPS data. */
  ret = sigtimedwait(&mask, NULL, &waitgps);
  if (ret != GNSS_USERSPACE_SIG)
  {
    gps.latitude = 102.0F; // error code
    gps.longitude = 102.F; // error code
    snerr("Waited signal, but instead of GNSS signal got: %d\n", ret);
  }
  else /* Read only if there's usefull data */
  {
    ret = read(gps_fd, &posdat, sizeof(posdat));
    if (ret < 0)
    {
      snerr("Can't read gps0\n");
    }
    else
    {
      parse_gps(&posdat, &gps);
    }
  }

  pkt.pressure = 0;
  pkt.uv = 0;
  pkt.gyro.accel.x = 0;
  pkt.gyro.accel.x = 0;
  pkt.gyro.accel.x = 0;
  pkt.gyro.temp = 0;
  pkt.gyro.roto.x = 0;
  pkt.gyro.roto.y = 0;
  pkt.gyro.roto.z = 0;
  pkt.gps = gps;
  pkt.imgclass = 10;
  pkt.counter++;

  _info("LoRa packet inside:\n");
  _info("pres: %f\n", pkt.pressure);
  _info("accelx %d; accely %d; accelz %d\n", pkt.gyro.accel.x, pkt.gyro.accel.y, pkt.gyro.accel.z);
  _info("temp %d\n", pkt.gyro.temp);
  _info("rotox %d; rotoy %d; rotoz %d\n", pkt.gyro.roto.x, pkt.gyro.roto.y, pkt.gyro.roto.z);
  _info("lat: %f; lon: %f\n", pkt.gps.latitude, pkt.gps.longitude);
  _info("uv: %d\n", pkt.uv);
  _info("imgclass: %d\n", pkt.imgclass);
  _info("counter: %d\n", pkt.counter);

  printf("Recover packet sent.\n");
  ret = write(radio_fd, &pkt, sizeof(struct lora_packet));
  if (ret < 0)
  {
    snerr("Error while sending packet\n");
  }
  sleep(15);
  return RECOVER;
}

/*
 * When does the fsm.. ends?
 * Cansat never ends.. you'll see us again.
 * soon.
 */

float pressureToAltitude(float pres)
{
  pres *= 100.0f;                                         // converti la pressione da hPa a Pa
  const float seaLevelPressure = 101325.0f;               // pressione a livello del mare (Pa)
  const float altitudeFactor = 44330.0f;                  // fattore di conversione altitudine/pressione (m/Pa)
  const float pressureSeaLevel = pres / seaLevelPressure; // pressione relativa al livello del mare
  float altitude = altitudeFactor * (1.0f - powf(pressureSeaLevel, 0.1903f));
  return altitude;
}
