#include <flight_fsm.h>
#include <math.h>

int ret;
int uv_fd, gyro_fd, baro_fd, camera_fd, gps_fd, radio_fd; // < file descriptors of sensors
struct pollfd fds;
struct lora_packet pkt = {.counter = 0};

/* Define structure length */
int len_baro = sizeof(baro_t);
int len_gyro = sizeof(gyro_t);
int len_uv = sizeof(uint16_t);

/* Allocate buffer space */
uint8_t *baro_buf;
uint8_t *gyro_buf;
uint8_t *uv_buf;

/* GPS data */
static uint32_t posfixflag;
static struct cxd56_gnss_positiondata_s posdat;
int posperiod;
sigset_t mask;
const struct timespec waitgps = {
    .tv_sec = 3,
    .tv_nsec = 0};

#define ALTITUDE_LEN 100
float pressureToAltitude(float pres);
bool isLaunched(float pres);
float *initCircularBuffer(int len);

// Funzioni di stato
int boot_state(void)
{
  baro_buf = malloc(len_baro);
  gyro_buf = malloc(len_gyro);
  uv_buf = malloc(len_uv);

  radio_fd = open(RADIO_DEV_NAME, O_WRONLY);
  if (radio_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for radio module");
    return ERROR;
  }
  camera_fd = open(CAMERA_DEV_NAME, 0);
  if (camera_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for camera module");
    return ERROR;
  }
  gps_fd = open(GPS_DEV_NAME, O_RDONLY);
  if (gps_fd < 0)
  {
    syslog(LOG_ERR, "Can't open GNSS character device at %s. Error code: %d\n", GPS_DEV_NAME, errno);
    return ERROR;
  }
  uv_fd = open(UV_DEV_NAME, O_RDONLY);
  if (uv_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for light sensor");
    return -1;
  }
  gyro_fd = open(GYRO_DEV_NAME, O_RDONLY);
  if (gyro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for gyro sensor");
    return -1;
  }
  baro_fd = open(BARO_DEV_NAME, O_RDONLY | O_NONBLOCK);
  if (baro_fd < 0)
  {
    syslog(LOG_ERR, "Can't open file descriptor for baro sensor");
    return -1;
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
    return ERROR;
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
    return ERROR;
  }
  fds.fd = baro_fd;
  fds.events = POLLIN;
  /* END SETUP BARO0 IOCTL */

  /* START SETUP RADIO0 IOCTL */
  ret = ioctl(radio_fd, 0);
  if (ret < 0)
  {

    _info("Failed to reset radio0\n");
    return ERROR;
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

  /* Initial positioning measurement becomes cold start if specified hot
   * start, so working period should be long term to receive ephemeris. */

  posperiod = 200;
  posfixflag = 0;

  /* Start GNSS. */
  ret = ioctl(gps_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    _info("Error while starting GNSS in Hot Mode. Error code: %d\n", errno);
  else
    syslog(LOG_INFO, "GNSS subsystem started correctly.\n");

  /* END SETUP GPS0 IOCTL */

  if (ret < 0)
  {
    return ERROR;
  }
  else
  {
    _info("Switching state from BOOT to IDLE\n");
    return IDLE;
  }

  _info("FATAL: ERROR OCCURED ON BOOT!!\n");
  return ERROR;
}

int idle_state(void)
{
  baro_t *baro;

  // read baro
  _info("Reading baro..\n");
  if (poll(&fds, 1, -1) > 0)
  {
    if (read(baro_fd, baro_buf, len_baro) >= len_baro)
    {
    }
    else
    {
      return ERROR;
    }
  }
  baro = (baro_t *)baro_buf;
  _info("waiting gps signal..\n");
  ret = sigtimedwait(&mask, NULL, &waitgps);
  //ret = sigwaitinfo(&mask, NULL);
  if (ret != GNSS_USERSPACE_SIG)
  {
    _info("Waited signal, but instead of GNSS signal got: %d\n", ret);
  }else _info("GPS signal received!\n");

  /* Read and print POS data. */
  ret = read(gps_fd, &posdat, sizeof(posdat));
  if (ret < 0)
  {
    _info("Can't read gps0\n");
    return ERROR;
  }
  // ...comportamento di IDLE...

  /* FSM IDLE --> COLLECT CONDITION:
   * altitude should be 300m high at least
   */
  float altitude = pressureToAltitude((float)baro->pressure);
  _info("altitude: %f\n", altitude);
  if (isLaunched(altitude))
  {
    _info("Switching state from IDLE to COLLECT");
    return COLLECT;
  }
  _info("Sleeping..\n");
  sleep(5);
  return IDLE;
}

int collect_state(void)
{
  baro_t *baro;
  gyro_t *gyro;
  gnss_t *gps;
  uint8_t *uv;

  /* Wait baro data. */
  if (poll(&fds, 1, -1) > 0)
  {
    if (read(baro_fd, baro_buf, len_baro) >= len_baro)
    {
    }
    else
    {
      return ERROR;
    }
  }
  baro = (baro_t *)baro_buf;
  /* Wait gyro data. */
  ret = read(gyro_fd, gyro_buf, len_gyro);
  if (ret < 0)
  {
    _info("Can't read baro0\n");
    return ERROR;
  }
  gyro = (gyro_t *)gyro_buf;
  /* Wait GPS data. */
  ret = sigtimedwait(&mask, NULL, &waitgps);
  // ret = sigwaitinfo(&mask, NULL); // we don't like blocking methods
  if (ret != GNSS_USERSPACE_SIG)
  {
    _info("Waited signal, but instead of GNSS signal got: %d\n", ret);
    return ERROR;
  }
  /* Read and print POS data. */
  ret = read(gps_fd, &posdat, sizeof(posdat));
  if (ret < 0)
  {
    _info("Can't read gps0\n");
    return ERROR;
  }
  parse_gps(&posdat, gps);

  // ...comportamento di COLLECT...
  ret = read(uv_fd, uv_buf, len_uv);
  if (ret < 0)
  {
    _info("Can't read uv0\n");
    return ERROR;
  }

  pkt.pressure = baro->pressure;
  pkt.gps = *gps;
  pkt.gyro = *gyro;
  pkt.uv = *uv;
  pkt.counter++;

  ret = write(radio_fd, &pkt, sizeof(struct lora_packet));
  _info("LoRa packet inside:\n");
  _info("pres: %f\n", pkt.pressure);
  _info("accelx %d; accely %d; accelz %d", pkt.gyro.accel.x, pkt.gyro.accel.y, pkt.gyro.accel.z);
  _info("temp %d", pkt.gyro.temp);
  _info("rotox %d; rotoy %d; rotoz %d", pkt.gyro.roto.x, pkt.gyro.roto.y, pkt.gyro.roto.z);
  _info("lat: %f; lon: %f", pkt.gps.latitude, pkt.gps.longitude);
  _info("uv: %d", pkt.uv);

  if (false)
  {
    return RECOVER;
  }
  sleep(5);
  return COLLECT;
}

int recover_state(void)
{

  // ...comportamento di RECOVER...

  return RECOVER;
}

/*
 * When does the fsm.. ends?
 * Cansat never ends.. you'll see us again.
 * soon.
 */

bool isLaunched(float alt)
{
  static int cindex = 0;
  static float *circularAltitude = NULL;
  bool result = false;

  if (circularAltitude == NULL)
  {
    _info("Circularbuffer initialized\n");
    circularAltitude = initCircularBuffer(ALTITUDE_LEN);
  }

  _info("Scrolling through conditions\n");
  for (int i = 0; i < ALTITUDE_LEN; i++)
  {
    if (alt - 300 > circularAltitude[i])
    {
      _info("Detected lauch from %f to %f at %d^ round", circularAltitude[i], alt, i);
      result = true;
      break;
    }
  }
  circularAltitude[cindex] = alt;
  cindex++;
  if (cindex == ALTITUDE_LEN)
  {
    cindex = 0;
  }

  return result;
}

float *initCircularBuffer(int len)
{
  float *circularBuffer = malloc(sizeof(float) * len);
  for (int i = 0; i < len; i++)
  {
    circularBuffer[i] = 0;
  }
  return circularBuffer;
}

float pressureToAltitude(float pres)
{
  pres *= 100.0f;                                         // converti la pressione da hPa a Pa
  const float seaLevelPressure = 101325.0f;               // pressione a livello del mare (Pa)
  const float altitudeFactor = 44330.0f;                  // fattore di conversione altitudine/pressione (m/Pa)
  const float pressureSeaLevel = pres / seaLevelPressure; // pressione relativa al livello del mare
  float altitude = altitudeFactor * (1.0f - powf(pressureSeaLevel, 0.1903f));
  return altitude;
}
