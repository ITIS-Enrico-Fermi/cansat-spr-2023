#include <flight_senutils.h>

void parse_gps(FAR void *in, FAR void *out)
{
    int posfixflag;
    struct cxd56_gnss_positiondata_s *posdat = (struct cxd56_gnss_positiondata_s *)in;
    struct cxd56_gnss_dms_s dmf;
    /* Print positioning data */
    //printf(">Hour:%d, minute:%d, sec:%d, usec:%ld\n", posdat.receiver.time.hour, posdat.receiver.time.minute, posdat.receiver.time.sec, posdat.receiver.time.usec);
    
    if (posdat->receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID) {
      /* 2D fix or 3D fix.
      * Convert latitude and longitude into dmf format and print it. */

      posfixflag = 1;
      printf(">LAT double: %lf\n", posdat->receiver.latitude);
      printf(">LAT float: %f\n", (float)posdat->receiver.latitude);
      double_to_dmf(posdat->receiver.latitude, &dmf);
      printf(">LAT %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);

      printf(">LNG double: %lf\n", posdat->receiver.longitude);
      printf(">LNG float: %f\n", (float)posdat->receiver.longitude);
      double_to_dmf(posdat->receiver.longitude, &dmf);
      printf(">LNG %d.%d.%04ld\n", dmf.degree, dmf.minute, dmf.frac);
    } else {
      /* No measurement. */

      printf(">No Positioning Data\n");
    }
}

void parse_gyro(void *in, void *out)
{
    uint8_t *ubuffer = (uint8_t *) in;
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
    rawAccX = ubuffer[0] << 8 | ubuffer[1];
    rawAccY = ubuffer[2] << 8 | ubuffer[3];
    rawAccZ = ubuffer[4] << 8 | ubuffer[5];

    rawTemp = ubuffer[6] << 8 | ubuffer[7];

    rawGyroX = ubuffer[8] << 8 | ubuffer[9];
    rawGyroY = ubuffer[10] << 8 | ubuffer[11];
    rawGyroZ = ubuffer[12] << 8 | ubuffer[13];

    temperature = (rawTemp / 340.0) + 36.53;

    // default for the driver
    float accel_scale = MPU6050_RANGE_8_G;

    // setup range dependant scaling
    accX = ((float)rawAccX) / accel_scale;
    accY = ((float)rawAccY) / accel_scale;
    accZ = ((float)rawAccZ) / accel_scale;

    // default for the driver
    float gyro_scale = MPU6050_RANGE_1000_DEG;

    gyroX = ((float)rawGyroX) / gyro_scale;
    gyroY = ((float)rawGyroY) / gyro_scale;
    gyroZ = ((float)rawGyroZ) / gyro_scale;

    printf("## mpu6050 ##\n");
    printf("x_accel: %.2f g\ny_accel: %.2f g\nz_accel: %.2f g\n", accX, accY, accZ);
    printf("temp: %.2f\n", temperature);
    printf("x_gyro: %.2f  dps\ny_gyro: %.2f dps\nz_gyro: %.2f dps\n", gyroX, gyroY, gyroZ);
}

void double_to_dmf(double x, struct cxd56_gnss_dms_s *dmf)
{
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