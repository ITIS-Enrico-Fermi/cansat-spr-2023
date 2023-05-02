#include <flight_senutils.h>

int uv_fd, gyro_fd, baro_fd, radio_fd; // < file descriptors of sensors
struct pollfd fds;

int open_sensors(void)
{
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
#ifdef CONFIG_RF_RFM95
    radio_fd = open(RADIO_DEV_NAME, O_WRONLY);
    if (radio_fd < 0)
    {
        syslog(LOG_ERR, "Can't open file descriptor for radio module");
        return -1;
    }
#endif
    return 0;
}

int setup_sensors(void)
{
    int ret;
    /* Start setup bmp280 */
    unsigned int interval = 1000000;
    unsigned int latency = 0;
    ret = ioctl(baro_fd, SNIOC_SET_INTERVAL, interval);
    if (ret < 0)
    {
        ret = -errno;
        if (ret != -ENOTSUP)
        {
            printf("Failed to set interval for sensor:%s, ret:%s\n",
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
            printf("Failed to batch for sensor:%s, ret:%s\n",
                   "bmp280", strerror(errno));
        }
        return ERROR;
    }

    fds.fd = baro_fd;
    fds.events = POLLIN;

    /* End setup bmp280 */
    return 0;
}

void close_sensors(void)
{
    close(uv_fd);
    close(gyro_fd);
    close(baro_fd);
}

/****************************************************/
/* READ SENSOR FUNCTIONS ****************************/
/****************************************************/

int read_baro(uint8_t *baro_buf, int len_baro)
{
    if (poll(&fds, 1, -1) > 0)
    {
        if (read(baro_fd, baro_buf, len_baro) >= len_baro)
        {
            // baro_val = (baro_t *)baro_buf;
        }
    }
    return OK;
}

int read_gyro(uint8_t *gyro_buf, int len_gyro)
{
    int ret;
    ret = read(gyro_fd, gyro_buf, len_gyro);
    if (ret < 0)
    {
        printf("Error while reading gyro");
        return ERROR;
    }
    // gyro_val = (gyro_t *)gyro_buf;
    return OK;
}

int read_uv(uint8_t *uv_buf, int len_uv)
{
    int ret;
    ret = read(uv_fd, uv_buf, len_uv);
    if (ret < 0)
    {
        printf("Error while reading gyro");
        return ERROR;
    }
    // uv_val = (uint8_t *)uv_buf;
    return OK;
}

void parse_gyro(uint8_t *buffer)
{
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    rawTemp = buffer[6] << 8 | buffer[7];

    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

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