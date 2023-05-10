#include "pwm_userspace.h"

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

static int fd;  // < file descriptor of PWM device driver

bool pwm_config(const char *device_path) {
    fd = open(device_path, O_RDONLY);
    if (fd < 0) {
        _err("Open %s failed: %d\n", PWM_DEVPATH, errno);
        return false;
    }

    return true;
}

bool pwm_config(const int frequency, const int duty_cycle) {
    struct pwm_info_s info = (struct pwm_info_s){
        .frequency = frequency,
        .duty = b16divi(uitoub16(duty_cycle), 100)
    };

    _info("pwm_main: starting output "
            "with frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
            info.frequency, (uint32_t)info.duty);

    int ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, &info);
    if (ret < 0) {
        _err("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
        return false;
    }

    return true;
}

bool pwm_start() {
    int ret = ioctl(fd, PWMIOC_START, 0);
    if (ret < 0) {
        _err("ioctl(PWMIOC_START) failed: %d\n", errno);
        return false;
    }

    return true;
}

void pwm_stop() {
    int ret = ioctl(fd, PWMIOC_STOP, 0);
    if (ret < 0) {
        _err("ioctl(PWMIOC_STOP) failed: %d\n", errno);
        return -1;
    }
}

void pwm_cleanup() {
    close(fd);
}