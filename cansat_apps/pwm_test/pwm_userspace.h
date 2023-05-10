#pragma once

#include <stdbool.h>

bool pwm_setup(const char *device_path);

/**
 * Set frequency and duty cycle for PWM
*/
bool pwm_config(const int frequency, const int duty_cycle);

/**
 * Starts the actual hardware and generate the wave
*/
bool pwm_start();

void pwm_stop();

void pwm_cleanup();
