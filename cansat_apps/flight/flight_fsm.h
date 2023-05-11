#pragma once

#include <flight_senutils.h>

// State functions
typedef int (*state_func_t)(void);

// Routine prototypes
int boot_state(void);
int idle_state(void);
int collect_state(void);
int recover_state(void);

// Define main states
typedef enum 
{
  BOOT,
  IDLE,
  COLLECT,
  RECOVER
} fsm_state;

/* Utility functions */
void parse_gyro(void *in, void *out);
void parse_gps(void *in, void *out);