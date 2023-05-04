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