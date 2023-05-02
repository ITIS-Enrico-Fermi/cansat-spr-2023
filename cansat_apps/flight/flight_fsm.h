#pragma once

#include <stdint.h>

// State functions
typedef int (*state_func_t)(void *);

// Data structure for idle state
typedef struct
{
  uint8_t *baro_buf;
  uint8_t *gyro_buf;
  uint8_t *uv_buf;
} idle_state_t;

typedef struct
{
} collect_state_t;

typedef struct
{
} recover_state_t;

// Routine prototypes
int idle_state(void *arg);
int collect_state(void *arg);
int recover_state(void *arg);

// Define main states
typedef enum 
{
  IDLE,
  COLLECT,
  RECOVER
} fsm_state;