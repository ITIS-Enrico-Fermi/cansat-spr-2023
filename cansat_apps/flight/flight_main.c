#include <flight_fsm.h>
#include <flight_senutils.h>

// Dichiarazione della tabella di transizione degli stati
const state_func_t state_table[] = {
    idle_state,
    collect_state,
    recover_state};


int main(int argc, FAR char *argv[])
{
  int ret;
  struct lora_packet pkt = {.counter = 0};

  /* Define structure length */
  int len_baro = sizeof(baro_t);
  int len_gyro = sizeof(gyro_t);
  int len_uv = sizeof(uint16_t);
  
  /* Allocate buffer space */
  uint8_t *baro_buf = malloc(len_baro);
  uint8_t *gyro_buf = malloc(len_gyro);
  uint8_t *uv_buf = malloc(len_uv);

  fsm_state current_state = IDLE;

  // Set default sm state 
  idle_state_t idle = 
  {
    .baro_buf = baro_buf,
    .gyro_buf = gyro_buf,
    .uv_buf = uv_buf,
  };

#ifdef CONFIG_RF_RFM95
  int radio_fd = open(RADIO_DEV_NAME, O_WRONLY);
  if (radio_fd < 0)
  {
      syslog(LOG_ERR, "Can't open file descriptor for radio module");
      return ERROR;
  }
#endif

  ret = open_sensors();
  if (ret < 0)
  {
    printf("Error while opening sensors.\n");
    return ERROR;
  }

  ret = setup_sensors();
  if (ret < 0)
  {
    printf("Error while setting sensors ioctl.\n");
    return ERROR;
  }

  // Loop principale della macchina a stati
  while(1) {
    // Chiamata alla funzione di stato corrente
    current_state = state_table[current_state](&idle);
    sleep(2);
  }

#ifdef CONFIG_RF_RFM95
    write(radio_fd, (void *)&pkt, sizeof(pkt));
#endif

  free(gyro_buf);
  free(baro_buf);
  free(uv_buf);
  close_sensors();

#ifdef CONFIG_RF_RFM95
  close(radio_fd);
#endif
  return 0;
}
