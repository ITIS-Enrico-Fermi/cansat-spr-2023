#include <flight_fsm.h>
#include <flight_senutils.h>

// Funzioni di stato
int idle_state(void *arg)
{
    int ret;
    idle_state_t *state = (idle_state_t *)arg;
    baro_t *baro;
    gyro_t *gyro;
    uint8_t *uv;

    /* JUST FOR TESTING !! */
    ret = read_baro(state->baro_buf, sizeof(baro_t));
    if (ret < 0)
    {
      return ERROR;
    }
    baro = (baro_t *) state->baro_buf;
    ret = read_gyro(state->gyro_buf, sizeof(gyro_t));
    if (ret < 0)
    {
      return ERROR;
    }
    gyro = (gyro_t *) state->gyro_buf;
    ret = read_uv(state->uv_buf, sizeof(uint16_t));
    if (ret < 0)
    {
      return ERROR;
    }


    printf("## veml6070 ##\n");
    printf("uv: %d\n", state->uv_buf[0]);
    parse_gyro(state->gyro_buf); // only for debug
    printf("## bmp280 ##\n");
    printf("timestamp: %llu\npress:%.2f (hPa)\ntemp:%.2f (C)\n",
           baro->timestamp, baro->pressure, baro->temperature);

    // ...comportamento di IDLE...

    // Controllo della condizione per passare allo stato COLLECT
    if (false)
    {
      return 1;
    }
    return 0;
}

int collect_state(void *arg)
{
    collect_state_t *state = (collect_state_t *)arg;

    // ...comportamento di COLLECT...

    if (false)
    {
        change_state(RECOVER);
    }
    return 0;
}

int recover_state(void *arg)
{
    recover_state_t *state = (collect_state_t *)arg;

    // ...comportamento di RECOVER...

    return 0;
}