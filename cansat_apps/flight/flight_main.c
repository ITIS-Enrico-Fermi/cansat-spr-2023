#include <flight_fsm.h>

// Dichiarazione della tabella di transizione degli stati
const state_func_t state_table[] = {
    boot_state,
    idle_state,
    collect_state,
    recover_state
  };


int main(int argc, FAR char *argv[])
{
  int ret;

  fsm_state current_state = BOOT;

  // Loop principale della macchina a stati
  while(1) {
    // Chiamata alla funzione di stato corrente
    current_state = state_table[current_state]();
    if (current_state < 0)
    {
      printf("Fatal error during inside fsm\n");
      return ERROR;
    }
    sleep(2);
  }

  return 0;
}
