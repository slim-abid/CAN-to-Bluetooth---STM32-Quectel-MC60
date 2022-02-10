#include "stm32g4xx.h"

typedef struct {
 char  buffer[64];
 __IO uint8_t cursor_buff;
}machine_struct;

void receiveChar(machine_struct * machine,uint8_t c);
void waitUntilConnection(machine_struct * machine);