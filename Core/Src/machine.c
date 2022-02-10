#include "machine.h"
#include "stm32g4xx_ll_gpio.h"
#include <string.h>

void receiveChar(machine_struct * machine,uint8_t c){ 
  if((c=='\n') && machine->cursor_buff>0){ 
    machine->buffer[machine->cursor_buff]='\0';
    for(int i= machine->cursor_buff;i<64;i++)
    machine->buffer[i]='\0';
    machine->cursor_buff=0;
  }else{
    if(c!='\r'&&c!='\0'&&c!='\n')
    machine->buffer[machine->cursor_buff++]=c;
  }

}
void waitUntilConnection(machine_struct * machine){
  while (!strstr(machine->buffer,"+QBTIND: \"conn\"")){
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
  }
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
  }