/*
 * main.c
 *
 *  Created on: 06-Jul-2019
 *      Author: DELL
 */

#include <stdint.h>


#define SRAM_ADDRESS1   0x20000004U

int main(void)
{

  uint32_t value = 0;
  uint32_t volatile *p = (uint32_t *) SRAM_ADDRESS1;

    while(1)
  {
   value = *p;
   if(value) break;

  }

  while(1);

  return 0;
}


