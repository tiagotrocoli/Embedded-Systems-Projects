
#include <stdint.h>
#include "Board_LED.h"

void delay(void){
	uint32_t num = 0;
	for(num =0; num<500000; num++);
}
	

int main(void){
	
	while(1){
		LED_Initialize();
		LED_On(0);
		LED_On(1);
		LED_On(2);
		LED_On(3);
		delay();
		LED_Off(0);
		LED_Off(1);
		LED_Off(2);
		LED_Off(3);
		delay();
	}

}

