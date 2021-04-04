
#include "adc.h"
#include "GLCD_ST7735.h"

#define YMAX 5000
#define YMIN 0

volatile int sensorValue;

ADC_HandleTypeDef hadc1;
void drawaxes(void);
void drawInfoBar(void);
void plotData(void);

int  main(){

	HAL_Init();
	ADCI_Init();
	ST7735_Init();
	
	//drawaxes();
	//drawInfoBar();

	__enable_irq();
	
	ST7735_DrawString(0, 0, "Hello LCD!", GREEN);
	
	while(1){
		
	//drawInfoBar();

	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1,1);
	//sensorValue =  HAL_ADC_GetValue(&hadc1);
	
	//ST7735_DrawString(uint16_t x, uint16_t y, char *pt, int16_t textColor)
	//ST7735_DrawString(0, 0, "Hello LCD!", GREEN);

	
	}
	
}

void plotData(void)
{
	ST7735_PlotPoint(sensorValue,GREEN);
	ST7735_PlotIncrement();
	
	
}
void drawaxes(void){
    ST7735_Drawaxes(AXISCOLOR, BGCOLOR, "Time", "ADC", LIGHTCOLOR, "", 0, YMAX, YMIN);
}

void drawInfoBar(void)
{
    ST7735_DrawString(1, 0, "CPU =", GREEN);

	    ST7735_DrawString(7, 0, "75%", BLUE );
    ST7735_DrawString(11, 0, "Temp =", GREEN);
   	    ST7735_DrawString(18, 0, "30", BLUE );



}

void SysTick_Handler(void)
{
	HAL_IncTick();

}

