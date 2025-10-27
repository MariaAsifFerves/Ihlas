#include "apm32f00x_int.h"
#include "main.h"

extern _Bool FLOW_start;
 void NMI_Handler(void)
 {
	 
 }
void HardFault_Handler(void)
{

}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{

}
void TMR4_IRQHandler(void)
{
    TMR4Isr();    					//C99 standartinda gecersiz
}
void EINTC_IRQHandler(void)
{
	if(FLOW_start)
	{Flow_Pulse();}    				//C99 standartinda gecersiz
}
void ADC_IRQHandler(void)
{
    
}

